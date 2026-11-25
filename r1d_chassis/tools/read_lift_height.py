#!/usr/bin/env python3
"""只读订阅 R1-A7-D 的立柱高度、速度和告警。"""
import argparse
from bisect import bisect_left
from contextlib import ExitStack
from itertools import groupby
import json
import math
from pathlib import Path
import sys
import time

from unitree_sdk2py.core.channel import ChannelSubscriber

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "r1d_scan"))
from dds_network import initialize
from lift_dds import (ALARM_TOPIC, HEIGHT_TOPIC, VELOCITY_TOPIC, Error_,
                      Point32_, Twist_, raw_height_mm)


def normalized_velocity(message):
    velocity = float(message.linear.z)
    if not math.isfinite(velocity):
        raise ValueError("速度反馈含非有限值")
    return velocity


class Samples:
    def __init__(self, decode):
        self.decode = decode
        self.records = []
        self.invalid = 0

    def receive(self, message):
        now = time.monotonic()
        try:
            value = self.decode(message)
        except ValueError:
            self.invalid += 1
            return
        self.records.append((now, value))

    def summary(self):
        count = len(self.records)
        span = self.records[-1][0] - self.records[0][0] if count > 1 else 0.0
        return {"samples": count, "invalid": self.invalid,
                "rate_hz": round((count - 1) / span, 2) if span > 0 else None}


def alarm_events(alarm_records, height_records, started):
    """按连续告警分组，附上时间相近的高度样本。"""
    height_times = [t for t, _ in height_records]

    def nearest_height(t):
        i = bisect_left(height_times, t)
        candidates = height_records[max(0, i - 1):i + 1]
        if not candidates:
            return None
        timestamp, value = min(candidates, key=lambda record: abs(record[0] - t))
        return round(value, 2) if abs(timestamp - t) <= 0.1 else None

    events = []
    for (source, state), group in groupby(alarm_records, key=lambda record: record[1]):
        if not state:
            continue
        times = [t for t, _ in group]
        events.append({"source": source, "state": state, "samples": len(times),
                       "first_seen_s": round(times[0] - started, 3),
                       "last_seen_s": round(times[-1] - started, 3),
                       "first_height_mm": nearest_height(times[0]),
                       "last_height_mm": nearest_height(times[-1])})
    return events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=10.0,
                        help="采集秒数。0 表示持续采集，按 Ctrl+C 结束并保存")
    parser.add_argument("--report", type=Path, help="将诊断摘要另存为 JSON")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds 必须为有限非负数")

    height = Samples(raw_height_mm)
    velocity = Samples(normalized_velocity)
    alarm = Samples(lambda message: (int(message.source), int(message.state)))
    initialize(args.network, args.domain)
    duration = f"持续 {args.seconds:g} 秒" if args.seconds else "按 Ctrl+C 结束采集"
    print(f"只读监听 {args.network} / domain {args.domain}，{duration}", flush=True)
    started = time.monotonic()
    with ExitStack() as resources:
        for topic, kind, samples in ((HEIGHT_TOPIC, Point32_, height),
                                     (VELOCITY_TOPIC, Twist_, velocity),
                                     (ALARM_TOPIC, Error_, alarm)):
            reader = ChannelSubscriber(topic, kind)
            reader.Init(samples.receive, 1)
            resources.callback(reader.Close)

        end = time.monotonic() + args.seconds if args.seconds else math.inf
        next_print = 0.0
        try:
            while (remaining := end - time.monotonic()) > 0:
                if height.records and time.monotonic() >= next_print:
                    now, value = height.records[-1]
                    age = time.monotonic() - now
                    print(f"{HEIGHT_TOPIC}.z = {value:.2f} mm，反馈年龄 {age:.2f} s", flush=True)
                    next_print = time.monotonic() + 1.0
                time.sleep(min(0.05, remaining))
        except KeyboardInterrupt:
            pass

    heights = [value for _, value in height.records]
    velocities = [value for _, value in velocity.records]
    alarms = sorted({value for _, value in alarm.records if value[1]})
    result = {
        "network": args.network,
        "domain": args.domain,
        HEIGHT_TOPIC: {**height.summary(),
                       "last_mm": heights[-1] if heights else None,
                       "min_mm": min(heights) if heights else None,
                       "max_mm": max(heights) if heights else None,
                       "span_mm": max(heights) - min(heights) if heights else None},
        VELOCITY_TOPIC: {**velocity.summary(),
                         "min_normalized_z": min(velocities) if velocities else None,
                         "max_normalized_z": max(velocities) if velocities else None},
        ALARM_TOPIC: {**alarm.summary(), "nonzero_source_state_pairs": alarms,
                      "events": alarm_events(alarm.records, height.records, started)},
    }
    payload = json.dumps(result, indent=2, ensure_ascii=False)
    print(payload, flush=True)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(payload + "\n", encoding="utf-8")
    if not heights:
        print("本次监听未收到有效立柱高度。", file=sys.stderr)
    return 0 if heights else 2


if __name__ == "__main__":
    raise SystemExit(main())
