#!/usr/bin/env python3
"""隔离实验：直接给单个关节发慢斜坡指令，记录仿真器的实际响应。

绕开状态机与控制器，用来判断"跟不动"是仿真器/DDS 的问题，还是控制器的问题。
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "r1d_scan"))

from dds_network import initialize  # noqa: E402
from profile import load as load_profile  # noqa: E402
from transport import Codec, ControlFault  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--joint", default="waist_yaw_joint")
    parser.add_argument("--delta", type=float, default=-0.5, help="目标增量 rad")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--rate", type=float, default=500.0, help="指令发送频率 Hz")
    parser.add_argument("--log", type=Path, default=ROOT / "out" / "probe_step.csv")
    args = parser.parse_args()

    profile = load_profile()
    codec = Codec(profile=profile)
    names = profile.names()
    index = names.index(args.joint)
    initialize(args.network, args.domain)
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

    cache = {"state": None, "good": 0}

    def on_state(msg):
        try:
            cache["state"] = codec.decode(msg, time.monotonic(), cache["state"])
            cache["good"] += 1
        except ControlFault:
            pass

    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(on_state, 1)
    deadline = time.monotonic() + 10
    while cache["good"] < 20 and time.monotonic() < deadline:
        time.sleep(0.01)
    if cache["good"] < 20:
        raise SystemExit("没有 LowState；仿真器是否启动？")
    start_q = cache["state"].q.copy()
    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()

    args.log.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    period = 1.0 / args.rate
    t0 = time.monotonic()
    deadline_t = t0
    while True:
        now = time.monotonic()
        elapsed = now - t0
        if elapsed > args.seconds:
            break
        u = min(1.0, elapsed / args.seconds)
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        target = start_q.copy()
        target[index] = start_q[index] + args.delta * s
        publisher.Write(codec.encode(target, cache["state"]))
        state = cache["state"]
        rows.append({"t": round(elapsed, 3), "target": round(float(target[index]), 4),
                     "actual": round(float(state.q[index]), 4),
                     "lag": round(float(target[index] - state.q[index]), 4)})
        deadline_t += period
        slack = deadline_t - time.monotonic()
        if slack > 0:
            time.sleep(slack)
        else:
            deadline_t = time.monotonic()
    subscriber.Close()
    publisher.Close()

    with args.log.open("w", encoding="utf-8") as handle:
        handle.write("t,target,actual,lag\n")
        for row in rows:
            handle.write(f"{row['t']},{row['target']},{row['actual']},{row['lag']}\n")
    final = rows[-1]
    max_lag = max(abs(r["lag"]) for r in rows)
    print(f"关节 {args.joint}: 目标 {final['target']:+.3f}，实际 {final['actual']:+.3f}，"
          f"最终误差 {final['lag']:+.3f} rad，最大误差 {max_lag:.3f} rad，样本 {len(rows)}")
    for r in rows[::max(1, len(rows)//10)]:
        print(f"  t={r['t']:5.2f}s  target={r['target']:+.3f}  actual={r['actual']:+.3f}  lag={r['lag']:+.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
