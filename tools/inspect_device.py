#!/usr/bin/env python3
"""只读采集 R1-D 的 rt/lowstate：全部槽位、CRC 校验、无 writer。

本脚本没有创建 rt/lowcmd publisher 的代码路径，也不切换任何服务。
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from lowstate import slot_rows, summary  # noqa: E402


def capture(network, domain, seconds, min_packets):
    from dds_network import initialize
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.utils.crc import CRC

    initialize(network, domain)
    reader = ChannelSubscriber("rt/lowstate", LowState_)
    reader.Init()
    crc = CRC()
    latest, valid, bad_crc, ticks = None, 0, 0, set()
    started = time.monotonic()
    try:
        while time.monotonic() - started < seconds:
            candidate = reader.Read()
            if candidate is None:
                time.sleep(.002)
                continue
            if int(candidate.crc) != crc.Crc(candidate):
                bad_crc += 1
                continue
            latest = candidate
            valid += 1
            ticks.add(int(candidate.tick))
            if valid >= min_packets and len(ticks) >= 2:
                break
    finally:
        reader.Close()
    if latest is None:
        raise SystemExit(f"未收到有效状态：valid={valid}, bad_crc={bad_crc}。"
                         "检查网卡、DDS 域，或机器人是否上电；未发送任何指令")
    elapsed = time.monotonic() - started
    report = {
        "scope": "READ ONLY; 未创建 rt/lowcmd writer，未切换运控",
        "captured_at": datetime.now(timezone.utc).astimezone().isoformat(),
        "network": network, "domain": domain,
        "valid_packets": valid, "bad_crc": bad_crc,
        "wall_seconds": round(elapsed, 3), "approx_rate_hz": round(valid / max(elapsed, 1e-9), 1),
        "unique_ticks": len(ticks),
        "tick_first": min(ticks), "tick_last": max(ticks),
        "mode_pr": int(latest.mode_pr), "mode_machine": int(latest.mode_machine),
        "slot_count": len(latest.motor_state),
        "slots": slot_rows(latest),
    }
    report["summary"] = summary(report["slots"])
    return report


def print_summary(report):
    rows = report["summary"]
    motor = [r for r in rows if r["signal"] == "motor"]
    power = [r for r in rows if r["signal"] == "power"]
    print(f"有效包 {report['valid_packets']}，坏 CRC {report['bad_crc']}，"
          f"约 {report['approx_rate_hz']} Hz，tick {report['tick_first']}→{report['tick_last']}，"
          f"mode_machine={report['mode_machine']}，槽位总数 {report['slot_count']}")
    print(f"有电机信号的槽位 {len(motor)} 个：{[r['slot'] for r in motor]}")
    print(f"仅供电/温度、无控制信号的槽位 {len(power)} 个：{[r['slot'] for r in power]}")
    print(f"{'槽位':>4} {'信号':>6} {'mode':>4} {'q(deg)':>9} {'dq':>8} {'tau':>8} "
          f"{'vol':>6} {'temp':>5} {'err':>4}")
    for row in motor + power:
        print(f"{row['slot']:>4} {row['signal']:>6} {row['mode']:>4} "
              f"{row['q_deg']:>9.3f} {row['dq']:>8.3f} {row['tau_est']:>8.3f} "
              f"{row['vol']:>6.2f} {row['temp_C']:>5.1f} {row['error']:>4}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", required=True, help="物理网卡名，如 eth0；本机回环用 lo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=5.0, help="最长采集秒数")
    parser.add_argument("--min-packets", type=int, default=20)
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "device.json")
    args = parser.parse_args()
    if not (0 < args.seconds <= 60) or args.min_packets < 1:
        parser.error("seconds 需在 (0, 60]，min-packets ≥ 1")
    report = capture(args.network, args.domain, args.seconds, args.min_packets)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print_summary(report)
    print(f"已保存 {args.report}；未发送运动指令。", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"采集失败: {exc}", file=sys.stderr)
        raise SystemExit(2)
