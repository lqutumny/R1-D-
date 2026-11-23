#!/usr/bin/env python3
"""阶段 5a：只读采集 R1-D 的 rt/lowstate —— 全部槽位、CRC 校验、统计一段时间。

**本脚本不 import 任何 publisher 类、不创建 rt/lowcmd 通道**（整个文件里搜不到那个类名，
这就是可检查的保证），也不切换任何服务、不改机器人状态。它只做一件事：听。

为什么要统计而不是取一帧：
  一帧只能告诉你"此刻角度是多少"，看不出这个关节是**稳稳停住**还是在**缓慢下垂**。
  采集窗口内每槽位的 q 均值/极差/噪声、dq 均值、|tau| 均值，才能回答：
  - 这个槽位是不是真有电机在闭环出力（|tau| 明显非零 / dq≈0 且 q 不漂）
  - 右臂保持指令如果没发，它会不会自己往下掉（q 极差大 = 掉了）
  - 有没有槽位在抖（q 噪声大）

用法：
  python3 tools/inspect_device.py --network eth0 --seconds 5      # 真机
  python3 tools/inspect_device.py --network lo  --seconds 3       # 本机对着仿真器/假机器人
"""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from lowstate import slot_rows, summary  # noqa: E402
from profile import load as load_profile  # noqa: E402

DEG = 180.0 / np.pi


def capture(network, domain, seconds, min_packets):
    from dds_network import initialize
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.utils.crc import CRC

    initialize(network, domain)
    reader = ChannelSubscriber("rt/lowstate", LowState_)
    reader.Init()
    crc = CRC()
    latest, valid, bad_crc = None, 0, 0
    ticks = set()
    samples = defaultdict(list)          # slot -> [(q, dq, tau, temp, mode, error), ...]
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
            for i, motor in enumerate(candidate.motor_state):
                samples[i].append((float(motor.q), float(motor.dq), float(motor.tau_est),
                                   float(max(motor.temperature)), int(motor.mode),
                                   int(motor.motorstate)))
            if valid >= min_packets and len(ticks) >= 2 and time.monotonic() - started >= seconds:
                break
    finally:
        reader.Close()
    if latest is None:
        raise SystemExit(f"未收到有效状态：valid={valid}, bad_crc={bad_crc}。"
                         "检查网卡、DDS 域，或机器人是否上电；未发送任何指令")
    if valid < min_packets:
        raise SystemExit(f"有效包只有 {valid} 个（要求 ≥{min_packets}），样本太少；未发送任何指令")
    elapsed = time.monotonic() - started

    stats = []
    for slot, rows in sorted(samples.items()):
        q = np.array([r[0] for r in rows], float)
        dq = np.array([r[1] for r in rows], float)
        tau = np.array([r[2] for r in rows], float)
        temp = np.array([r[3] for r in rows], float)
        mode = np.array([r[4] for r in rows], int)
        error = np.array([r[5] for r in rows], int)
        stats.append({
            "slot": slot, "samples": len(rows),
            "q_deg_mean": round(float(q.mean()) * DEG, 3),
            "q_deg_min": round(float(q.min()) * DEG, 3),
            "q_deg_max": round(float(q.max()) * DEG, 3),
            "q_deg_range": round(float(q.max() - q.min()) * DEG, 3),
            "q_deg_std": round(float(q.std()) * DEG, 4),
            "dq_mean": round(float(dq.mean()), 4),
            "tau_mean": round(float(tau.mean()), 3),
            "tau_abs_max": round(float(np.abs(tau).max()), 3),
            "temp_max": round(float(temp.max()), 1),
            "mode_max": int(mode.max()),
            "error_any": int(error.max()),
        })

    report = {
        "scope": "READ ONLY；未 import 任何 publisher 类，未发任何指令",
        "captured_at": datetime.now(timezone.utc).astimezone().isoformat(),
        "network": network, "domain": domain,
        "valid_packets": valid, "bad_crc": bad_crc,
        "wall_seconds": round(elapsed, 3),
        "approx_rate_hz": round(valid / max(elapsed, 1e-9), 1),
        "unique_ticks": len(ticks),
        "tick_first": min(ticks), "tick_last": max(ticks),
        "mode_pr": int(latest.mode_pr), "mode_machine": int(latest.mode_machine),
        "slot_count": len(latest.motor_state),
        "slots": slot_rows(latest),          # 最后一帧原样，给 analyze_device 用
        "slot_stats": stats,                 # 采集窗口内的统计
    }
    report["summary"] = summary(report["slots"])
    return report


def compare_with_profile(report, profile, raw):
    """把本程序关心的槽位（控制 7 + 保持 7）和期望姿态比一比。"""
    expected = {}
    expected.update({k: v for k, v in (raw.get("initial") or {}).items() if k != "note"})
    expected.update({k: v for k, v in (raw.get("held") or {}).items() if k != "note"})
    by_slot = {row["slot"]: row for row in report["slot_stats"]}
    rows = []
    pairs = [(j, "控制") for j in profile.joints] + [(j, "保持") for j in profile.hold_joints]
    for joint, role in pairs:
        stat = by_slot.get(joint.slot)
        want = expected.get(joint.name)
        row = {"slot": joint.slot, "name": joint.name, "role": role,
               "expected_deg": None if want is None else round(want * DEG, 1)}
        if stat:
            row.update({k: stat[k] for k in ("q_deg_mean", "q_deg_range", "q_deg_std",
                                             "dq_mean", "tau_mean", "tau_abs_max",
                                             "temp_max", "mode_max", "error_any")})
            row["alive"] = stat["mode_max"] != 0 or abs(stat["q_deg_mean"]) > 1e-3 \
                or stat["q_deg_std"] > 1e-4 or stat["tau_abs_max"] > 1e-3
            if want is not None:
                row["delta_deg"] = round(stat["q_deg_mean"] - want * DEG, 1)
        else:
            row["alive"] = False
        rows.append(row)
    return rows


def _pad(text, width):
    """按显示宽度补空格（中文算 2 格），否则终端里表格会歪。"""
    import unicodedata
    used = sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in text)
    return text + " " * max(0, width - used)


def print_report(report, rows, profile):
    print(f"采集 {report['wall_seconds']:.1f} s：有效包 {report['valid_packets']}，"
          f"坏 CRC {report['bad_crc']}，约 {report['approx_rate_hz']} Hz，"
          f"mode_pr={report['mode_pr']} mode_machine={report['mode_machine']}，"
          f"槽位总数 {report['slot_count']}")

    known = {j.slot for j in profile.joints} | {j.slot for j in profile.hold_joints}
    others = [s for s in report["slot_stats"]
              if s["slot"] not in known and (s["mode_max"] or abs(s["q_deg_mean"]) > 0.01
                                             or s["tau_abs_max"] > 0.01)]
    print(f"\n本程序关心的槽位（{len(rows)} 个 = 控制 7 + 保持 7）：")
    header = (f"{_pad('槽位', 5)}{_pad('角色', 6)}{_pad('关节名', 30)}"
              f"{_pad('活着', 6)}{_pad('q均值', 9)}{_pad('极差', 8)}{_pad('噪声', 8)}"
              f"{_pad('|tau|峰', 9)}{_pad('temp', 6)}{_pad('err', 5)}期望/偏差")
    print(header)
    for row in rows:
        head = (f"{row['slot']:>4} {_pad(row['role'], 6)}{_pad(row['name'], 30)}")
        if not row.get("alive"):
            print(head + _pad("否", 6) + "—— 没有信号（检查槽位号/是否上电）")
            continue
        delta = row.get("delta_deg")
        want = row.get("expected_deg")
        flag = "  ← 偏差>5°" if (delta is not None and abs(delta) > 5) else ""
        tail = "" if want is None else f"期望 {want:+.1f}° / 偏差 {delta:+.1f}°"
        print(head + _pad("是", 6)
              + _pad(f"{row['q_deg_mean']:+.1f}°", 9)
              + _pad(f"{row['q_deg_range']:.2f}°", 8)
              + _pad(f"{row['q_deg_std']:.3f}", 8)
              + _pad(f"{row['tau_abs_max']:.2f}", 9)
              + _pad(f"{row['temp_max']:.0f}", 6)
              + _pad(str(row["error_any"]), 5) + tail + flag)

    if others:
        print(f"\nprofile 之外有信号的槽位（{len(others)} 个，多半是腰/头/腿）：")
        for s in others:
            print(f"{s['slot']:>4}  q={s['q_deg_mean']:>8.1f}°  极差 {s['q_deg_range']:>6.2f}°  "
                  f"|tau|峰 {s['tau_abs_max']:>6.2f}  mode={s['mode_max']}  err={s['error_any']}")
        print("  → 这些槽位要不要由本程序保持，取决于现场核对（腰 yaw 是 12 还是 13）")

    bad = [r for r in rows if r.get("alive") and r.get("delta_deg") is not None
           and abs(r["delta_deg"]) > 5]
    if bad:
        print(f"\n⚠ {len(bad)} 个关节与期望姿态差 >5°："
              + "、".join(f"{r['name']}({r['delta_deg']:+.1f}°)" for r in bad))
        print("  要么机器人不是下垂姿态，要么方向/零位与 URDF 不一致——这正是核对要查的")
    else:
        print("\n✓ 关心槽位的角度与期望姿态一致（±5° 内）")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--network", required=True, help="物理网卡名，如 eth0；本机回环用 lo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=5.0, help="最长采集秒数")
    parser.add_argument("--min-packets", type=int, default=20)
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "device.json")
    args = parser.parse_args()
    if not (0 < args.seconds <= 60) or args.min_packets < 1:
        parser.error("seconds 需在 (0, 60]，min-packets ≥ 1")

    profile = load_profile()
    raw = json.loads((ROOT / "r1d_scan" / "robot_profile.json").read_text(encoding="utf-8"))
    report = capture(args.network, args.domain, args.seconds, args.min_packets)
    rows = compare_with_profile(report, profile, raw)
    report["profile_check"] = rows
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print_report(report, rows, profile)
    print(f"\n已保存 {args.report}；未发送任何运动指令。", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(f"采集失败: {exc}", file=sys.stderr)
        raise SystemExit(2)
