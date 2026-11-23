#!/usr/bin/env python3
"""只读监听 rt/lowcmd：看看**现在还有没有别人在发指令**。

为什么必须查：Unitree 的原厂运控在 500 Hz 往 `rt/lowcmd` 写指令。如果它还在跑，
我们再往同一个话题写，就是两个写者抢一个通道——电机收到的是"最后到达的那包"，
结果不可预测（抖动、猛拽）。**低层接管的第一个动作不是发指令，而是确认通道是空的。**

本脚本只订阅、不发布（文件里搜不到 publisher 类名）。

用法：
  python3 tools/watch_lowcmd.py --network ens33 --domain 0 --seconds 5
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from profile import load as load_profile  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--network", required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "lowcmd_watch.json")
    args = parser.parse_args()

    from dds_network import initialize
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.utils.crc import CRC

    initialize(args.network, args.domain)
    reader = ChannelSubscriber("rt/lowcmd", LowCmd_)
    crc = CRC()
    state = {"packets": 0, "bad_crc": 0, "slots": {}, "modes": set(), "machines": set()}

    def on_cmd(msg):
        # 用回调而不是 Read()：没人发指令时 Read() 会一直阻塞，监听就卡死了
        if int(msg.crc) != crc.Crc(msg):
            state["bad_crc"] += 1
            return
        state["packets"] += 1
        state["modes"].add(int(msg.mode_pr))
        state["machines"].add(int(msg.mode_machine))
        for slot, motor in enumerate(msg.motor_cmd):
            if float(motor.kp) > 0.1 or float(motor.kd) > 0.1:
                state["slots"][slot] = (float(motor.q), float(motor.kp), float(motor.kd))

    reader.Init(on_cmd, 1)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            time.sleep(0.01)
    finally:
        reader.Close()
    elapsed = time.monotonic() - started
    packets, bad_crc = state["packets"], state["bad_crc"]
    active_slots = state["slots"]
    modes, machines = state["modes"], state["machines"]

    profile = load_profile()
    ours = {j.slot for j in profile.joints} | {j.slot for j in profile.hold_joints}
    others = sorted(s for s in active_slots if s not in ours)

    result = {
        "network": args.network, "domain": args.domain,
        "packets": packets, "bad_crc": bad_crc,
        "rate_hz": round(packets / max(elapsed, 1e-9), 1),
        "mode_pr": sorted(modes), "mode_machine": sorted(machines),
        "active_slots": {str(s): {"q": round(v[0], 4), "kp": v[1], "kd": v[2]}
                         for s, v in sorted(active_slots.items())},
        "slots_outside_ours": others,
        "verdict": "空闲（没人发指令，可以接管）" if packets == 0 else (
            "原厂运控在发指令 → 先别接管" if others else
            "只有本程序的槽位在动（可能是上一次没关掉的程序）"),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"监听 rt/lowcmd {elapsed:.1f} s：收到 {packets} 包（{result['rate_hz']} Hz），"
          f"坏 CRC {bad_crc}，mode_pr {result['mode_pr']}，mode_machine {result['mode_machine']}")
    if packets == 0:
        print("==> 通道空闲：没有人在发指令，可以接管", flush=True)
    else:
        print(f"==> 有人在发指令，涉及 {len(active_slots)} 个槽位（kp/kd > 0）")
        if others:
            print(f"    其中 {len(others)} 个不在本程序的 14 个槽位里：{others}")
            print("    说明原厂运控（或别的程序）还握着控制权 —— 现在接管会两个写者抢通道")
            for slot in others[:12]:
                q, kp, kd = active_slots[slot]
                print(f"      槽位{slot:>3}: q={np.degrees(q):+7.1f}°  kp={kp:6.1f}  kd={kd:5.2f}")
        else:
            print("    只在我们的 14 个槽位上有指令：大概率是上次没退干净的进程")
    print(f"已保存 {args.report}；本脚本没有发送任何指令。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
