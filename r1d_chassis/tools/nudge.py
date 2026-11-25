#!/usr/bin/env python3
"""接管真机做一个"看得见的小动作"——用来确认：能发指令、关节会动、方向对不对。

它**不是**扫描程序：只动一个关节、幅度小（默认 8°）、速度慢（约 0.15 rad/s）、
时间短（默认 6 s），其余槽位全部按"启动时实测角"按住不动。

时序：
    0 – 1 s     保持（抓住手臂，先让电机从阻尼切到位置伺服）
    1 – 3 s     缓慢转到 +A°
    3 – 4 s     停一下（肉眼确认）
    4 – 6 s     缓慢回到原位
    6 – 7 s     保持
    然后停止发送（电机继续执行最后一条指令，保持姿态）

保护（每一拍都查，任一触发就停发）：
    状态超时 0.2 s / CRC / mode_machine 变化 / 目标偏离起点超过 envelope /
    预计力矩超过 90% effort / 控制周期异常

用法：
  python3 tools/nudge.py --network ens33 --domain 0 --joint left_elbow_joint --dry-run
  python3 tools/nudge.py --network ens33 --domain 0 --joint left_elbow_joint
"""
import argparse
import csv
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from profile import load as load_profile  # noqa: E402
from transport import Codec, ControlFault  # noqa: E402

DEG = 180.0 / np.pi


def smooth(t):
    """0→1 的五次曲线，两端速度为零（避免起步/收尾的冲击）。"""
    t = np.clip(t, 0.0, 1.0)
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def plan(elapsed, amp, scale=1.0):
    """返回 (阶段, 相对起点的偏移比例 -1..1)。"""
    if elapsed < 1.0:
        return "抓取", 0.0
    if elapsed < 3.0:
        return "出去", scale * smooth((elapsed - 1.0) / 2.0)
    if elapsed < 4.0:
        return "停一下", scale * 1.0
    if elapsed < 6.0:
        return "回来", scale * (1.0 - smooth((elapsed - 4.0) / 2.0))
    if elapsed < 7.0:
        return "保持", 0.0
    return "结束", 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--network", required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--joint", default="left_elbow_joint",
                        help="要动的关节名（必须在控制集里）")
    parser.add_argument("--amplitude-deg", type=float, default=8.0)
    parser.add_argument("--rate", type=float, default=200.0, help="发送频率 Hz")
    parser.add_argument("--envelope-deg", type=float, default=20.0,
                        help="目标允许偏离起点的最大角度，超过就停")
    parser.add_argument("--log", type=Path, default=ROOT / "out" / "nudge.csv")
    parser.add_argument("--dry-run", action="store_true", help="只打印计划，不接 DDS")
    args = parser.parse_args()

    profile = load_profile()
    codec = Codec(profile=profile)
    names = codec.joints
    if args.joint not in names:
        raise SystemExit(f"{args.joint} 不在控制集里；可选：{names}")
    index = names.index(args.joint)
    amp = np.radians(args.amplitude_deg)

    if args.dry_run:
        print(f"计划：{args.joint}（槽位 {codec.slots[index]}）从起点偏移 0 → "
              f"{args.amplitude_deg:+.1f}° → 0，共 7 s，发送 {args.rate:.0f} Hz")
        for t in np.arange(0, 7.01, 0.5):
            phase, s = plan(float(t), amp)
            print(f"  t={t:4.1f}s  {phase:<6} 偏移 {np.degrees(s * amp):+6.2f}°")
        return 0

    from dds_network import initialize
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

    initialize(args.network, args.domain)
    cache = {"state": None, "good": 0, "bad": 0}

    def on_state(msg):
        try:
            cache["state"] = codec.decode(msg, time.monotonic(), cache["state"])
            cache["good"] += 1
        except ControlFault:
            cache["bad"] += 1

    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(on_state, 1)
    deadline = time.monotonic() + 10.0
    while cache["good"] < 20 and time.monotonic() < deadline:
        time.sleep(0.01)
    if cache["good"] < 20:
        raise SystemExit(f"没有有效 LowState（good={cache['good']}, bad={cache['bad']}）")

    state = cache["state"]
    q0 = state.q.copy()
    hold0 = np.asarray(state.hold_q, float).copy()
    machine = state.mode_machine

    insane = [f"槽位{s}={np.degrees(v):.0f}°" for s, v in zip(codec.hold_slots, hold0)
              if not -np.pi <= v <= np.pi]
    if insane:
        raise SystemExit("保持关节实测角不合理，拒绝启动：" + "、".join(insane))
    codec.hold_targets = hold0

    effort = np.array([80.0] * len(names))     # 粗略上限，真正判据用 profile 的 effort
    raw = __import__("json").loads((ROOT / "r1d_scan" / "robot_profile.json")
                                   .read_text(encoding="utf-8"))
    effort = np.array([j["effort"] for j in raw["joints"]], float)

    print(f"捕获：{args.joint} = {np.degrees(q0[index]):+.2f}°，"
          f"mode_machine={machine}，准备动作幅度 {args.amplitude_deg:+.1f}°", flush=True)
    print("3 秒后开始，手放在急停上；Ctrl-C 可立即中止……", flush=True)
    time.sleep(3.0)

    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    args.log.parent.mkdir(parents=True, exist_ok=True)
    log = args.log.open("w", newline="", encoding="utf-8")
    writer = csv.writer(log)
    writer.writerow(["t", "phase", "q_target_deg", "q_actual_deg", "tau_est", "sent"])

    period = 1.0 / args.rate
    started = time.monotonic()
    last = started
    sent = 0
    phase_seen = None
    fault = None
    try:
        while True:
            now = time.monotonic()
            elapsed = now - started
            phase, scale = plan(elapsed, amp)
            if phase == "结束":
                break
            st = cache["state"]
            if st is None:
                raise ControlFault("没有 LowState")
            if now - st.received > 0.2:
                raise ControlFault(f"状态超时 {now - st.received:.2f}s")
            if st.mode_machine != machine:
                raise ControlFault(f"mode_machine 变化 {st.mode_machine} != {machine}")
            target = q0.copy()
            target[index] = q0[index] + scale * amp
            if np.max(np.abs(target - q0)) > np.radians(args.envelope_deg):
                raise ControlFault("目标超出允许范围")
            pd = codec.kp * (target - st.q) - codec.kd * st.dq
            if np.max(np.abs(pd) / effort) > 0.9:
                raise ControlFault(f"预计力矩超预算 {np.max(np.abs(pd) / effort):.2f}")
            if not publisher.Write(codec.encode(target, st)):
                raise ControlFault("LowCmd 写入失败")
            sent += 1
            dt = now - last
            if dt <= 0 or dt > 0.1:
                raise ControlFault(f"控制周期异常 {dt*1000:.0f} ms")
            last = now
            if phase != phase_seen:
                phase_seen = phase
                print(f"  [{elapsed:4.1f}s] {phase}  "
                      f"目标 {args.joint} = {np.degrees(target[index]):+.1f}°  "
                      f"实际 {np.degrees(st.q[index]):+.1f}°", flush=True)
            if int(elapsed * 10) % 2 == 0:
                writer.writerow([round(elapsed, 3), phase,
                                 round(float(np.degrees(target[index])), 3),
                                 round(float(np.degrees(st.q[index])), 3),
                                 round(float(st.tau[index]), 3), sent])
            slack = period - (time.monotonic() - now)
            if slack > 0:
                time.sleep(slack)
    except ControlFault as exc:
        fault = str(exc)
        print(f"\n保护触发，停止发送：{fault}", flush=True)
    except KeyboardInterrupt:
        fault = "人工中止"
        print("\n人工中止，停止发送", flush=True)
    finally:
        log.close()
        subscriber.Close()
        publisher.Close()

    final = cache["state"]
    print(f"\n共发送 {sent} 条指令，用时 {time.monotonic() - started:.1f} s")
    if final is not None:
        moved = np.degrees(final.q[index] - q0[index])
        print(f"{args.joint} 从 {np.degrees(q0[index]):+.1f}° 到 {np.degrees(final.q[index]):+.1f}°，"
              f"净位移 {moved:+.1f}°")
    print("已停止发送；电机继续执行最后一条指令，保持当前姿态。")
    return 0 if fault is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
