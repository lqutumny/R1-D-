#!/usr/bin/env python3
"""假机器人（DDS）：订阅 rt/lowcmd，按"完美跟踪"发布 rt/lowstate，并可按需注入故障。

为什么需要它：真实的 MuJoCo 仿真器只会给出健康状态，造不出
CRC 错误 / 丢包 / 故障字 / 高温 / 模式变化 这些坏状态，
而 Gate 4 要求"每个保护项都有注入测试"。所以故障由这个假机器人来演。

"完美跟踪"= 把收到的指令当作实际关节角原样回报，跟踪误差恒为 0；
这样测试前段不会误触跟踪保护，能干净地只测目标那一项。

用法：
  python3 r1d_scan/fake_robot.py --fault error --at 5
  python3 r1d_scan/fake_robot.py --fault none --duration 70
"""
import argparse
import json
from pathlib import Path
import signal
import time

import numpy as np

from dds_network import initialize
from profile import load as load_profile

ROOT = Path(__file__).resolve().parents[1]
PROFILE_JSON = ROOT / "r1d_scan" / "robot_profile.json"
FAULTS = ("none", "crc", "timeout", "tick", "nan", "error", "temp", "mode")


class FakeRobot:
    """最小可信机器人：只实现"收到指令就到位"和"按需报坏状态"。"""

    def __init__(self, fault="none", at=5.0, mode_machine=1):
        self.fault = fault
        self.fault_time = float(at)
        self.mode_machine = mode_machine
        self.profile = load_profile()
        self.slots = np.array([j.slot for j in self.profile.joints], int)
        self.names = [j.name for j in self.profile.joints]
        raw = json.loads(PROFILE_JSON.read_text(encoding="utf-8"))
        self.initial = {name: angle for name, angle in raw["initial"].items() if name != "note"}
        self.q = {int(slot): 0.0 for slot in self.slots}
        for i, name in enumerate(self.names):
            self.q[int(self.slots[i])] = float(self.initial.get(name, 0.0))
        # 保持关节（右臂）也要报真实角度，否则控制端"照原样写回"会写成 0°
        held = raw.get("held") or {}
        self.hold_names = [j.name for j in getattr(self.profile, "hold_joints", ())]
        self.hold_slots = np.array([j.slot for j in getattr(self.profile, "hold_joints", ())], int)
        self.hold_q = {int(s): float(held.get(n, 0.0)) for n, s in zip(self.hold_names, self.hold_slots)}
        self.dq = {int(slot): 0.0 for slot in self.slots}
        self.command = None
        self.last_cmd = time.monotonic()
        self.accepted = 0
        self.rejected = 0
        self.last_reject = ""
        self.started = time.monotonic()
        self.fault_announced = False
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.utils.crc import CRC
        self.state_factory = unitree_hg_msg_dds__LowState_
        self.crc = CRC()

    # ---------- 收指令 ----------
    def accept(self, msg, now=None):
        try:
            from transport import Codec
            if not hasattr(self, "_codec"):
                self._codec = Codec(profile=self.profile)
            if int(msg.crc) != self._codec.crc.Crc(msg):
                raise ValueError("CRC 错误")
            if int(msg.mode_pr) != 0 or int(msg.mode_machine) != self.mode_machine:
                raise ValueError(f"模式不匹配 mode_pr={msg.mode_pr} "
                                 f"mode_machine={msg.mode_machine}")
            q = np.array([msg.motor_cmd[int(s)].q for s in self.slots], float)
            if not np.isfinite(q).all():
                raise ValueError("指令含非有限值")
            now = time.monotonic() if now is None else now
            dt = max(now - self.last_cmd, 1e-4)
            self.last_cmd = now
            for i, slot in enumerate(self.slots):
                # 速度必须是"位移 / 真实耗时"。曾经写成固定的 2 ms，
                # DDS 抖动把几条指令攒在一起送达时，dq 会被放大成 3 rad/s 的假高速，
                # 直接把力矩预算保护误触了（关节 5，-9.2 N·m）。
                raw = (q[i] - self.q[int(slot)]) / dt
                self.dq[int(slot)] = float(np.clip(raw, -2.0, 2.0))
                self.q[int(slot)] = float(q[i])
            self.command = q
            self.accepted += 1
            return True
        except Exception as exc:  # noqa: BLE001
            self.rejected += 1
            self.last_reject = str(exc)
            return False

    # ---------- 发状态 ----------
    def active_fault(self, now):
        if self.fault == "none":
            return "none"
        return self.fault if now - self.started >= self.fault_time else "none"

    def state(self, now):
        fault = self.active_fault(now)
        state = self.state_factory()
        state.mode_pr = 0
        state.mode_machine = self.mode_machine
        tick = int(round((now - self.started) * 1000)) % 2**32
        for slot in self.slots:
            motor = state.motor_state[int(slot)]
            motor.mode = 1
            motor.q = float(self.q[int(slot)])
            motor.dq = float(self.dq[int(slot)])
            motor.ddq = 0.0
            motor.tau_est = 0.0
            motor.temperature = [30, 30]
            motor.vol = 48.0
        for slot, angle in self.hold_q.items():
            motor = state.motor_state[int(slot)]
            motor.mode = 1
            motor.q = float(angle)
            motor.dq = 0.0
            motor.tau_est = 0.0
            motor.temperature = [30, 30]
            motor.vol = 48.0
        if fault == "tick":
            tick = (tick - 5000) % 2**32          # 倒退 → 解码判定 tick 倒退
        elif fault == "nan":
            state.motor_state[int(self.slots[0])].q = float("nan")
        elif fault == "error":
            state.motor_state[int(self.slots[0])].motorstate = 1
        elif fault == "temp":
            for slot in self.slots:
                state.motor_state[int(slot)].temperature = [80, 80]
        elif fault == "mode":
            state.mode_pr = 1
        state.tick = tick
        state.crc = self.crc.Crc(state)
        if fault == "crc":
            state.crc = (int(state.crc) + 1) % 2**32   # CRC 本来就不对
        return state

    def announce(self, fault, now):
        if fault != "none" and not self.fault_announced:
            self.fault_announced = True
            print(f"FAULT_INJECTED t={time.monotonic():.6f} kind={fault}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--fault", choices=FAULTS, default="none")
    parser.add_argument("--at", type=float, default=5.0, help="故障生效时刻（启动后秒数）")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "fake_robot.json")
    args = parser.parse_args()

    robot = FakeRobot(args.fault, args.at)
    initialize(args.network, args.domain)
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

    publisher = ChannelPublisher("rt/lowstate", LowState_)
    publisher.Init()
    subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
    subscriber.Init(robot.accept, 1)

    stop = {"value": False}

    def handler(*_):
        stop["value"] = True

    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, handler)

    print(f"FAKE_READY network={args.network} domain={args.domain} "
          f"fault={args.fault} at={args.at:.1f}s joints={len(robot.slots)}", flush=True)
    due = time.monotonic()
    published = 0
    try:
        while not stop["value"]:
            now = time.monotonic()
            if now - robot.started >= args.duration:
                break
            fault = robot.active_fault(now)
            robot.announce(fault, now)
            if fault != "timeout":     # timeout：干脆不发，模拟链路中断/丢包
                if not publisher.Write(robot.state(now)):
                    raise SystemExit("LowState 发送失败")
                published += 1
            due += 0.002
            delay = due - time.monotonic()
            if delay > 0:
                time.sleep(min(delay, 0.002))
            else:
                due = time.monotonic()
    finally:
        subscriber.Close()
        publisher.Close()
        for sig, fn in previous.items():
            signal.signal(sig, fn)
        report = {"fault": args.fault, "fault_at_s": args.at,
                  "commands_accepted": robot.accepted, "commands_rejected": robot.rejected,
                  "last_reject": robot.last_reject,
                  "states_published": published,
                  "wall_seconds": round(time.monotonic() - robot.started, 2)}
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n",
                               encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
