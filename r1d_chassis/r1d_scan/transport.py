"""HG LowCmd / LowState 编解码：槽位与增益来自 robot_profile.json，CRC 用官方 SDK。"""
from dataclasses import dataclass

import numpy as np

from profile import load as load_profile


class ControlFault(RuntimeError):
    pass


@dataclass(frozen=True)
class Snapshot:
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    errors: np.ndarray
    temperature: np.ndarray
    present: tuple
    mode_machine: int
    mode_pr: int
    tick: int
    received: float
    progressed: float
    hold_q: np.ndarray = None      # 保持关节（右臂）的实测角，用于"照原样写回"


class Codec:
    def __init__(self, profile=None, profile_path=None):
        try:
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            from unitree_sdk2py.utils.crc import CRC
        except ImportError as exc:  # pragma: no cover
            raise ControlFault("缺少 unitree_sdk2py") from exc
        self.profile = profile
        if self.profile is None:
            self.profile = load_profile(profile_path) if profile_path else load_profile()
        self.joints = [j.name for j in self.profile.joints]
        self.slots = np.array([j.slot for j in self.profile.joints], int)
        self.kp = np.array([j.kp for j in self.profile.joints], float)
        self.kd = np.array([j.kd for j in self.profile.joints], float)
        # 保持关节：本程序不规划它们，但每拍要写固定值让它们不动（Q9）
        self.hold_names = [j.name for j in getattr(self.profile, "hold_joints", ())]
        self.hold_slots = np.array([j.slot for j in getattr(self.profile, "hold_joints", ())], int)
        self.hold_kp = np.array([j.kp for j in getattr(self.profile, "hold_joints", ())], float)
        self.hold_kd = np.array([j.kd for j in getattr(self.profile, "hold_joints", ())], float)
        self.hold_targets = None      # 启动后由实测姿态填入，见 scan.py
        self.max_slot = int(max(list(self.slots) + list(self.hold_slots)))
        self.default_command = unitree_hg_msg_dds__LowCmd_
        self.crc = CRC()

    def decode(self, msg, now, previous=None):
        if len(msg.motor_state) <= self.max_slot:
            raise ControlFault(f"LowState 长度 {len(msg.motor_state)} 不足，"
                               f"至少需要 {self.max_slot + 1} 个槽位")
        if int(msg.crc) != self.crc.Crc(msg):
            raise ControlFault("LowState CRC 错误")
        states = [msg.motor_state[int(s)] for s in self.slots]
        q = np.array([s.q for s in states], float)
        dq = np.array([s.dq for s in states], float)
        tau = np.array([s.tau_est for s in states], float)
        hold_q = (np.array([msg.motor_state[int(s)].q for s in self.hold_slots], float)
                  if self.hold_slots.size else np.zeros(0))
        if not np.isfinite(np.concatenate((q, dq, tau, hold_q))).all():
            raise ControlFault("LowState 含非有限数值")
        present = tuple(int(s) for s in self.slots
                        if int(msg.motor_state[int(s)].mode) != 0
                        or abs(msg.motor_state[int(s)].q) > 1e-6
                        or abs(msg.motor_state[int(s)].dq) > 1e-6
                        or abs(msg.motor_state[int(s)].tau_est) > 1e-6)
        tick = int(msg.tick)
        if previous is not None:
            delta = (tick - previous.tick) % 2**32
            if delta >= 2**31:
                raise ControlFault("LowState tick 倒退")
            progressed = now if delta else previous.progressed
        else:
            progressed = now
        return Snapshot(q=q, dq=dq, tau=tau,
                        errors=np.array([int(s.motorstate) for s in states], np.uint32),
                        temperature=np.array([max(s.temperature) for s in states], float),
                        present=present, mode_machine=int(msg.mode_machine),
                        mode_pr=int(msg.mode_pr), tick=tick,
                        received=now, progressed=progressed, hold_q=hold_q)

    def encode(self, q, state, feedforward=None):
        q = np.asarray(q, float)
        if q.shape != self.slots.shape or not np.isfinite(q).all():
            raise ControlFault(f"指令必须是 {self.slots.size} 个有限关节角")
        hold = None
        if self.hold_slots.size:
            if self.hold_targets is None:
                # 配置了保持关节却不给目标，等于让右臂听天由命，宁可拒发
                raise ControlFault("profile 配置了保持关节，但没有设置保持目标")
            hold = np.asarray(self.hold_targets, float)
            if hold.shape != self.hold_slots.shape or not np.isfinite(hold).all():
                raise ControlFault(f"保持目标必须是 {self.hold_slots.size} 个有限关节角")
        command = self.default_command()
        command.mode_pr = 0
        command.mode_machine = state.mode_machine
        for i, slot in enumerate(self.slots):
            motor = command.motor_cmd[int(slot)]
            motor.mode = 1
            motor.q = float(q[i])
            motor.dq = 0.0
            motor.tau = 0.0 if feedforward is None else float(feedforward[i])
            motor.kp = float(self.kp[i])
            motor.kd = float(self.kd[i])
        for i, slot in enumerate(self.hold_slots):
            # 写固定值：位置=动作开始前的实测值，速度/前馈 0，只靠 KP/KD 顶住
            motor = command.motor_cmd[int(slot)]
            motor.mode = 1
            motor.q = float(hold[i])
            motor.dq = 0.0
            motor.tau = 0.0
            motor.kp = float(self.hold_kp[i])
            motor.kd = float(self.hold_kd[i])
        command.crc = self.crc.Crc(command)
        return command
