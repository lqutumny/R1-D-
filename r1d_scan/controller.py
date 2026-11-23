"""状态机 + 保护：启动保持 → 接近扫描起点 → 往返扫描 → 保持。

不依赖 DDS，可直接用离线数据驱动（便于测试）。
"""
from dataclasses import dataclass
from pathlib import Path
import json

import numpy as np

from transport import ControlFault

PROFILE = Path(__file__).resolve().parents[1] / "models" / "scan_profile.json"


@dataclass
class Limits:
    joint_speed: float = 0.40          # rad/s
    joint_acceleration: float = 1.0    # rad/s²
    margin: float = 0.02               # rad，距限位余量
    tracking_error: float = 0.20       # rad
    tracking_seconds: float = 0.30
    torque_fraction: float = 0.90
    state_timeout: float = 0.20
    # 控制周期：单次停顿和"反复停顿"分开判。
    # 虚拟机/普通 PC 上偶发一次 50–80 ms 的调度停顿是无害的（手臂保持最后指令，路径少走 ~1.5 mm），
    # 但"10 s 内反复停顿"说明环境根本不适合跑控制，那才必须停。
    loop_stall_stop: float = 0.15     # 单次超过这个 → 立即停
    loop_stall_warn: float = 0.05     # 超过这个记一次"打嗝"
    loop_hiccups: int = 5             # 窗口内打嗝次数达到这个 → 停
    loop_window: float = 10.0
    startup_hold: float = 2.0
    approach_seconds: float = 4.0
    approach_speed_factor: float = 0.5    # 接近段用一半的关节限速，减小 PD 滞后
    approach_accel_factor: float = 0.5


def transition(start, end, seconds, samples=200):
    """五次平滑过渡：两端位置/速度为零。"""
    u = np.linspace(0.0, 1.0, samples)
    s = 10 * u**3 - 15 * u**4 + 6 * u**5
    return u * seconds, np.asarray(start)[None, :] + s[:, None] * (np.asarray(end)[None, :] - np.asarray(start)[None, :])


def load_profile_names():
    import json as _json
    raw = _json.loads((Path(__file__).resolve().parent / "robot_profile.json").read_text(encoding="utf-8"))
    return [joint["name"] for joint in raw["joints"]]


class Controller:
    def __init__(self, captured, motion, lo, hi, kp, kd, effort,
                 limits=None, start=0.0, scene=None, expected_mode_machine=None):
        import mujoco
        self.mujoco = mujoco
        self.scene = Path(scene) if scene else Path(__file__).resolve().parents[1] / "models" / "r1_a7_scene.xml"
        self.model = mujoco.MjModel.from_xml_path(str(self.scene))
        self.data = mujoco.MjData(self.model)
        self.joint_names = [j for j in load_profile_names()]
        self.qadr = np.array([self.model.jnt_qposadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in self.joint_names])
        self.dadr = np.array([self.model.jnt_dofadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in self.joint_names])
        self.last_ff = np.zeros(len(self.joint_names))
        self.limits = limits or Limits()
        # mode_machine 标识机型/固件族；运行中变化说明有人在切模式，必须停
        self.expected_mode_machine = expected_mode_machine
        self.captured = np.asarray(captured, float).copy()
        self.q_scan_start = np.asarray(motion.sample(0.0), float)
        self.motion = motion
        self.lo, self.hi = np.asarray(lo), np.asarray(hi)
        self.kp, self.kd = np.asarray(kp), np.asarray(kd)
        self.effort = np.asarray(effort)
        self.start = start + self.limits.startup_hold
        delta = float(np.max(np.abs(self.q_scan_start - self.captured)))
        # 五次过渡的峰值速度是 1.875·Δ/T，峰值加速度 5.78·Δ/T²；
        # 按限速/限加速度反算最小过渡时长，否则接近段会超速。
        approach_speed = self.limits.joint_speed * self.limits.approach_speed_factor
        approach_accel = self.limits.joint_acceleration * self.limits.approach_accel_factor
        self.approach_seconds = max(
            self.limits.approach_seconds,
            1.875 * delta / approach_speed,
            float(np.sqrt(5.78 * delta / approach_accel))) if delta > 1e-9 else 0.0
        self.t_approach = None
        self.q_approach = None
        self.previous = start - 0.002
        self.last_q = self.captured.copy()
        self.last_velocity = np.zeros_like(self.captured)
        self.bad_since = None
        self.hiccups = []               # 控制循环的"打嗝"时刻
        self.phase = "startup-hold"
        self.complete = False
        self.finished_at = None

    # ---------- 轨迹 ----------
    def target(self, now):
        elapsed = now - self.start
        if elapsed < 0:
            return "startup-hold", self.captured
        if elapsed < self.approach_seconds:
            # 解析五次曲线：查表会在采样点之间造成速度尖峰
            u = elapsed / self.approach_seconds if self.approach_seconds > 0 else 1.0
            s = 10 * u**3 - 15 * u**4 + 6 * u**5
            return "approach", self.captured + s * (self.q_scan_start - self.captured)
        scan_elapsed = elapsed - self.approach_seconds
        if scan_elapsed < self.motion.duration:
            direction = "scan-up" if scan_elapsed < self.motion.one_way_duration else "scan-down"
            return direction, self.motion.sample(scan_elapsed)
        # 往返结束后回到捕获姿态，让整轮动作回到起点
        back_elapsed = scan_elapsed - self.motion.duration
        scan_end = self.motion.sample(self.motion.duration)
        if back_elapsed < self.approach_seconds:
            u = back_elapsed / self.approach_seconds if self.approach_seconds > 0 else 1.0
            s = 10 * u**3 - 15 * u**4 + 6 * u**5
            return "return", scan_end + s * (self.captured - scan_end)
        self.complete = True
        return "finished-hold", self.captured

    # ---------- 重力前馈 ----------
    def gravity(self, q):
        """模型静态重力前馈（零速度）。kp=100 的 PD 撑不住 17 kg 手臂，必须补这一项。"""
        self.data.qpos[self.qadr] = np.asarray(q, float)
        mujoco = self.mujoco
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        bias = np.zeros(self.model.nv)
        mujoco.mj_rne(self.model, self.data, 0, bias)
        return bias[self.dadr]

    # ---------- 保护 ----------
    def check(self, q_target, state, now, dt, feedforward=None):
        if state is None:
            raise ControlFault("没有 LowState")
        if now - state.received > self.limits.state_timeout:
            raise ControlFault(f"LowState 超时 {now - state.received:.2f}s")
        if state.mode_pr != 0:
            raise ControlFault("mode_pr 不是 0")
        if self.expected_mode_machine is not None and state.mode_machine != self.expected_mode_machine:
            raise ControlFault(f"mode_machine 变化: {state.mode_machine}"
                               f"（期望 {self.expected_mode_machine}）")
        if np.any(state.errors):
            raise ControlFault(f"电机故障字: {state.errors.tolist()}")
        if np.max(state.temperature) >= 70:
            raise ControlFault("电机温度超过 70°C")
        if np.any(np.abs(state.tau) > self.effort * 1.1):
            raise ControlFault("实测力矩超过 URDF 预算 110%")
        if np.any(q_target < self.lo + self.limits.margin) or np.any(q_target > self.hi - self.limits.margin):
            j = int(np.argmax(np.maximum(self.lo + self.limits.margin - q_target,
                                         q_target - (self.hi - self.limits.margin))))
            raise ControlFault(f"目标接近限位: 关节 {j} q={q_target[j]:+.3f} "
                               f"range=[{self.lo[j]:+.3f}, {self.hi[j]:+.3f}]")
        velocity = (q_target - self.last_q) / max(dt, 1e-9)
        if np.max(np.abs(velocity)) > self.limits.joint_speed * 1.10:
            raise ControlFault(f"目标角速度超限: {np.max(np.abs(velocity)):.3f} rad/s")
        torque = self.kp * (q_target - state.q) - self.kd * state.dq
        total = torque if feedforward is None else torque + feedforward
        ratio = np.abs(total) / self.effort
        if np.max(ratio) > self.limits.torque_fraction:
            j = int(np.argmax(ratio))
            print(f"[诊断] 关节{j}: q_target={q_target[j]:+.3f} q={state.q[j]:+.3f} "
                  f"dq={state.dq[j]:+.3f} kp={self.kp[j]:.0f} kd={self.kd[j]:.0f} "
                  f"pd={torque[j]:+.1f} ff={0 if feedforward is None else feedforward[j]:+.1f} "
                  f"-> total={total[j]:+.1f}", flush=True)
            raise ControlFault(f"预计力矩超预算: 关节 {j} {total[j]:+.1f} N·m "
                               f"(限额 {self.effort[j]:.0f})")
        error = np.abs(q_target - state.q)
        if np.max(error) > self.limits.tracking_error:
            if self.bad_since is None:
                self.bad_since = now
            elif now - self.bad_since > self.limits.tracking_seconds:
                j = int(np.argmax(error))
                raise ControlFault(f"跟踪误差持续超限: 关节{j} {self.joint_names[j]} "
                                   f"err={error[j]:.3f} rad (目标 {q_target[j]:+.3f}, "
                                   f"实际 {state.q[j]:+.3f}, 状态年龄 {now - state.received:.3f}s, "
                                   f"阶段 {self.phase})")
        else:
            self.bad_since = None
        return torque

    def tick(self, state, now):
        dt = now - self.previous
        # 两级判据：单次大停顿 vs 窗口内反复打嗝（见 Limits 里的说明）
        if dt <= 0:
            raise ControlFault(f"控制周期异常 {dt*1000:.1f} ms")
        if dt > self.limits.loop_stall_stop:
            raise ControlFault(f"控制周期异常 {dt*1000:.1f} ms（单次停顿超过 "
                               f"{self.limits.loop_stall_stop*1000:.0f} ms）")
        if dt > self.limits.loop_stall_warn:
            self.hiccups.append(now)
            self.hiccups = [t for t in self.hiccups if now - t < self.limits.loop_window]
            if len(self.hiccups) >= self.limits.loop_hiccups:
                raise ControlFault(
                    f"控制循环反复停顿：{self.limits.loop_window:.0f} s 内 "
                    f"{len(self.hiccups)} 次超过 {self.limits.loop_stall_warn*1000:.0f} ms")
        self.phase, q_target = self.target(now)
        feedforward = self.gravity(q_target)
        self.last_ff = feedforward
        torque = self.check(q_target, state, now, dt, feedforward)
        self.last_q = q_target.copy()
        self.previous = now
        return self.phase, q_target, float(np.max(np.abs(torque + feedforward)))


def load_motion(path=PROFILE):
    from path_timing import LocalPathMotion, SmoothedTiming
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    q = np.array(data["q_rad"], float)
    distance = np.linspace(0.0, data["length_m"], len(q))
    motion = LocalPathMotion(distance, q, data["requested_peak_speed_m_s"],
                             data["joint_speed_limit_rad_s"],
                             data["joint_acceleration_limit_rad_s2"], cycles=1)
    # 时间最优解是 bang-bang 的，加速度硬切换 → 真机上顿挫。
    # 这里只重排时间轴（路径不变），默认 0.15 s 滑动平均；profile 里可覆盖或设 0 关闭。
    window = float(data.get("time_smooth_s", 0.15))
    if window > 0:
        motion = SmoothedTiming(motion, window,
                                 accel_limit=data["joint_acceleration_limit_rad_s2"])
    return data, motion
