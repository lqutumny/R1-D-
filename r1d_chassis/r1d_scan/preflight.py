"""只规划、不发指令的真机预检（`scan.py --dry-run` 的实现）。

为什么要有它：在真机上，"第一次发指令"和"第一次验证规划"不能是同一件事。
真机的起始姿态每次都不同（实测过肩 +6°、肘 +84°，也见过肩 +14°、肘 +62°），
程序从**实测姿态**规划到扫描起点并检查组合几何，
再按完整时间轴复核限位、距离和关节导数。

这里把整轮时间轴离线走一遍（保持 → 接近 → 扫描 → 回位），只做检查：

1. 目标角是否始终留在关节限位内（含 margin）；
2. 目标角速度是否超过关节限速；
3. 扫描段 TCP 的横向偏差、探头轴线角；
4. 全程 TCP 离箱面的最小距离（接近段最容易贴箱面）。

整个过程**不创建任何 publisher**，报告里 `published_commands` 恒为 0。
"""
from dataclasses import dataclass, field

import numpy as np


@dataclass
class PreflightResult:
    ok: bool
    seconds: float
    published_commands: int = 0
    min_limit_margin_deg: float = float("inf")
    binding_joint: int = -1
    max_joint_speed_deg_s: float = 0.0
    joint_derivative_peaks: dict = field(default_factory=dict)
    max_transverse_mm: float = 0.0
    max_axis_deg: float = 0.0
    min_standoff_mm: float = float("inf")
    scan_standoff_range_mm: tuple = (0.0, 0.0)
    z_range_m: tuple = (0.0, 0.0)
    approach_delta_deg: float = 0.0
    phases: list = field(default_factory=list)
    violations: list = field(default_factory=list)

    def report(self):
        return {
            "preflight": True,
            "published_commands": self.published_commands,
            "ok": self.ok,
            "seconds": round(self.seconds, 2),
            "min_limit_margin_deg": round(self.min_limit_margin_deg, 2),
            "binding_joint": self.binding_joint,
            "max_joint_speed_deg_s": round(self.max_joint_speed_deg_s, 1),
            "joint_derivative_peaks": self.joint_derivative_peaks,
            "max_transverse_mm": round(self.max_transverse_mm, 3),
            "max_axis_deg": round(self.max_axis_deg, 3),
            "min_standoff_mm": round(self.min_standoff_mm, 1),
            "scan_standoff_range_mm": [round(v, 2) for v in self.scan_standoff_range_mm],
            "z_range_m": [round(v, 3) for v in self.z_range_m],
            "approach_delta_deg": round(self.approach_delta_deg, 1),
            "phases": self.phases,
            "violations": self.violations,
        }


def run(controller, motion, geom, limits, kin, sample_dt=0.002, fk_dt=0.010,
        clearance=None):
    """把整轮时间轴离线走一遍。

    controller: 已用实测姿态构造好的 Controller（它的 start/approach_seconds 决定时间轴）
    motion:     load_motion() 返回的轨迹对象
    geom:       dict，来自 models/scan_profile.json：tcp_x_m / y_m / z_bottom_m / z_top_m / standoff_m
    limits:     Controller 的 Limits
    kin:        kinematics.Kinematics，提供 forward(q) -> (position, axis, rotation)
    """
    t_end = (controller.start + controller.approach_seconds
             + controller.ready_seconds + motion.duration + controller.return_seconds)
    times = np.arange(0.0, t_end + sample_dt, sample_dt)

    check_derivatives = getattr(motion, "joint_jerk_limit", None) is not None
    phases, positions = controller.preflight_targets(times)
    scan_zs, transverse, axis_deg, standoffs = [], [], [], []
    wall_axis = int(geom.get("wall_axis", 0))
    axis_target = np.eye(3)[wall_axis]
    line = np.array([geom["tcp_x_m"], geom["y_m"]])
    wall_position = line[wall_axis] + float(geom["standoff_m"])

    result = PreflightResult(ok=True, seconds=float(t_end))
    if not np.isfinite(positions).all():
        result.ok = False
        result.violations.append("预检轨迹含非有限关节角")
        return result

    margins = np.minimum(positions - controller.lo, controller.hi - positions)
    joints = np.argmin(margins, axis=1)
    smallest = margins[np.arange(len(times)), joints]
    i = int(np.argmin(smallest))
    result.binding_joint = int(joints[i])
    result.min_limit_margin_deg = float(np.degrees(smallest[i]))
    for i in np.flatnonzero(smallest < limits.margin):
        result.violations.append(
            f"t={times[i]:.2f}s {phases[i]}: 关节{joints[i]} {controller.joint_names[joints[i]]} 余量 "
            f"{np.degrees(smallest[i]):.2f}° < {np.degrees(limits.margin):.1f}°")

    velocities = np.max(np.abs(np.diff(positions, axis=0)), axis=1) / sample_dt
    if len(velocities):
        result.max_joint_speed_deg_s = float(np.degrees(velocities.max()))
    for i in np.flatnonzero(velocities > limits.joint_speed * 1.10):
        result.violations.append(
            f"t={times[i + 1]:.2f}s {phases[i + 1]}: 目标角速度 {np.degrees(velocities[i]):.1f}°/s 超限")

    fk_next = 0.0
    for t, phase, q in zip(times, phases, positions):
        if t < fk_next:
            continue
        fk_next += fk_dt
        issue = clearance.issue(q) if clearance is not None else None
        if issue:
            result.violations.append(f"t={t:.2f}s {phase}: {issue}")
        if clearance is not None and clearance.kin is kin and issue is None:
            position, axis, _ = kin.current_pose()
        else:
            position, axis, _ = kin.forward(q)
        standoff = wall_position - float(position[wall_axis])
        result.min_standoff_mm = min(result.min_standoff_mm, standoff * 1000.0)
        if standoff < 0.10:
            result.violations.append(
                f"t={t:.2f}s {phase}: TCP 离箱面 {standoff*1000:.0f} mm < 100 mm")
        if phase in ("scan-up", "scan-down"):
            transverse.append(float(np.hypot(position[0] - line[0], position[1] - line[1])))
            axis_deg.append(float(np.degrees(np.arccos(np.clip(float(axis @ axis_target), -1.0, 1.0)))))
            scan_zs.append(float(position[2]))
            standoffs.append(standoff * 1000)

    if transverse:
        result.max_transverse_mm = max(transverse) * 1000.0
        result.max_axis_deg = max(axis_deg)
        result.z_range_m = (min(scan_zs), max(scan_zs))
        result.scan_standoff_range_mm = (min(standoffs), max(standoffs))
    if check_derivatives:
        bounds = (
            ("speed_rad_s", motion.joint_speed_limit),
            ("acceleration_rad_s2", motion.joint_acceleration_limit),
            ("jerk_rad_s3", motion.joint_jerk_limit),
        )
        for order, (name, limit) in enumerate(bounds, start=1):
            values = np.diff(positions, n=order, axis=0) / sample_dt ** order
            peak = float(np.max(np.abs(values)))
            result.joint_derivative_peaks[name] = round(peak, 5)
            if peak > limit * 1.005:  # 差分浮点误差留少量余量
                result.violations.append(f"整轮 {name} 峰值 {peak:.4f} 超过限值 {limit:.4f}")
        if result.max_transverse_mm > 20.0:
            result.violations.append(f"扫描段横向偏差 {result.max_transverse_mm:.1f} mm > 20 mm")
        if result.max_axis_deg > 10.0:
            result.violations.append(f"扫描段轴线偏差 {result.max_axis_deg:.1f}° > 10°")
    transitions = np.r_[0, np.flatnonzero(phases[1:] != phases[:-1]) + 1]
    result.phases = [{"phase": phases[i], "t": round(float(times[i]), 2)} for i in transitions]
    result.approach_delta_deg = float(np.degrees(
        np.max(np.abs(controller.q_scan_start - controller.captured))))
    result.ok = not result.violations
    return result