"""只规划、不发指令的真机预检（`scan.py --dry-run` 的实现）。

为什么要有它：在真机上，"第一次发指令"和"第一次验证规划"不能是同一件事。
真机的起始姿态每次都不同（实测过肩 +6°、肘 +84°，也见过肩 +14°、肘 +62°），
而接近段是从**实测姿态**线性插值到扫描起点——这条插值路径在仿真里没跑过，
限位余量、离箱面距离都可能与规划值不同。

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
    max_transverse_mm: float = 0.0
    max_axis_deg: float = 0.0
    min_standoff_mm: float = float("inf")
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
            "max_transverse_mm": round(self.max_transverse_mm, 3),
            "max_axis_deg": round(self.max_axis_deg, 3),
            "min_standoff_mm": round(self.min_standoff_mm, 1),
            "z_range_m": [round(v, 3) for v in self.z_range_m],
            "approach_delta_deg": round(self.approach_delta_deg, 1),
            "phases": self.phases,
            "violations": self.violations,
        }


def run(controller, motion, geom, limits, kin, sample_dt=0.002, fk_dt=0.010):
    """把整轮时间轴离线走一遍。

    controller: 已用实测姿态构造好的 Controller（它的 start/approach_seconds 决定时间轴）
    motion:     load_motion() 返回的轨迹对象
    geom:       dict，来自 models/scan_profile.json：tcp_x_m / y_m / z_bottom_m / z_top_m / standoff_m
    limits:     Controller 的 Limits
    kin:        kinematics.Kinematics，提供 forward(q) -> (position, axis, rotation)
    """
    t_end = (controller.start + controller.approach_seconds
             + motion.duration + controller.approach_seconds)
    times = np.arange(0.0, t_end + sample_dt, sample_dt)

    q_prev = None
    fk_next = 0.0
    scan_zs, transverse, axis_deg, standoffs = [], [], [], []
    axis_target = np.array([1.0, 0.0, 0.0])
    wall_x = float(geom["tcp_x_m"]) + float(geom["standoff_m"])
    ideal_z = (float(geom["z_bottom_m"]), float(geom["z_top_m"]))

    result = PreflightResult(ok=True, seconds=float(t_end))
    min_margin_rad = float("inf")
    seen = []
    for t in times:
        phase, q = controller.target(t)
        if not seen or seen[-1][0] != phase:
            seen.append((phase, float(t)))
        q = np.asarray(q, float)

        # 1) 限位余量
        margin = np.minimum(q - controller.lo, controller.hi - q)
        j = int(np.argmin(margin))
        if margin[j] < min_margin_rad:
            min_margin_rad = float(margin[j])
            result.min_limit_margin_deg = float(np.degrees(margin[j]))
            result.binding_joint = j
        if margin[j] < limits.margin:
            result.violations.append(
                f"t={t:.2f}s {phase}: 关节{j} {controller.joint_names[j]} 余量 "
                f"{np.degrees(margin[j]):.2f}° < {np.degrees(limits.margin):.1f}°")

        # 2) 目标角速度（用参考轨迹自身的差分，和 Controller.check 同一条判据）
        if q_prev is not None:
            speed = float(np.max(np.abs(q - q_prev)) / sample_dt)
            result.max_joint_speed_deg_s = max(result.max_joint_speed_deg_s,
                                               float(np.degrees(speed)))
            if speed > limits.joint_speed * 1.10:
                result.violations.append(
                    f"t={t:.2f}s {phase}: 目标角速度 {np.degrees(speed):.1f}°/s 超限")

        # 3) 几何：FK 较贵，按 fk_dt 抽
        if t >= fk_next:
            fk_next += fk_dt
            position, axis, _ = kin.forward(q)
            standoff = wall_x - float(position[0])
            result.min_standoff_mm = min(result.min_standoff_mm, standoff * 1000.0)
            if standoff < 0.10:
                result.violations.append(
                    f"t={t:.2f}s {phase}: TCP 离箱面 {standoff*1000:.0f} mm < 100 mm")
            if phase.startswith("scan"):
                transverse.append(float(np.hypot(position[0] - geom["tcp_x_m"],
                                                 position[1] - geom["y_m"])))
                axis_deg.append(float(np.degrees(np.arccos(
                    np.clip(float(axis @ axis_target), -1.0, 1.0)))))
                scan_zs.append(float(position[2]))

        q_prev = q

    if transverse:
        result.max_transverse_mm = max(transverse) * 1000.0
        result.max_axis_deg = max(axis_deg)
        result.z_range_m = (min(scan_zs), max(scan_zs))
    result.phases = [{"phase": p, "t": round(t, 2)} for p, t in seen]
    result.approach_delta_deg = float(np.degrees(
        np.max(np.abs(controller.q_scan_start - controller.captured))))
    result.ok = not result.violations
    return result