#!/usr/bin/env python3
"""生成竖直扫描轨迹：101 点 IK + 时间规划 + 10 ms 整轨复核。

用法：
    python3 tools/scan_plan.py --standoff 0.25 --z-bottom 0.85 --z-top 1.35 --speed 0.04
"""
import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.interpolate import CubicSpline

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))
from kinematics import ArmKinematics, JOINTS, default_seed, SCENE  # noqa: E402
from path_timing import LocalPathMotion  # noqa: E402
from profile import load as load_profile  # noqa: E402

PROFILE = ROOT / "models" / "scan_profile.json"
REPORT = ROOT / "out" / "scan_plan.json"


def check_trajectory(kin, motion, points, axis_target, wall_x, standoff, density=0.01):
    """按 10 ms 重采样检查整条轨迹（不是只看 101 个节点）。"""
    duration = motion.duration
    times = np.arange(0.0, duration + 1e-9, density)
    deviations, angles, gaps, speeds, margins = [], [], [], [], []
    worst_contact = 0.0
    data = kin.data
    for t in times:
        q = motion.sample(t)
        position, axis, _ = kin.forward(q)
        deviations.append(np.linalg.norm(position[:2] - np.array(points[0][:2])))
        angles.append(np.degrees(np.arccos(np.clip(axis @ axis_target, -1, 1))))
        gaps.append(wall_x - position[0])
        margin_rad, _ = kin.margin(q)
        margins.append(margin_rad)
        velocity = kin.position_jacobian(q) @ motion.sample(t, 1)
        speeds.append(float(np.linalg.norm(velocity)))
        mujoco.mj_forward(kin.model, data)
        for contact in data.contact:
            if contact.dist < worst_contact:
                worst_contact = float(contact.dist)
    return {"samples": len(times),
            "max_transverse_mm": float(max(deviations) * 1000),
            "max_axis_deg": float(max(angles)),
            "min_wall_gap_m": float(min(gaps)),
            "max_tip_speed_m_s": float(max(speeds)),
            "min_limit_margin_deg": float(np.degrees(min(margins))),
            "worst_penetration_mm": float(-worst_contact * 1000)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument("--standoff", type=float, default=0.25, help="TCP 端面到箱面距离 m")
    parser.add_argument("--y", type=float, default=0.14, help="扫描线横向位置 m（左臂自然平面）")
    parser.add_argument("--z-bottom", type=float, default=0.85)
    parser.add_argument("--z-top", type=float, default=1.35)
    parser.add_argument("--points", type=int, default=101)
    parser.add_argument("--speed", type=float, default=0.04, help="路径速度上限 m/s")
    parser.add_argument("--joint-speed", type=float, default=0.25)
    parser.add_argument("--joint-acceleration", type=float, default=0.5)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--lock", nargs="*", default=[],
                        help="锁定的关节（不参与 IK）。默认不锁：锁定会改变 IK 分支、在路径上造成折角")
    parser.add_argument("--no-refine", action="store_true",
                        help="关掉零空间微调，只做连续求解（路径更平滑）")
    parser.add_argument("--waist-stiffness", type=float, default=1.0,
                        help="腰 yaw 的刚度权重，越大越不愿意转（>2 会破坏连续性，待优化）")
    parser.add_argument("--wrist-stiffness", type=float, default=1.0,
                        help="腕 roll 的刚度权重（对圆柱探头无影响）")
    parser.add_argument("--stiff", nargs="*", default=[],
                        help="额外关节刚度，形如 left_wrist_pitch_joint=6（越大越不愿意动）。"
                             "真机上小力矩关节（腕）吃不住大扫角时用它，让肩肘去承担")
    parser.add_argument("--chain", action="store_true",
                        help="用「沿 z 串行求解」代替微分推进：每点以上一点为初值，"
                             "能走到余量更好的分支（真机腕限位收窄后必须这样）")
    parser.add_argument("--pre-roll", type=float, default=0.15,
                        help="串行求解时先在扫描线下方预热的距离 m（决定进入哪个分支）")
    args = parser.parse_args()
    weights = np.ones(len(JOINTS))
    if "waist_yaw_joint" in JOINTS:
        weights[JOINTS.index("waist_yaw_joint")] = args.waist_stiffness
    if "left_wrist_roll_joint" in JOINTS:
        weights[JOINTS.index("left_wrist_roll_joint")] = args.wrist_stiffness
    for item in args.stiff:
        name, _, value = item.partition("=")
        if name not in JOINTS:
            raise SystemExit(f"--stiff 里的 {name} 不是受控关节；可选：{JOINTS}")
        weights[JOINTS.index(name)] = float(value)
    active = np.array([name not in args.lock for name in JOINTS])
    if active.sum() < 5:
        raise SystemExit(f"锁定 {args.lock} 后只剩 {int(active.sum())} 个自由度，少于 5 个约束")

    kin = ArmKinematics(args.scene)
    # profile 是限位的唯一真源：真机实测过的限位（例如左腕 pitch 只有 ±43°，
    # 而 URDF 写 ±92.5°）在这里覆盖模型，规划器就会自己避开，而不是撞上去再被保护拦住。
    for joint in load_profile().joints:
        if joint.name in JOINTS:
            i = JOINTS.index(joint.name)
            kin.lo[i] = joint.lower
            kin.hi[i] = joint.upper
    wall_x = float(kin.model.geom_pos[mujoco.mj_name2id(kin.model, mujoco.mjtObj.mjOBJ_GEOM, "wall")][0])
    axis_target = np.array([1.0, 0.0, 0.0])
    tcp_x = wall_x - args.standoff
    zs = np.linspace(args.z_bottom, args.z_top, args.points)
    points = [[tcp_x, args.y, float(z)] for z in zs]

    qs, seed = [], default_seed()
    # 多初值求解。注意：不能"第一个收敛的就收"——那样会选中贴着限位的分支，
    # 后面整条路径都跟着贴死（真机上就是撞机械限位）。要在所有收敛解里挑**余量最大**的。
    candidates = [seed, seed + 0.3, seed - 0.3, np.zeros(len(JOINTS)),
                  seed + np.array([0, 0, 0, 0, 0, -0.6, 0]),
                  seed + np.array([-0.3, 0, 0, 0.3, 0, -0.3, 0])]
    sol, sol_margin = None, -1e9
    for candidate in candidates:
        trial = kin.solve(points[0], axis_target, candidate, toward=candidate, active=active,
                          refine=True)
        if trial.position_error > 1e-3:
            continue
        margin = kin.margin(trial.q)[0]
        if sol is None or margin > sol_margin:
            sol, sol_margin = trial, margin
    if sol is None:
        for candidate in candidates:
            trial = kin.solve(points[0], axis_target, candidate, toward=candidate, active=active,
                              refine=False)
            if sol is None or trial.position_error < sol.position_error:
                sol = trial
    print(f"首点：选了余量 {sol_margin:.1f}° 的分支" if sol_margin > -1e8 else "首点：没有收敛解")
    if sol.position_error > 1e-3:
        raise SystemExit(f"首点 IK 失败：位置 {sol.position_error*1000:.2f} mm")
    if args.chain:
        # 串行求解：先在扫描线下方预热一段，再逐点以上一点为初值。
        # 这样能走到"腕 pitch 只在 −33°~+32° 之间摆"的好分支，
        # 而微分推进从通用初值出发会落到贴着限位的坏分支上。
        q = default_seed()
        if args.pre_roll > 0:
            for z in np.linspace(args.z_bottom - args.pre_roll, args.z_bottom, 9)[:-1]:
                q = kin.solve([tcp_x, args.y, float(z)], axis_target, q,
                              active=active, refine=True).q
        qs = []
        for p in points:
            q = kin.solve(p, axis_target, q, active=active, refine=True).q
            qs.append(q)
        qs = np.array(qs)
        print(f"串行求解：预热 {args.pre_roll*100:.0f} cm，共 {len(qs)} 点")
        patched, rejected = [], []
    else:
        # iters 从默认 6 提到 15：换探头后首段步长变大，6 次迭代收敛到 1.02 mm 就停了
        qs = kin.follow_line(points, axis_target, sol.q, active=active, joint_weights=weights,
                             iters=15)
        # 局部修补：腕 pitch 限位收窄后，微分推进偶尔会在某个点丢掉分支
        patched, rejected = [], []
        for i, q in enumerate(qs):
            pos, _, _ = kin.forward(q)
            if np.linalg.norm(pos - np.array(points[i])) > 1e-3 and i > 0:
                trial = kin.solve(points[i], axis_target, qs[i - 1], toward=qs[i - 1],
                                  active=active, refine=True)
                jump = float(np.max(np.abs(trial.q - qs[i - 1])))
                if trial.position_error <= 1e-3 and jump < 0.15:
                    qs[i] = trial.q
                    patched.append(i)
                elif trial.position_error > 1e-3:
                    rejected.append((i, round(trial.position_error * 1000, 2)))
        if patched:
            print(f"局部修补了 {len(patched)} 个点（微分推进丢分支）")
        if rejected:
            print(f"仍无法解出的点：{rejected[:5]}")

    errors, margins, bindings = [], [], []
    for i, q in enumerate(qs):
        position, axis, _ = kin.forward(q)
        errors.append(float(np.linalg.norm(position - np.array(points[i]))))
        margins.append(kin.margin(q)[0])
        bindings.append(kin.margin(q)[1])
        if errors[-1] > 1e-3:
            raise SystemExit(f"第 {i} 点连续推进误差 {errors[-1]*1000:.2f} mm，"
                             "扫描线可能超出可达范围")

    distances = np.linspace(0.0, float(zs[-1] - zs[0]), len(zs))
    motion = LocalPathMotion(distances, qs, args.speed, args.joint_speed,
                             args.joint_acceleration, args.cycles)
    check = check_trajectory(kin, motion, points, axis_target, wall_x, args.standoff)

    report = {
        "standoff_m": args.standoff, "tcp_x_m": tcp_x, "y_m": args.y,
        "z_bottom_m": args.z_bottom, "z_top_m": args.z_top,
        "length_m": float(zs[-1] - zs[0]), "points": args.points,
        "joints": JOINTS, "locked_joints": list(args.lock), "q_rad": qs.round(9).tolist(),
        "requested_peak_speed_m_s": args.speed,
        "joint_speed_limit_rad_s": args.joint_speed,
        "joint_acceleration_limit_rad_s2": args.joint_acceleration,
        "planned_peak_path_speed_m_s": motion.peak_path_speed,
        "scan_seconds_one_way": motion.one_way_duration,
        "scan_seconds_total": motion.duration,
        "certification_scale": motion.certification_scale,
        "ik_max_position_error_mm": float(max(errors) * 1000),
        "ik_min_limit_margin_deg": float(np.degrees(min(margins))),
        "binding_joint": max(set(bindings), key=bindings.count),
        "trajectory_check": check,
    }
    PROFILE.parent.mkdir(parents=True, exist_ok=True)
    PROFILE.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"扫描线：x={tcp_x:.3f}（站距 {args.standoff:.3f}） y={args.y:.2f} "
          f"z={args.z_bottom:.2f}→{args.z_top:.2f}（{report['length_m']*100:.0f} cm，{args.points} 点）")
    print(f"IK：最大位置误差 {report['ik_max_position_error_mm']:.4f} mm，"
          f"最小限位余量 {report['ik_min_limit_margin_deg']:.1f}°")
    print(f"时间规划：峰值路径速度 {motion.peak_path_speed:.4f} m/s，"
          f"单程 {motion.one_way_duration:.2f} s，往返 {motion.duration:.2f} s，"
          f"认证倍率 {motion.certification_scale:.3f}")
    print(f"整轨复核（{check['samples']} 个采样点，10 ms）："
          f"横向偏差 {check['max_transverse_mm']:.3f} mm，轴线 {check['max_axis_deg']:.3f}°，"
          f"离箱面最小 {check['min_wall_gap_m']*1000:.1f} mm，"
          f"末端峰值 {check['max_tip_speed_m_s']*1000:.1f} mm/s，"
          f"限位余量 {check['min_limit_margin_deg']:.1f}°，"
          f"最大穿透 {check['worst_penetration_mm']:.2f} mm")
    print(f"轨迹文件：{PROFILE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
