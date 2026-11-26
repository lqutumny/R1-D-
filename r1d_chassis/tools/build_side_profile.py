#!/usr/bin/env python3
"""从左侧扫描搜索结果选出一条指定行程，进行整轮预检并保存。"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

from build_side_scene import build_scene

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))
from controller import Controller, build_motion, side_scan_limits
from kinematics import ArmKinematics
import preflight
from transfer import Clearance, plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--length", type=float, required=True)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--model-gap", type=float, default=.505)
    parser.add_argument("--probe-envelope", type=float, help="仅扩大探头轴向防碰撞包络（m）")
    parser.add_argument("--probe-cross-section", type=float, nargs=2, metavar=("WIDTH", "HEIGHT"),
                        help="替代件的碰撞截面尺寸（m），不改变 TCP 或重力模型")
    parser.add_argument("--scene", type=Path, default=ROOT / "models/r1_a7_scene_side.xml")
    parser.add_argument("--profile", type=Path, default=ROOT / "models/side_scan_profile.json")
    timing_options = {
        "--peak-speed": ("requested_peak_speed_m_s", "探头峰值速度上限（m/s）"),
        "--joint-speed": ("joint_speed_limit_rad_s", "全轮关节速度上限（rad/s）"),
        "--joint-accel": ("joint_acceleration_limit_rad_s2", "全轮关节加速度上限（rad/s²）"),
        "--smooth-window": ("time_smooth_s", "扫描时间平滑窗口（s）"),
        "--transfer-speed": ("transfer_joint_speed_rad_s", "接近和收纳关节限速（rad/s）"),
    }
    for flag, (key, description) in timing_options.items():
        parser.add_argument(flag, dest=key, type=float, help=description)
    args = parser.parse_args()
    if args.length <= 0 or not np.isclose(args.length * 100, round(args.length * 100)):
        parser.error("扫描长度须为正，且为 0.01 m 的整数倍")
    candidates = json.loads(args.candidates.read_text())
    selected = next((c for c in candidates if np.isclose(c["tcp_x_m"], args.x)
                     and np.isclose(c["y_m"], args.y)), None)
    n = round(args.length * 100) + 1
    if selected is None or len(selected.get("q_rad", ())) < n:
        raise SystemExit("未找到指定站位和长度的连续 IK 解")
    base = json.loads((ROOT / "models/sync_scan_profile.json").read_text())
    keys = ("requested_peak_speed_m_s", "joint_speed_limit_rad_s",
            "joint_acceleration_limit_rad_s2", "joint_jerk_limit_rad_s3", "time_smooth_s",
            "fit_interior_knots")
    spec = {key: base[key] for key in keys}
    for key, description in timing_options.values():
        value = getattr(args, key)
        if value is not None:
            if not np.isfinite(value) or value <= 0.0:
                parser.error(f"{description}须是有限正数")
            spec[key] = value
    spec.update(wall_axis=1, tcp_x_m=args.x, y_m=args.y, wall_y_m=args.y + 0.20,
                model_gap_m=args.model_gap, standoff_m=0.20, length_m=args.length,
                points=n, z_bottom_m=selected["z_bottom_m"],
                z_top_m=selected["z_bottom_m"] + args.length, q_rad=selected["q_rad"][:n])
    if args.probe_envelope is not None:
        if not np.isfinite(args.probe_envelope):
            parser.error("探头包络必须是有限值")
        spec["probe_envelope_m"] = args.probe_envelope
    if args.probe_cross_section is not None:
        if not np.isfinite(args.probe_cross_section).all() or min(args.probe_cross_section) <= 0:
            parser.error("探头碰撞截面尺寸须为有限正值")
        spec["probe_cross_section_m"] = args.probe_cross_section
    build_scene(args.scene, spec["wall_y_m"], args.model_gap, args.probe_envelope,
                args.probe_cross_section)
    kin = ArmKinematics(args.scene)
    raw = json.loads((ROOT / "r1d_scan/robot_profile.json").read_text())
    spec["joints"] = [j["name"] for j in raw["joints"]]
    lo, hi, kp, kd, effort = [np.array([j[key] for j in raw["joints"]])
                              for key in ("lower", "upper", "kp", "kd", "effort")]
    typical = np.array([raw["initial"].get(j, 0.0) for j in spec["joints"]])
    stow = np.radians([raw["stow_deg"][j] for j in spec["joints"]])
    motion = build_motion(spec)
    c = Controller(typical, motion, lo, hi, kp, kd, effort,
                   limits=side_scan_limits(spec), scene=args.scene)
    held = {name: angle for name, angle in raw["held"].items() if name != "note"}
    space = Clearance(kin, lo + c.limits.margin, hi - c.limits.margin,
                      spec["wall_y_m"], held, wall_axis=1)
    approach = plan(space, typical, c.q_scan_start, "左侧接近")
    retract = plan(space, motion.sample(motion.duration), stow, "收纳", via=(typical,))
    c.set_transfers(approach, retract, ready_seconds=.8)
    check = preflight.run(c, motion, spec, c.limits, kin, clearance=space)
    spec["preflight"] = check.report()
    spec["scan_seconds_one_way"] = motion.one_way_duration
    print(json.dumps(spec["preflight"], indent=2, ensure_ascii=False), flush=True)
    if not check.ok or check.min_limit_margin_deg < 4:
        raise SystemExit("整轮预检失败或关节余量低于 4°")
    args.profile.parent.mkdir(parents=True, exist_ok=True)
    args.profile.write_text(json.dumps(spec, indent=2, ensure_ascii=False) + "\n")
    print(f"单程 {args.length:.2f} m / {motion.one_way_duration:.2f} s，轨迹 {args.profile}")


if __name__ == "__main__":
    main()
