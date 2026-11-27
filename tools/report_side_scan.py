#!/usr/bin/env python3
"""从回环仿真的关节反馈反算左侧扫描的探头轨迹。"""
import argparse
import csv
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))
from kinematics import ArmKinematics, JOINTS
from transfer import Clearance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    args = parser.parse_args()
    spec = json.loads(args.profile.read_text())
    report = json.loads((args.output / "scan_report.json").read_text())
    kin = ArmKinematics(args.scene)
    hold_names = report["hold_joints"]
    hold_angles = np.radians(report["hold_captured_deg"])
    raw = json.loads((ROOT / "r1d_scan/robot_profile.json").read_text())
    held = {name: angle for name, angle in raw["held"].items() if name != "note"}
    held.update(zip(hold_names, hold_angles))
    space = Clearance(kin, kin.lo, kin.hi, spec["wall_y_m"], held, wall_axis=1)
    with (args.output / "arm.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    phase_samples = {name: [] for name in ("scan-up", "scan-down")}
    finished = False
    violations = []
    all_gaps = []
    min_margin = float("inf")
    max_penetration_mm = 0.0
    min_base_gap = 0.20
    for row in rows:
        q = np.array([float(row["q_act_" + name]) for name in JOINTS])
        t = float(row["t"])
        phase = row["phase"]
        issue = space.issue(q)
        if issue:
            violations.append({"time_s": t, "phase": phase, "issue": issue})
        else:
            for contact in kin.data.contact:
                if contact.geom1 in space.left or contact.geom2 in space.left:
                    max_penetration_mm = max(max_penetration_mm, -contact.dist * 1000)
            for left in space.left:
                for base in space.base:
                    min_base_gap = min(min_base_gap, mujoco.mj_geomDistance(
                        kin.model, kin.data, left, base, 0.20, None))
        point, axis, _ = kin.forward(q)
        gap = (spec["wall_y_m"] - point[1]) * 1000
        all_gaps.append(gap)
        min_margin = min(min_margin, kin.margin(q)[0])
        finished = finished or phase == "finished-hold"
        if phase in phase_samples:
            q_target = np.array([float(row["q_cmd_" + name]) for name in JOINTS])
            z_target = kin.tcp(q_target)[2]
            angle = float(np.degrees(np.arccos(np.clip(axis[1], -1, 1))))
            phase_samples[phase].append((t, point, gap, angle, z_target))

    result = {"complete": report["complete"], "fault": report["fault"], "scan_phases": {},
              "whole_cycle_min_standoff_mm": min(all_gaps) if all_gaps else None,
              "min_arm_to_base_mm": min_base_gap * 1000,
              "max_arm_contact_penetration_mm": max_penetration_mm,
              "geometry_violations": violations[:20], "geometry_violation_count": len(violations),
              "joint_limit_margin_deg": float(np.degrees(min_margin)) if rows else None}
    for phase in ("scan-up", "scan-down"):
        samples = phase_samples[phase]
        if not samples:
            continue
        points = np.array([a[1] for a in samples])
        times = np.array([a[0] for a in samples])
        reference_z = np.array([a[4] for a in samples])
        # 160 ms 窗口降低离散反馈及 DDS 时间抖动对差分速度的影响。
        half_window = .08
        centers = np.arange(times[0] + half_window, times[-1] - half_window, .02)
        result["scan_phases"][phase] = {
            "sampled_duration_s": round(samples[-1][0] - samples[0][0], 3),
            "z_range_mm": (1000 * np.array([points[:, 2].min(), points[:, 2].max()])).tolist(),
            "travel_mm": float(1000 * np.ptp(points[:, 2])),
            "wall_gap_mm": [min(a[2] for a in samples), max(a[2] for a in samples)],
            "maximum_transverse_mm": float(1000 * np.linalg.norm(
                points[:, :2] - [spec["tcp_x_m"], spec["y_m"]], axis=1).max()),
            "maximum_probe_angle_deg": max(a[3] for a in samples),
        }
        if len(centers):
            start_times, end_times = centers - half_window, centers + half_window
            heights = np.stack([points[:, 2], reference_z])
            velocity = np.array([np.interp(end_times, times, z) - np.interp(start_times, times, z)
                                 for z in heights]) * (1000 / (2 * half_window))
            actual, target = velocity
            result["scan_phases"][phase].update(
                vertical_speed_p95_mm_s=float(np.percentile(np.abs(actual), 95)),
                vertical_speed_tracking_error_p95_mm_s=float(np.percentile(np.abs(actual - target), 95)))
    if finished:
        q = np.radians(report["measured_final_q_deg"])
        _, axis, _ = kin.forward(q)
        result["stow_probe_angle_from_down_deg"] = float(np.degrees(np.arccos(
            np.clip(-axis[2], -1, 1))))
    up, down = phase_samples["scan-up"], phase_samples["scan-down"]
    if up and down:
        result["turn_z_mm"] = 1000 * max(max(a[1][2] for a in up), max(a[1][2] for a in down))
        before = [a for a in up if a[0] >= up[-1][0] - 0.25]
        after = [a for a in down if a[0] <= down[0][0] + 0.25]
        result["turn_mean_vertical_speed_mm_s"] = [
            float(1000 * (window[-1][1][2] - window[0][1][2]) / (window[-1][0] - window[0][0]))
            if len(window) >= 2 else None for window in (before, after)
        ]
    checks = result["scan_phases"]
    result["meets_geometry"] = (report["complete"] and not violations and len(checks) == 2
                                and result["joint_limit_margin_deg"] >= 4.0
                                and result.get("stow_probe_angle_from_down_deg", float("inf")) <= 5
                                and all(abs(s["travel_mm"] - 1000 * spec["length_m"]) <= 20
                                        and 180 <= s["wall_gap_mm"][0] <= s["wall_gap_mm"][1] <= 220
                                        and s["maximum_probe_angle_deg"] <= 10
                                        and s["maximum_transverse_mm"] <= 20
                                        for s in checks.values()))
    payload = json.dumps(result, indent=2, ensure_ascii=False)
    (args.output / "side_geometry.json").write_text(payload + "\n")
    print(payload)
    return 0 if result["meets_geometry"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
