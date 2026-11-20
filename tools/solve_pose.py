#!/usr/bin/env python3
"""求一个"探头对准箱面"的姿态（阶段 3 的最小 IK）。

约束：TCP 位置 3 个 + 探头轴线方向 2 个（共 5 个），8 个关节 → 2 个冗余自由度。
主残差里不放参考项；参考项只作为很小的正则，用来选一个"肘部别太别扭"的解。
"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "models" / "r1_a7_scene.xml"
OUT = ROOT / "out" / "pose.json"

JOINTS = [
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
]


def kinematics(model, data, q, qadr):
    data.qpos[qadr] = q
    mujoco.mj_kinematics(model, data)
    return data


def solve(model, target, axis_target, seed, reg=0.02):
    data = mujoco.MjData(model)
    jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS]
    qadr = np.array([model.jnt_qposadr[j] for j in jids])
    lo = np.array([model.jnt_range[j][0] for j in jids])
    hi = np.array([model.jnt_range[j][1] for j in jids])
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    wrist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")

    def residual(q):
        kinematics(model, data, q, qadr)
        tcp = data.site_xpos[site]
        axis = data.xmat[wrist].reshape(3, 3)[:, 0]   # 探头沿腕部局部 +x（前臂方向）
        # 软限位惩罚：离限位不足 0.15 rad 就开始推回来
        penalty = 0.05 * (np.maximum(0.0, lo + 0.15 - q) + np.maximum(0.0, q - (hi - 0.15)))
        return np.r_[tcp - target, 0.6 * (axis - axis_target), reg * (q - seed), penalty]

    solution = least_squares(residual, np.clip(seed, lo + 1e-6, hi - 1e-6),
                             bounds=(lo, hi), xtol=1e-12, ftol=1e-12, gtol=1e-12,
                             max_nfev=400, method="trf")
    q = solution.x
    kinematics(model, data, q, qadr)
    position_error = float(np.linalg.norm(data.site_xpos[site] - target))
    axis_error = float(np.degrees(np.arccos(np.clip(
        data.xmat[wrist].reshape(3, 3)[:, 0] @ axis_target, -1, 1))))
    return q, position_error, axis_error, lo, hi


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument("--target", type=float, nargs=3, default=[0.50, 0.0, 1.20],
                        help="TCP 目标位置（世界系）")
    parser.add_argument("--axis", type=float, nargs=3, default=[1.0, 0.0, 0.0],
                        help="探头轴线目标方向（世界系），默认指向 +x 的箱面")
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    target = np.array(args.target, float)
    axis_target = np.array(args.axis, float)
    axis_target /= np.linalg.norm(axis_target)

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    seeds = [
        np.array([0.0, 0.5, 0.2, 0.0, 0.9, 0.0, -0.9, 0.0]),
        np.array([0.0, 0.9, 0.1, 0.2, 1.2, 0.0, -1.2, 0.0]),
        np.array([0.0, 0.3, 0.4, -0.2, 0.6, 0.0, -0.6, 0.0]),
        np.zeros(8),
    ]
    best = None
    for seed in seeds:
        q, pe, ae, lo, hi = solve(model, target, axis_target, seed)
        score = pe * 1000 + ae
        if best is None or score < best[0]:
            best = (score, q, pe, ae, lo, hi)
        if pe < 1e-3 and ae < 0.5:
            break
    _, q, pe, ae, lo, hi = best
    margin = float(np.min(np.minimum(q - lo, hi - q)))
    print(f"目标 TCP: {target.round(3)}  轴线: {axis_target.round(3)}")
    print(f"位置误差 {pe*1000:.2f} mm，轴线误差 {ae:.2f}°，最小限位余量 {np.degrees(margin):.1f}°")
    print("关节角(deg): " + ", ".join(f"{n.replace('_joint','')}={np.degrees(v):+.1f}"
                                     for n, v in zip(JOINTS, q)))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({
        "joints": JOINTS, "q_rad": q.tolist(),
        "target": target.tolist(), "axis": axis_target.tolist(),
        "position_error_mm": pe * 1000, "axis_error_deg": ae,
        "min_limit_margin_deg": float(np.degrees(margin)),
    }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("saved", args.out)
    return 0 if pe < 1e-2 else 1


if __name__ == "__main__":
    raise SystemExit(main())
