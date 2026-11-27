#!/usr/bin/env python3
"""搜索左侧竖直扫描线，生成可执行轨迹与几何检验结果。"""
import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.optimize import least_squares

from build_side_scene import build_scene

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))
from kinematics import ArmKinematics
from transfer import Clearance

MARGIN_RAD = np.radians(4)
STEP_M = 0.01
NORMAL = np.array([0., 1., 0.])
STANDOFF_M = 0.20


class IKSolver:
    """位置与探头轴向为主约束，用上一点约束冗余关节漂移。"""

    def __init__(self, kin, lo, hi):
        self.kin, self.lo, self.hi = kin, lo, hi
        self.jac_pos = np.zeros((3, kin.model.nv))
        self.jac_rot = np.zeros_like(self.jac_pos)

    def solve(self, target, seed, weight):
        k = self.kin
        seed = np.clip(seed, self.lo + 1e-8, self.hi - 1e-8)

        def residual(q):
            pos, axis, _ = k.forward(q)
            return np.r_[pos - target, 0.18 * (axis - NORMAL), weight * (q - seed)]

        def jacobian(q):
            _, axis, _ = k.forward(q)
            mujoco.mj_comPos(k.model, k.data)
            mujoco.mj_jacSite(k.model, k.data, self.jac_pos, self.jac_rot, k.tcp_site)
            rotation = self.jac_rot[:, k.dadr]
            return np.vstack((self.jac_pos[:, k.dadr],
                              0.18 * np.cross(rotation.T, axis).T, weight * np.eye(len(q))))

        return least_squares(residual, seed, jac=jacobian, bounds=(self.lo, self.hi),
                             max_nfev=60, ftol=1e-9, xtol=1e-9, gtol=1e-9).x


def error(kin, q, target):
    pos, axis, _ = kin.forward(q)
    return float(np.linalg.norm(pos - target)), float(np.degrees(np.arccos(
        np.clip(axis @ NORMAL, -1, 1))))


def trace_line(solver, space, x, y, bottom, top, seeds):
    best = None
    heights = bottom + STEP_M * np.arange(round((top - bottom) / STEP_M) + 1)
    failures = []
    for seed in seeds:
        path = []
        reason = "探索上限"
        for z in heights:
            target = np.array([x, y, z])
            q = solver.solve(target, path[-1] if path else seed, .004 if path else .001)
            position_error, angle = error(solver.kin, q, target)
            if position_error > .002 or angle > 2:
                reason = f"IK 残差 {position_error * 1000:.1f} mm / {angle:.1f}°"
                break
            issue = space.issue(q)
            if issue:
                reason = issue
                break
            if path and not space.segment_is_clear(path[-1], q):
                reason = "相邻姿态无法安全连续连接"
                break
            path.append(q)
        if not path:
            failures.append(reason)
            continue
        quality = min(solver.kin.margin(q)[0] for q in path)
        score = (len(path), quality)
        if best is None or score > best[0]:
            best = (score, np.array(path), reason)
    if best is None:
        return None, "; ".join(dict.fromkeys(failures))
    return best[1], best[2]


def get_scene(model_gap, output, y):
    path = output / "search_scene.xml"
    build_scene(path, y + STANDOFF_M, model_gap)
    return ArmKinematics(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "out/side_scan_search")
    parser.add_argument("--bottom", type=float, default=.585,
                        help="保留原扫描下端相对机身的高度，默认对应模型 DDS 400 mm")
    parser.add_argument("--top-cap", type=float, default=1.40)
    parser.add_argument("--model-gap", type=float, default=.505)
    parser.add_argument("--x", type=float, nargs="+", default=[-.10, 0., .10, .20])
    parser.add_argument("--y", type=float, nargs="+", default=[.30, .36, .42, .48])
    parser.add_argument("--random-seeds", type=int, default=10)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    raw = json.loads((ROOT / "r1d_scan/robot_profile.json").read_text())
    lo = np.array([j["lower"] for j in raw["joints"]])
    hi = np.array([j["upper"] for j in raw["joints"]])
    held = {name: value for name, value in raw["held"].items() if name != "note"}
    rng = np.random.default_rng(60)
    seeds = np.vstack([np.zeros(len(lo)), (lo + hi) / 2,
                       rng.uniform(lo + MARGIN_RAD, hi - MARGIN_RAD, (args.random_seeds, len(lo)))])
    results = []
    for y in args.y:
        k = get_scene(args.model_gap, args.output, y)
        space = Clearance(k, lo + MARGIN_RAD, hi - MARGIN_RAD, y + STANDOFF_M, held, wall_axis=1)
        solver = IKSolver(k, lo + MARGIN_RAD, hi - MARGIN_RAD)
        for x in args.x:
            path, reason = trace_line(solver, space, x, y, args.bottom, args.top_cap, seeds)
            entry = {"tcp_x_m": x, "y_m": y, "z_bottom_m": args.bottom, "limit": reason}
            if path is not None:
                entry.update(z_top_m=args.bottom + STEP_M * (len(path) - 1), q_rad=path.tolist())
            results.append(entry)
            print({key: value for key, value in entry.items() if key != "q_rad"}, flush=True)
    (args.output / "candidates.json").write_text(json.dumps(results, indent=2, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
