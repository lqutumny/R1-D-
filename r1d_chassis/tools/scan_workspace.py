#!/usr/bin/env python3
"""可达性 + 自碰撞扫描：求左臂"伸到多远、能覆盖多高"。

扫两个维度：TCP 的 x（伸出去的远近）和 z（高度）。
判据：位置误差 <1 mm、轴线误差 <0.5°、限位余量 ≥3°、自碰撞穿透 ≤2 mm。

得到可行区间后，实际停机距离 = TCP_x + 站距（探头端面到箱面）。
"""
import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))
from kinematics import ArmKinematics, JOINTS, default_seed, SCENE  # noqa: E402
from profile import load as load_profile  # noqa: E402

FEASIBLE_POS_M = 1e-3
FEASIBLE_AXIS_DEG = 0.5
FEASIBLE_MARGIN_RAD = np.radians(3.0)
FEASIBLE_PENETRATION_M = 0.002


def worst_penetration(kin, q):
    kin.data.qpos[kin.qadr] = q
    mujoco.mj_forward(kin.model, kin.data)
    worst = 0.0
    pair = None
    for contact in kin.data.contact:
        if contact.dist < worst:
            worst = contact.dist
            body1 = mujoco.mj_id2name(kin.model, mujoco.mjtObj.mjOBJ_BODY,
                                      int(kin.model.geom_bodyid[contact.geom1]))
            body2 = mujoco.mj_id2name(kin.model, mujoco.mjtObj.mjOBJ_BODY,
                                      int(kin.model.geom_bodyid[contact.geom2]))
            pair = tuple(sorted([body1, body2]))
    return -worst, pair


def runs_of(values, flags, min_length):
    runs, start, last = [], None, None
    for value, ok in zip(values, flags):
        if ok and start is None:
            start = value
        if ok:
            last = value
        elif start is not None:
            runs.append((start, last))
            start = last = None
    if start is not None:
        runs.append((start, last))
    return [(a, b) for a, b in runs if b - a >= min_length]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument("--tcp-x", type=float, nargs="+",
                        default=[0.40, 0.45, 0.50, 0.55, 0.60, 0.65])
    parser.add_argument("--y", type=float, default=0.14)
    parser.add_argument("--z-min", type=float, default=0.75)
    parser.add_argument("--z-max", type=float, default=1.45)
    parser.add_argument("--z-step", type=float, default=0.04)
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "workspace.json")
    args = parser.parse_args()

    kin = ArmKinematics(args.scene)
    # 与 scan_plan 一致：profile 的限位优先（真机实测的左腕 pitch 只有 ±43°）
    for joint in load_profile().joints:
        if joint.name in JOINTS and joint.lower is not None:
            i = JOINTS.index(joint.name)
            kin.lo[i] = joint.lower
            kin.hi[i] = joint.upper
    zs = np.arange(args.z_min, args.z_max + 1e-9, args.z_step)
    print(f"扫描 TCP x ∈ {args.tcp_x}，y={args.y}，z {args.z_min:.2f}–{args.z_max:.2f} m")
    print(f"{'TCP x':>6} {'可行 z 区间':>22} {'长度':>7} {'最小余量':>9} {'最差穿透':>9}  说明")
    report = {"y": args.y, "z_step": args.z_step, "sweeps": {}}

    for tcp_x in args.tcp_x:
        seed = default_seed()
        flags, rows = [], []
        for z in zs:
            sol = kin.solve([tcp_x, args.y, float(z)], [1.0, 0.0, 0.0], seed)
            penetration, pair = worst_penetration(kin, sol.q)
            ok = (sol.position_error < FEASIBLE_POS_M
                  and sol.axis_error_deg < FEASIBLE_AXIS_DEG
                  and sol.min_margin_rad >= FEASIBLE_MARGIN_RAD
                  and penetration <= FEASIBLE_PENETRATION_M)
            if ok:
                seed = sol.q
            flags.append(ok)
            rows.append({"z": round(float(z), 3), "ok": bool(ok),
                         "pos_err_mm": round(sol.position_error * 1000, 3),
                         "axis_err_deg": round(sol.axis_error_deg, 3),
                         "margin_deg": round(np.degrees(sol.min_margin_rad), 2),
                         "penetration_mm": round(penetration * 1000, 2),
                         "pair": pair, "binding": sol.binding_joint,
                         "q_rad": sol.q.round(6).tolist()})
        runs = runs_of(zs, flags, args.z_step * 1.5)
        report["sweeps"][f"{tcp_x:.2f}"] = {"runs": runs, "rows": rows}
        if runs:
            a, b = max(runs, key=lambda r: r[1] - r[0])
            inside = [r for r in rows if a <= r["z"] <= b]
            print(f"{tcp_x:>6.2f} {a:>10.2f} ~ {b:<8.2f} {b-a:>7.2f} "
                  f"{min(r['margin_deg'] for r in inside):>8.1f}° "
                  f"{max(r['penetration_mm'] for r in inside):>8.1f}mm")
        else:
            causes = {}
            for r in rows:
                if not r["ok"]:
                    key = r["binding"] if r["pos_err_mm"] > 1 else (
                        r["pair"] if r["penetration_mm"] > 2 else "axis/other")
                    causes[str(key)] = causes.get(str(key), 0) + 1
            top = sorted(causes.items(), key=lambda kv: -kv[1])[:2]
            print(f"{tcp_x:>6.2f} {'（无连续可行区间）':>22} {'':>7} {'':>9} {'':>9}  主因 {top}")

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"\n明细: {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
