#!/usr/bin/env python3
"""多次真机运行的重复性：把每轮的实测关节角反算成 TCP 轨迹，比较彼此散差。

判据（P8）：三轮之间的散差 ≤ 30 mm；同时给出每轮自己的直线度。
用途：阶段 5.6 的重复性验收。

注意：反算依赖 URDF 与实际连杆几何一致。零位偏差会让绝对位置整体平移
（三轮同时平移，不影响散差），所以**散差这个指标对零位不敏感**，可以信。

用法：
  python3 tools/repeatability.py out/rep1.csv out/rep2.csv out/rep3.csv
"""
import argparse
import csv
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from profile import load as load_profile  # noqa: E402

DEG = 180.0 / np.pi


def tcp_path(csv_path, phase, model, qadr, site):
    rows = [r for r in csv.DictReader(Path(csv_path).open(encoding="utf-8"))
            if r["phase"] == phase]
    if len(rows) < 10:
        return None, None, None
    names = [j.name for j in load_profile().joints]
    data = mujoco.MjData(model)
    pts = []
    for r in rows:
        for i, n in enumerate(names):
            data.qpos[qadr[i]] = float(r[f"q_act_{n}"])
        mujoco.mj_kinematics(model, data)
        pts.append(data.site_xpos[site].copy())
    return np.array(pts), np.array([float(r["t"]) for r in rows]), names


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csvs", nargs="+", type=Path)
    parser.add_argument("--phase", default="scan-up")
    parser.add_argument("--scene", type=Path, default=ROOT / "models" / "r1_a7_scene.xml")
    args = parser.parse_args()
    if len(args.csvs) < 2:
        raise SystemExit("至少给两个 CSV 才能比重复性")

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    names = [j.name for j in load_profile().joints]
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]

    runs = []
    for path in args.csvs:
        pts, t, _ = tcp_path(path, args.phase, model, qadr, site)
        if pts is None:
            print(f"{path.name}: 阶段 {args.phase} 采样不足，跳过")
            continue
        # 每轮自己的直线度
        z, x = pts[:, 2], pts[:, 0]
        fit = np.polyfit(z, x, 1)
        straight = np.abs(x - np.polyval(fit, z)) * 1000
        runs.append({"name": path.name, "pts": pts, "t": t,
                     "straight_max": straight.max(), "straight_rms": np.sqrt((straight ** 2).mean())})
        print(f"{runs[-1]['name']:<14} {len(pts):4d} 点  "
              f"z {z.min():.3f}~{z.max():.3f} m  直线度 最大 {straight.max():.1f} mm / RMS {np.sqrt((straight**2).mean()):.1f} mm")

    if len(runs) < 2:
        raise SystemExit("可用运行不足")

    # 把每轮按"进度"重采样（都用 0→1 的归一化进度），比较同一进度处的 TCP 位置
    grid = np.linspace(0.05, 0.95, 60)
    resampled = []
    for run in runs:
        z = run["pts"][:, 2]
        idx = np.argsort(z)
        zs, order = z[idx], idx
        zp = np.interp(grid * (zs[-1] - zs[0]) + zs[0], zs, z[order])
        # 用插值得到同一 z 处的 x、y
        xp = np.interp(grid * (zs[-1] - zs[0]) + zs[0], zs, run["pts"][order, 0])
        yp = np.interp(grid * (zs[-1] - zs[0]) + zs[0], zs, run["pts"][order, 1])
        resampled.append(np.stack([xp, yp, zp], axis=1))
    stack = np.stack(resampled)                   # 轮次 × 点 × 3
    spread = (stack.max(axis=0) - stack.min(axis=0)) * 1000.0  # 每点三向极差，mm
    dist = np.linalg.norm(spread, axis=1)

    print()
    print("=== 重复性（同一竖直高度处，多轮 TCP 位置的散差）===")
    print(f"最大散差 {dist.max():.1f} mm（判据 ≤30 mm）")
    print(f"RMS  散差 {np.sqrt((dist**2).mean()):.1f} mm")
    print(f"x 方向最大 {spread[:,0].max():.1f} mm，y 方向最大 {spread[:,1].max():.1f} mm")

    print()
    print("逐段散差（按 z 分 5 段）：")
    for lo, hi in zip(np.linspace(0, len(grid)-1, 5, dtype=int)[:-1],
                      np.linspace(0, len(grid)-1, 5, dtype=int)[1:]):
        seg = dist[lo:hi]
        print(f"  z {resampled[0][lo,2]:.3f}~{resampled[0][hi,2]:.3f} m: 最大 {seg.max():.1f} mm")

    ok = dist.max() <= 30.0
    print()
    print(f"结论：{'通过' if ok else '未通过'}（P8：散差 ≤30 mm，实测 {dist.max():.1f} mm）")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
