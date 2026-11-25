#!/usr/bin/env python3
"""用实测关节角反算探头端面（TCP）轨迹，给出真机的直线度与朝向偏差。

原理：CSV 里记的是**实测** q（不是指令），把它代进 MuJoCo 模型的运动学，
算出 TCP 位置和探头轴线方向。于是"机器人实际把探头端面走成了什么形状"就是可量的。

前提：真机的连杆几何/零位与 URDF 一致。如果零位有偏差，绝对位置会整体偏，
但**形状**（直线度、竖直度）对零位不敏感，仍然可信。这一点必须在结论里写清楚。

用法：
  python3 tools/tcp_from_csv.py out/scan_real2.csv
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


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--scene", type=Path, default=ROOT / "models" / "r1_a7_scene.xml")
    parser.add_argument("--phase", default="scan-up", help="只统计这个阶段（默认 scan-up）")
    args = parser.parse_args()

    names = [j.name for j in load_profile().joints]
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    if site < 0:
        raise SystemExit("场景里没有名为 tcp 的 site")

    rows = [r for r in csv.DictReader(args.csv_path.open(encoding="utf-8"))
            if r["phase"] == args.phase]
    if len(rows) < 10:
        raise SystemExit(f"阶段 {args.phase} 只有 {len(rows)} 个采样点")

    tcp, axis = [], []
    for r in rows:
        for i, n in enumerate(names):
            data.qpos[qadr[i]] = float(r[f"q_act_{n}"])
        mujoco.mj_kinematics(model, data)
        tcp.append(data.site_xpos[site].copy())
        # 探头轴线 = 腕部局部 +x（工具就是沿这个方向伸出的）
        wrist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")
        axis.append(data.xmat[wrist].reshape(3, 3)[:, 0].copy())
    tcp = np.array(tcp)
    axis = np.array(axis)
    t = np.array([float(r["t"]) for r in rows])

    # 直线拟合（在 x-z 平面里，扫描线是竖直的）：残差 = 直线度
    z = tcp[:, 2]
    x = tcp[:, 0]
    fit = np.polyfit(z, x, 1)
    residual_mm = np.abs(x - np.polyval(fit, z)) * 1000
    axis_err = np.degrees(np.arccos(np.clip(axis[:, 0], -1, 1)))   # 与 +x 的夹角

    print(f"阶段 {args.phase}：{len(rows)} 个采样点，{t[0]:.1f}–{t[-1]:.1f} s（{t[-1]-t[0]:.1f} s）")
    print(f"TCP 竖直行程   : {np.ptp(z)*1000:7.1f} mm  (z {z.min():.3f} → {z.max():.3f} m)")
    print(f"TCP 横向摆动   : {np.ptp(tcp[:,1])*1000:7.1f} mm  (y)")
    print(f"直线度（x-z 拟合残差）: 最大 {residual_mm.max():.1f} mm，RMS {np.sqrt((residual_mm**2).mean()):.1f} mm")
    print(f"离箱面距离     : {fit[0]:.0f}·z + {fit[1]:.3f}，"
          f"即 z 中位处 {np.polyval(fit, np.median(z)):.3f} m")
    print(f"探头轴线偏差   : 最大 {axis_err.max():.2f}°，均值 {axis_err.mean():.2f}°（相对 +x）")
    print(f"竖直速度       : 平均 {np.ptp(z)/(t[-1]-t[0])*1000:.1f} mm/s")
    print()
    print("注：以上由实测关节角 + URDF 模型反算。若真机零位与 URDF 有偏差，绝对数值会整体平移，"
          "但直线度/竖直度这类形状量仍然可信。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
