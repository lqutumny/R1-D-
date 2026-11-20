#!/usr/bin/env python3
"""用闭环运行的 CSV（实际关节角）渲染三帧对比图，验证仿真里真实发生的动作。"""
import argparse
import csv
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))
from kinematics import JOINTS, SCENE  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=ROOT / "out" / "scan_run.csv")
    parser.add_argument("--camera", default="cam_side")
    parser.add_argument("--out", type=Path, default=ROOT / "out" / "closed_loop_frames.png")
    args = parser.parse_args()

    rows = list(csv.DictReader(args.csv.open(encoding="utf-8")))
    scan_rows = [r for r in rows if r["phase"] == "scan-up"]
    if not scan_rows:
        raise SystemExit("CSV 里没有 scan-up 阶段的数据")
    picks = [scan_rows[0], scan_rows[len(scan_rows) // 2], scan_rows[-1]]
    labels = ["scan-up start", "scan-up middle", "scan-up top"]

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), 640)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), 720)
    data = mujoco.MjData(model)
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS]
    renderer = mujoco.Renderer(model, 720, 640)

    frames = []
    for row, label in zip(picks, labels):
        data.qpos[qadr] = [float(row[f"q_act_{n}"]) for n in JOINTS]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=args.camera)
        image = Image.fromarray(renderer.render())
        ImageDraw.Draw(image).text((12, 10), f"{label}  t={float(row['t']):.1f}s",
                                   fill=(255, 255, 255))
        frames.append(image)

    canvas = Image.new("RGB", (sum(f.width for f in frames), frames[0].height))
    x = 0
    for frame in frames:
        canvas.paste(frame, (x, 0))
        x += frame.width
    args.out.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.out)
    print("wrote", args.out, canvas.size)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
