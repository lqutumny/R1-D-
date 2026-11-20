#!/usr/bin/env python3
"""把扫描轨迹的三帧（下端/中段/上端）渲染成一张对比图。"""
import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))
from kinematics import JOINTS, SCENE  # noqa: E402

PROFILE = ROOT / "models" / "scan_profile.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument("--profile", type=Path, default=PROFILE)
    parser.add_argument("--camera", default="cam_side")
    parser.add_argument("--out", type=Path, default=ROOT / "out" / "scan_frames.png")
    args = parser.parse_args()

    profile = json.loads(args.profile.read_text(encoding="utf-8"))
    qs = np.array(profile["q_rad"])
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), 640)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), 720)
    data = mujoco.MjData(model)
    qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS]
    renderer = mujoco.Renderer(model, 720, 640)

    frames = []
    for index, label in ((0, "bottom"), (len(qs) // 2, "middle"), (-1, "top")):
        data.qpos[qadr] = qs[index]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=args.camera)
        image = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(image)
        draw.text((12, 10), f"{label}  z={profile['z_bottom_m'] if index == 0 else (profile['z_top_m'] if index == -1 else (profile['z_bottom_m']+profile['z_top_m'])/2):.2f} m",
                  fill=(255, 255, 255))
        frames.append(image)

    width = sum(f.width for f in frames)
    canvas = Image.new("RGB", (width, frames[0].height))
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
