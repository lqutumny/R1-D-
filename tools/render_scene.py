#!/usr/bin/env python3
"""渲染场景预览图（离屏，借用 X 显示）。用法：DISPLAY=:0 MUJOCO_GL=glfw python3 tools/render_scene.py"""
import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "models" / "r1_a7_scene.xml"
OUT = ROOT / "out"
JOINTS = [
    "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
]


def shoot(renderer, data, path, camera):
    renderer.update_scene(data, camera=camera)
    Image.fromarray(renderer.render()).save(path)
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--pose", type=Path, help="solve_pose.py 输出的 pose.json")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), args.width)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), args.height)
    data = mujoco.MjData(model)
    if args.pose:
        pose = json.loads(args.pose.read_text(encoding="utf-8"))
        for name, value in zip(pose["joints"], pose["q_rad"]):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[jid]] = value
    mujoco.mj_forward(model, data)
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    tcp = data.site_xpos[site]
    renderer = mujoco.Renderer(model, args.height, args.width)

    paths = [
        shoot(renderer, data, args.out / "scene_iso.png", "cam_iso"),
        shoot(renderer, data, args.out / "scene_side.png", "cam_side"),
    ]
    print(f"TCP = ({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f}) m")
    for p in paths:
        print("wrote", p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
