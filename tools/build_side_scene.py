#!/usr/bin/env python3
"""从现有升降模型生成固定立柱、箱面朝机器人左侧的单臂场景。"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / "models"


def build_scene(output, wall_y, model_gap, probe_envelope=None, probe_cross_section=None):
    tree = ET.parse(MODELS / "r1_a7_scene_lift.xml")
    root = tree.getroot()
    manifest = json.loads((MODELS / "simulation_lift.json").read_text())
    nominal = float(manifest["lift_nominal_gap_m"])
    shift = model_gap - nominal
    column = root.find(".//body[@name='lift_column']")
    column.remove(column.find("joint[@name='lift_slide']"))
    column.set("pos", f"0 0 {shift:.6f}")
    middle = root.find(".//body[@name='lift_middle_visual']")
    del middle.attrib["mocap"]
    center = float(middle.attrib["pos"].split()[2])
    middle.set("pos", f"0 0 {center + shift / 2:.6f}")
    probe = root.find(".//body[@name='radiate_probe']")
    if probe_envelope is not None or probe_cross_section is not None:
        body = probe.find("geom[@name='probe_body']")
        width, height, original_half = map(float, body.attrib["size"].split())
        if probe_cross_section is not None:
            width, height = (side / 2 for side in probe_cross_section)
        if probe_envelope is not None and probe_envelope < 2 * original_half:
            raise ValueError("探头防碰撞包络不能短于原模型")
        half = original_half if probe_envelope is None else probe_envelope / 2
        body.set("size", f"{width} {height} {half}")
        body.set("pos", f"0 0 {half}")
    face = probe.find("geom[@name='probe_face']")
    face_center = [float(v) for v in face.attrib["pos"].split()]
    face_center[2] += float(face.attrib["size"].split()[1])
    probe.find("site[@name='tcp']").set("pos", " ".join(f"{v:.6f}" for v in face_center))
    wall = root.find(".//geom[@name='wall']")
    wall.set("size", "1.20 0.02 1.20")
    wall.set("pos", f"0 {wall_y + .02:.6f} 1.2")
    output.parent.mkdir(parents=True, exist_ok=True)
    root.find("compiler").set("meshdir", str(MODELS / "meshes") if output.parent.resolve() != MODELS else "meshes")
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=MODELS / "r1_a7_scene_side.xml")
    parser.add_argument("--wall-y", type=float, required=True, help="+Y 箱面近侧坐标（m）")
    parser.add_argument("--model-gap", type=float, default=0.505, help="模型中机身与底盘间隙 m")
    options = parser.parse_args()
    build_scene(options.output, options.wall_y, options.model_gap)
