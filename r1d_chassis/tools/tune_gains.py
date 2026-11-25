#!/usr/bin/env python3
"""按"目标阻尼比"标定模型关节阻尼，保持指令里的 kp/kd 为官方值。

原理：被控对象近似为 I·q̈ = τ_pd，τ_pd = kp·e − kd·q̇。
闭环自然频率 ω=√(kp/I)，阻尼比 ζ=(kd+b)/(2√(kp·I))，其中 b 是模型里没建模的
齿轮摩擦/反电动势阻尼。给定目标 ζ（默认 0.8），可反解 b。
I 由 MuJoCo 质量矩阵在标定姿态下取对角元得到。
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))
from profile import load as load_profile  # noqa: E402

SCENE = ROOT / "models" / "r1_a7_scene.xml"
PROFILE = ROOT / "models" / "scan_profile.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=SCENE)
    parser.add_argument("--pose", type=Path, default=PROFILE,
                        help="用于标定的姿态（默认扫描起点）")
    parser.add_argument("--zeta", type=float, default=0.8)
    parser.add_argument("--write", action="store_true", help="把结果写回场景 XML")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "gains_tuned.json")
    args = parser.parse_args()

    profile = load_profile()
    names = [j.name for j in profile.joints]
    kp = np.array([j.kp for j in profile.joints], float)
    kd = np.array([j.kd for j in profile.joints], float)

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
    qadr = np.array([model.jnt_qposadr[j] for j in jids])
    dadr = np.array([model.jnt_dofadr[j] for j in jids])

    if args.pose.exists():
        saved = json.loads(args.pose.read_text(encoding="utf-8"))
        by_name = dict(zip(saved["joints"], saved["q_rad"][0]))
        q = np.array([by_name.get(name, 0.0) for name in names])
    else:
        q = np.zeros(len(names))
    data.qpos[qadr] = q
    mujoco.mj_forward(model, data)

    matrix = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, matrix)   # MuJoCo 3.2+ 签名：(model, data, dst)
    inertia = np.diag(matrix)[dadr]

    rows, damping = [], {}
    for i, name in enumerate(names):
        stiff = float(kp[i])
        damp_cmd = float(kd[i])
        omega = np.sqrt(max(stiff, 1e-9) / max(inertia[i], 1e-9))
        need = 2.0 * args.zeta * np.sqrt(stiff * inertia[i]) - damp_cmd
        need = max(need, 0.0)
        damping[name] = round(float(need), 3)
        rows.append({"joint": name, "inertia_kg_m2": round(float(inertia[i]), 4),
                     "kp": stiff, "kd_cmd": damp_cmd,
                     "omega_rad_s": round(float(omega), 2),
                     "zeta_without_model_damping": round(float(damp_cmd / (2 * np.sqrt(stiff * inertia[i]))), 3),
                     "model_damping": round(float(need), 3),
                     "tau_at_0p4_rad_s": round(float((damp_cmd + need) * 0.4), 1)})

    print(f"{'关节':<30} {'I(kg·m²)':>9} {'ω(rad/s)':>9} {'原ζ':>6} {'r加阻尼':>8} {'0.4rad/s力矩':>12}")
    for r in rows:
        print(f"{r['joint']:<30} {r['inertia_kg_m2']:>9.3f} {r['omega_rad_s']:>9.2f} "
              f"{r['zeta_without_model_damping']:>6.3f} {r['model_damping']:>8.2f} "
              f"{r['tau_at_0p4_rad_s']:>11.1f} N·m")

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps({"zeta_target": args.zeta, "rows": rows},
                                      indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    if args.write:
        tree = ET.parse(args.scene)
        root = tree.getroot()
        changed = 0
        for joint in root.iter("joint"):
            name = joint.get("name")
            if name in damping:
                joint.set("damping", f"{damping[name]:.3f}")
                changed += 1
        ET.indent(tree, space="  ")
        tree.write(args.scene, encoding="utf-8", xml_declaration=True)
        print(f"已写回 {changed} 个关节的 damping -> {args.scene}")
    print(f"报告: {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
