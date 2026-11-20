#!/usr/bin/env python3
"""从官方 r1_a7.urdf 生成 MuJoCo 场景：固定底座 + 地面 + 箱面 + 假探头。

设计要点：
- 只保留"腰 yaw + 左臂 7 关节"为可动关节，其余（右臂、头）设为 fixed；
- 底座抬到指定高度（升降柱升起后的高度），底盘/轮/升降柱不建模；
- 箱面只是一块不参与接触的视觉几何，距离与夹角由规划器算，不靠碰撞求解；
- 探头挂在左腕 link 上，沿工具轴伸出 L，末端有 site "tcp"。
"""
import argparse
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "models" / "r1_a7.urdf"
MESHES = ROOT / "models" / "meshes"
OUT = ROOT / "models" / "r1_a7_scene.xml"
MANIFEST = ROOT / "models" / "simulation.json"
PROFILE = ROOT / "r1d_scan" / "robot_profile.json"

_RAW = json.loads(PROFILE.read_text(encoding="utf-8"))
CONTROL = [joint["name"] for joint in _RAW["joints"]]
HOLD = {name: float(value) for name, value in _RAW.get("held", {}).items() if name != "note"}
# 真机上要让它们"写固定值不动"的关节（右臂）：仿真里必须也给它执行器，
# 否则 --hold servo 模式没法验证 PD 保持到底顶不顶得住。
HOLD_CONTROL = [j["name"] for j in (_RAW.get("hold_on_robot") or {}).get("joints", [])
                if j.get("name") in HOLD]
ACTUATED = CONTROL + HOLD_CONTROL
BASE_LINK = "waist_yaw_link"   # pelvis 在 URDF 转换时被 welded 到 world，它是实际的根 body
TOOL_LINK = "left_wrist_yaw_link"

DEFAULTS = dict(base_z=1.0, probe_len=0.19, probe_width=0.126, probe_face_r=0.0235,
                probe_com=0.095, probe_mass=1.5,
                wall_x=0.75, armature=0.01, timestep=0.001)


def read_limits(root):
    table = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
        table[joint.get("name")] = {
            "lower": float(limit.get("lower")), "upper": float(limit.get("upper")),
            "effort": float(limit.get("effort")),
        }
    return table


def convert(cfg):
    """URDF → MuJoCo XML（只做机械模型转换，不加场景物体）。"""
    tree = ET.parse(URDF)
    root = tree.getroot()
    limits = read_limits(root)
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name in CONTROL or name in HOLD:
            continue                      # 受控关节保持可动；被保持的关节稍后用等式约束锁住
        if joint.get("type") != "fixed":
            joint.set("type", "fixed")
    for mesh in root.iter("mesh"):
        mesh.set("filename", str(MESHES / Path(mesh.get("filename")).name))
    with tempfile.TemporaryDirectory(prefix="r1a7_") as tmp:
        prepared = Path(tmp) / "r1_a7_fixed.urdf"
        tree.write(prepared, encoding="utf-8", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(str(prepared))
        converted = Path(tmp) / "converted.xml"
        mujoco.mj_saveLastXML(str(converted), model)
        scene = ET.parse(converted).getroot()
    return scene, limits


def add_scene(scene, cfg, limits):
    meshdir = scene.find("compiler")
    if meshdir is None:
        meshdir = ET.SubElement(scene, "compiler")
    meshdir.set("meshdir", str(MESHES))
    for mesh in scene.findall("asset/mesh"):
        mesh.set("file", Path(mesh.get("file")).name)

    option = scene.find("option")
    if option is None:
        option = ET.SubElement(scene, "option")
    option.set("timestep", str(cfg["timestep"]))
    option.set("integrator", "implicitfast")
    option.set("gravity", "0 0 -9.81")

    world = scene.find("worldbody")
    base = None
    wrist = None
    for body in world.iter("body"):
        if body.get("name") == BASE_LINK:
            base = body
        elif body.get("name") == TOOL_LINK:
            wrist = body
    if base is None or wrist is None:
        raise SystemExit(f"模型里找不到 {BASE_LINK} 或 {TOOL_LINK}")

    # 整机抬升：pelvis 在 URDF 里没有关节、会被焊到 world，
    # 所以不能只抬腰——要把 world 下所有机器人元素包进一个 mount body 再整体抬到工作高度。
    mount = ET.Element("body", name="robot_mount", pos=f"0 0 {cfg['base_z']:.4f}")
    for child in list(world):
        world.remove(child)
        mount.append(child)
    world.append(mount)

    # 地面
    ET.SubElement(world, "geom", name="floor", type="plane", size="3 3 0.1",
                  pos="0 0 0", rgba="0.22 0.25 0.28 1")
    # 箱面：一块不碰撞的薄板，法向 -x 朝向机器人；距离与夹角由规划器计算
    ET.SubElement(world, "geom", name="wall", type="box",
                  size=f"0.02 {cfg['wall_half']:.3f} {cfg['wall_half']:.3f}",
                  pos=f"{cfg['wall_x']:.4f} 0 {cfg['wall_z']:.4f}",
                  rgba="0.55 0.45 0.30 0.85", contype="0", conaffinity="0")
    # 升降柱示意（纯视觉，不参与碰撞）：底盘板 + 立柱，从地面顶到 mount 底面
    plate_h, mast_h = 0.16, max(0.05, cfg["base_z"] - 0.16)
    ET.SubElement(world, "geom", name="base_plate", type="box",
                  size=f"0.30 0.22 {plate_h/2:.4f}", pos=f"0 0 {plate_h/2:.4f}",
                  rgba="0.30 0.32 0.35 1", contype="0", conaffinity="0")
    ET.SubElement(world, "geom", name="lift_mast", type="box",
                  size=f"0.07 0.07 {mast_h/2:.4f}", pos=f"0 0 {plate_h + mast_h/2:.4f}",
                  rgba="0.42 0.45 0.50 1", contype="0", conaffinity="0")
    ET.SubElement(world, "light", name="key", pos="1.5 -1.5 2.5", dir="-0.4 0.4 -1")

    # 固定机位（避免方位角猜错被箱面挡住）
    target = np.array([0.20, 0.0, cfg["base_z"] + 0.10])
    for name, offset in (("cam_iso", np.array([-1.3, -1.6, 0.9])),
                         ("cam_side", np.array([0.0, 2.8, 0.20])),   # 机器人左侧（+y）
                         ("cam_front", np.array([-0.7, -0.9, 0.3]))):
        pos = target + offset
        zaxis = pos - target
        zaxis = zaxis / np.linalg.norm(zaxis)
        xaxis = np.cross([0.0, 0.0, 1.0], zaxis)
        xaxis = xaxis / np.linalg.norm(xaxis)
        yaxis = np.cross(zaxis, xaxis)
        ET.SubElement(world, "camera", name=name,
                      pos=" ".join(f"{v:.4f}" for v in pos),
                      xyaxes=" ".join(f"{v:.4f}" for v in np.r_[xaxis, yaxis]))

    # 探头：挂在左腕 link 上。实测零位前臂方向在腕部局部是 +x，
    # 所以让探头沿局部 +x 伸出（几何体本身沿 z，故 body 绕 y 转 90°）。
    # 真件（SolidWorks 实测）：本体 126×126×190、探测面 Ø47、质量 1.5 kg、正装。
    probe = ET.SubElement(wrist, "body", name="radiate_probe", pos="0 0 0",
                          quat="0.70710678 0 0.70710678 0")
    half_w = cfg["probe_width"] / 2
    half_l = cfg["probe_len"] / 2
    com = cfg["probe_com"]
    # 均质长方体：Ixx=Iyy=m(w²+l²)/12，Izz=m(w²+w²)/12
    i_xy = cfg["probe_mass"] * (cfg["probe_width"] ** 2 + cfg["probe_len"] ** 2) / 12
    i_z = cfg["probe_mass"] * (cfg["probe_width"] ** 2 * 2) / 12
    ET.SubElement(probe, "inertial", pos=f"0 0 {com:.4f}",
                  mass=f"{cfg['probe_mass']:.4f}",
                  diaginertia=f"{i_xy:.6f} {i_xy:.6f} {i_z:.6f}")
    ET.SubElement(probe, "geom", name="probe_body", type="box",
                  size=f"{half_w:.4f} {half_w:.4f} {half_l:.4f}",
                  pos=f"0 0 {half_l:.4f}", rgba="0.70 0.72 0.78 1")
    ET.SubElement(probe, "geom", name="probe_face", type="cylinder",
                  size=f"{cfg['probe_face_r']:.4f} 0.004",
                  pos=f"0 0 {cfg['probe_len']:.4f}", rgba="1 0.75 0.2 1")
    ET.SubElement(probe, "site", name="tcp", pos=f"0 0 {cfg['probe_len']:.4f}", size="0.01")

    # 执行器：受控 7 关节 + 保持 7 关节（右臂），力矩范围取 URDF effort
    actuator = scene.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(scene, "actuator")
    for name in ACTUATED:
        effort = limits[name]["effort"]
        bound = f"{-effort:.1f} {effort:.1f}"
        ET.SubElement(actuator, "motor", name=name.replace("_joint", ""), joint=name,
                      ctrllimited="true", ctrlrange=bound,
                      forcelimited="true", forcerange=bound)
    for joint in scene.findall(".//joint"):
        if joint.get("name") in ACTUATED:
            joint.set("armature", str(cfg["armature"]))
            effort = limits[joint.get("name")]["effort"]
            joint.set("actuatorfrcrange", f"{-effort:.1f} {effort:.1f}")
            # 关节阻尼由 tools/tune_gains.py 按"目标阻尼比"计算后写回，
            # 这里不拍一个数，避免两处定义冲突。

    ET.SubElement(scene, "visual")
    # 骨盆网格与腰连杆本来是一体（URDF 里 pelvis 无关节），包进 mount 后相邻关系丢失，
    # 会产生 4 对自接触、以约 18 N·m 的约束力顶住腰 yaw。显式排除。
    contact = scene.find("contact")
    if contact is None:
        contact = ET.SubElement(scene, "contact")
    ET.SubElement(contact, "exclude", body1="robot_mount", body2="waist_yaw_link")
    # 探头本体 126 mm 见方，必然"包住"腕部的几个连杆；真件靠法兰和支架隔开，
    # 在这里硬碰会把可达区间误判成不可达（实测穿透 3–10 mm，截掉了 z 0.92~1.0）。
    for link in ("left_wrist_roll_link", "left_wrist_pitch_link"):
        ET.SubElement(contact, "exclude", body1=link, body2="radiate_probe")

    # 被保持的关节（腰、右臂、头）：保留可动，用等式约束锁在指定角度。
    # 这样模型里仍有这些自由度，但物理上不会动。
    equality = scene.find("equality")
    if equality is None:
        equality = ET.SubElement(scene, "equality")
    for name, angle in HOLD.items():
        # 实测约定：polycoef 的第一个系数就是平衡角本身（写 -angle 会拉到反号，右臂会突然反折）
        ET.SubElement(equality, "joint", joint1=name,
                      polycoef=f"{angle:.6f} 1 0 0 0",
                      solref="0.002 1", solimp="0.999 0.999 0.001")
    ET.indent(scene, space="  ")
    return scene


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-z", type=float, default=DEFAULTS["base_z"])
    parser.add_argument("--probe-len", type=float, default=DEFAULTS["probe_len"])
    parser.add_argument("--probe-width", type=float, default=DEFAULTS["probe_width"])
    parser.add_argument("--probe-face-r", type=float, default=DEFAULTS["probe_face_r"])
    parser.add_argument("--probe-mass", type=float, default=DEFAULTS["probe_mass"])
    parser.add_argument("--wall-x", type=float, default=DEFAULTS["wall_x"])
    parser.add_argument("--wall-half", type=float, default=1.2, help="箱面板半宽/半高")
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()
    cfg = dict(DEFAULTS, base_z=args.base_z, probe_len=args.probe_len,
               probe_width=args.probe_width, probe_face_r=args.probe_face_r,
               probe_mass=args.probe_mass,
               wall_x=args.wall_x, wall_half=args.wall_half,
               wall_z=args.base_z + 0.15)

    scene, limits = convert(cfg)
    scene = add_scene(scene, cfg, limits)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(scene).write(args.out, encoding="utf-8", xml_declaration=True)

    model = mujoco.MjModel.from_xml_path(str(args.out))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    # 自检：独立的数据对象里跑 0.5 s，确认"被保持关节"停在目标角上（符号写错会在这里暴露）
    held_check = {}
    check = mujoco.MjData(model)
    for name, angle in HOLD.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            check.qpos[model.jnt_qposadr[jid]] = angle
    mujoco.mj_forward(model, check)
    for _ in range(500):
        mujoco.mj_step(model, check)
    for name, angle in HOLD.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid >= 0:
            actual = float(check.qpos[model.jnt_qposadr[jid]])
            held_check[name] = {"target_deg": round(np.degrees(angle), 2),
                                "actual_deg": round(np.degrees(actual), 2)}
    bad = [n for n, v in held_check.items() if abs(v["actual_deg"] - v["target_deg"]) > 2.0]
    print("保持关节自检：" + ("通过" if not bad else f"**失败** -> {bad}"))
    for name in bad:
        print(f"  {name}: 目标 {held_check[name]['target_deg']}°, 实际 {held_check[name]['actual_deg']}°")
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp")
    wrist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TOOL_LINK)
    tcp = data.site_xpos[site]
    frame = data.xmat[wrist].reshape(3, 3)
    print(f"场景: {args.out}")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} 总质量={model.body_subtreemass[0]:.2f} kg")
    print(f"零位 TCP = ({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:+.3f}) m")
    print(f"腕部局部轴在世界系: x={frame[:,0].round(3)} y={frame[:,1].round(3)} z={frame[:,2].round(3)}")
    print(f"箱面 x={cfg['wall_x']:.3f} m，TCP 到箱面距离 = {cfg['wall_x']-tcp[0]:+.3f} m")

    manifest = {
        "source_urdf": URDF.name, "scene": args.out.name,
        "base_z_m": cfg["base_z"], "wall_x_m": cfg["wall_x"],
        "controlled_joints": CONTROL,
        "hold_control_joints": HOLD_CONTROL,
        "held_joints_rad": HOLD,
        "fixed": "右臂、头、相机（fixed 关节）；底盘/升降柱不建模",
        "probe": {"length_m": cfg["probe_len"], "width_m": cfg["probe_width"],
                  "face_diameter_m": 2 * cfg["probe_face_r"], "mass_kg": cfg["probe_mass"],
                  "com_from_flange_m": cfg["probe_com"], "axis_local": [1, 0, 0],
                  "source": "SolidWorks 实测：本体 126×126×190、探测面 Ø47、质量按 1.5 kg",
                  "note": "沿腕部局部 +x（零位前臂方向），正装；法兰 70×70×6 未单独建模"},
        "effort_limit_Nm": {n: limits[n]["effort"] for n in ACTUATED},
        "torque_check": {"probe_static_Nm_at_wrist":
                         round(cfg["probe_mass"] * 9.81 * cfg["probe_com"], 2)},
        "nq": model.nq, "nv": model.nv, "nu": model.nu,
        "total_mass_kg": round(float(model.body_subtreemass[0]), 3),
    }
    MANIFEST.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"清单: {MANIFEST}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
