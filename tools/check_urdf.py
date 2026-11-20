#!/usr/bin/env python3
"""检查 R1-D 的 URDF：关节表、限位、effort、以及零位时的臂长。

纯 Python（xml.etree），不需要机器人、不需要 numpy。
臂长只做"零位姿态"的近似：沿运动链把各段 origin 平移累加。
"""
import argparse
import json
import math
from pathlib import Path


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def vec_add(a, b):
    return [a[i] + b[i] for i in range(3)]


def vec_norm(v):
    return math.sqrt(sum(x * x for x in v))


def read_joint(joint):
    def floats(text, count=3, default=0.0):
        values = [float(x) for x in (text or "").split()]
        return (values + [default] * count)[:count]

    origin = joint.find("origin")
    axis = joint.find("axis")
    limit = joint.find("limit")
    return {
        "name": joint.get("name"),
        "type": joint.get("type"),
        "parent": joint.find("parent").get("link"),
        "child": joint.find("child").get("link"),
        "xyz": floats(origin.get("xyz") if origin is not None else ""),
        "rpy": floats(origin.get("rpy") if origin is not None else ""),
        "axis": floats(axis.get("xyz") if axis is not None else "1 0 0"),
        "lower": float(limit.get("lower")) if limit is not None and limit.get("lower") else None,
        "upper": float(limit.get("upper")) if limit is not None and limit.get("upper") else None,
        "effort": float(limit.get("effort")) if limit is not None and limit.get("effort") else None,
        "velocity": float(limit.get("velocity")) if limit is not None and limit.get("velocity") else None,
    }


def chain(joints, root_link, tip_link):
    by_child = {j["child"]: j for j in joints}
    path, link = [], tip_link
    while link != root_link:
        joint = by_child.get(link)
        if joint is None:
            raise ValueError(f"无法从 {tip_link} 回溯到 {root_link}；检查连杆名")
        path.append(joint)
        link = joint["parent"]
    return path[::-1]


def chain_geometry(joints, root_link, tip_link):
    """零位姿态下的末端位置与各段平移长度之和。"""
    position, rotation = [0.0, 0.0, 0.0], [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]
    length_sum = 0.0
    for joint in chain(joints, root_link, tip_link):
        position = vec_add(position, mat_vec(rotation, joint["xyz"]))
        rotation = mat_mul(rotation, rpy_matrix(*joint["rpy"]))
        length_sum += vec_norm(joint["xyz"])
    return position, length_sum


def summarize(path, root_link, tip_link):
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    robot_name = root.get("name")
    joints = [read_joint(j) for j in root.findall("joint")]
    links = {link.get("name") for link in root.findall("link")}

    movable = [j for j in joints if j["type"] != "fixed"]
    arm_keys = ("shoulder", "elbow", "wrist")
    left_arm = [j for j in movable
                if j["name"].startswith("left_") and any(k in j["name"] for k in arm_keys)]
    right_arm = [j for j in movable
                 if j["name"].startswith("right_") and any(k in j["name"] for k in arm_keys)]
    waist = [j for j in movable if "waist" in (j["name"] or "").lower()
             or j["name"] in ("torso_Joint", "Yaw_Joint")]

    position, length_sum = chain_geometry(joints, root_link, tip_link)

    problems = []
    for joint in movable:
        if joint["lower"] is None or joint["upper"] is None:
            problems.append(f"{joint['name']}: 缺 lower/upper")
        elif joint["lower"] == joint["upper"]:
            problems.append(f"{joint['name']}: 上下限相等")
        if joint["effort"] is None or joint["effort"] <= 0:
            problems.append(f"{joint['name']}: 缺 effort")
        if "joint" not in (joint["name"] or "").lower():
            problems.append(f"{joint['name']}: 名称里没有 joint，检查是否解析错")
    if len(left_arm) != 7:
        problems.append(f"左臂活动关节数为 {len(left_arm)}，A7 应为 7")
    for link in (root_link, tip_link):
        if link not in links:
            problems.append(f"缺少连杆 {link}")

    return {
        "urdf": str(path), "robot": robot_name,
        "counts": {"links": len(links), "joints": len(joints), "movable": len(movable),
                   "left_arm": len(left_arm), "right_arm": len(right_arm), "waist": len(waist)},
        "chain": {"root": root_link, "tip": tip_link,
                  "reach_m": round(vec_norm(position), 4),
                  "segment_sum_m": round(length_sum, 4),
                  "tip_position_m": [round(v, 4) for v in position],
                  "note": "零位姿态近似；reach 不是厂商标注的臂长，仅用于数量级核对"},
        "joints": [{k: (round(v, 6) if isinstance(v, float) else v)
                    for k, v in joint.items()} for joint in joints],
        "movable_joints": [j["name"] for j in movable],
        "problems": problems,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("--from", dest="root_link", default="left_shoulder_pitch_link")
    parser.add_argument("--to", dest="tip_link", default="left_wrist_yaw_link")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    result = summarize(args.urdf, args.root_link, args.tip_link)

    counts = result["counts"]
    print(f"URDF: {result['urdf']}  机器人名: {result['robot']}")
    print(f"连杆 {counts['links']}，关节 {counts['joints']}（活动 {counts['movable']}："
          f"左臂 {counts['left_arm']}，右臂 {counts['right_arm']}，腰 {counts['waist']}）")
    geometry = result["chain"]
    print(f"零位臂段：{geometry['root']} → {geometry['tip']}，"
          f"直线距离 {geometry['reach_m']*1000:.1f} mm，各段平移之和 {geometry['segment_sum_m']*1000:.1f} mm")
    print(f"\n{'关节':<30} {'type':<9} {'axis':<12} {'lower(deg)':>11} {'upper(deg)':>11} "
          f"{'effort(N·m)':>11}")
    for joint in result["joints"]:
        if joint["type"] == "fixed":
            continue
        lower = "—" if joint["lower"] is None else f"{math.degrees(joint['lower']):+.1f}"
        upper = "—" if joint["upper"] is None else f"{math.degrees(joint['upper']):+.1f}"
        effort = "—" if joint["effort"] is None else f"{joint['effort']:.1f}"
        print(f"{joint['name']:<30} {joint['type']:<9} "
              f"{' '.join(f'{v:g}' for v in joint['axis']):<12} {lower:>11} {upper:>11} {effort:>11}")

    print("\n问题：" + ("无" if not result["problems"] else ""))
    for problem in result["problems"]:
        print(f"  - {problem}")
    print("\n提示：A7 的厂商标注臂长为 555 mm（大臂+小臂），与这里的零位距离不同属正常，"
          "重点看数量级和左右对称性。")

    report = args.report or args.urdf.with_name("urdf_check.json")
    report.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"已保存 {report}")
    return 0 if not result["problems"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(f"URDF 检查失败: {exc}")
