#!/usr/bin/env python3
"""没有 CAD 也能从 STEP(AP214) 里抠出几何量：外廓尺寸、主要圆柱半径、轴向。

用途：探头 STEP 到手后，先量出"长多少、粗多少、轴线朝哪"，才能定
`robot_profile.json` 里的 `tool.offset_m`（TCP 沿腕部轴线的伸出量）。
不追求精确体积——那个需要质量属性，还是找结构要。

用法：
  python3 tools/measure_step.py /path/to/probe.STEP
"""
import re
import sys
from collections import Counter

import numpy as np

POINT = re.compile(
    r"CARTESIAN_POINT\s*\(\s*'[^']*'\s*,\s*\(\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*\)")
CYLINDER = re.compile(r"CYLINDRICAL_SURFACE\s*\(\s*'[^']*'\s*,\s*#\d+\s*,\s*([-\d.Ee+]+)\s*\)")
CONE = re.compile(r"CONICAL_SURFACE\s*\(\s*'[^']*'\s*,\s*#\d+\s*,\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*\)")
DIRECTION = re.compile(
    r"DIRECTION\s*\(\s*'[^']*'\s*,\s*\(\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*\)")
ENTITY = re.compile(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*?)\)\s*;", re.S)
POINT_XYZ = re.compile(r"\(\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*,\s*([-\d.Ee+]+)\s*\)")


def parse_entities(text):
    return {int(m.group(1)): (m.group(2), m.group(3)) for m in ENTITY.finditer(text)}


def refs(fragment):
    return [int(x) for x in re.findall(r"#(\d+)", fragment)]


def solid_parts(ents):
    """按 MANIFOLD_SOLID_BREP 拆分，每个实体给出 bbox（只用顶点，曲面径向略偏小）。"""
    out = []
    for sid, (kind, args) in ents.items():
        if kind != "MANIFOLD_SOLID_BREP":
            continue
        shells = refs(args)
        if not shells or ents.get(shells[0], ("", ""))[0] != "CLOSED_SHELL":
            continue
        faces = refs(ents[shells[0]][1])
        pts = set()
        for face in faces:
            if ents.get(face, ("", ""))[0] not in ("ADVANCED_FACE", "FACE_SURFACE"):
                continue
            for bound in refs(ents[face][1]):
                if ents.get(bound, ("", ""))[0] not in ("FACE_OUTER_BOUND", "FACE_BOUND"):
                    continue
                for loop in refs(ents[bound][1]):
                    for oriented in refs(ents.get(loop, ("", ""))[1]):
                        if ents.get(oriented, ("", ""))[0] != "ORIENTED_EDGE":
                            continue
                        for curve in refs(ents[oriented][1]):
                            if ents.get(curve, ("", ""))[0] != "EDGE_CURVE":
                                continue
                            for vertex in refs(ents[curve][1])[:2]:
                                if ents.get(vertex, ("", ""))[0] == "VERTEX_POINT":
                                    for cp in refs(ents[vertex][1]):
                                        if cp in ents and ents[cp][0] == "CARTESIAN_POINT":
                                            m = POINT_XYZ.search(ents[cp][1])
                                            if m:
                                                pts.add(tuple(float(v) for v in m.groups()))
        if len(pts) < 4:
            continue
        P = np.array(sorted(pts))
        lo, hi = P.min(axis=0), P.max(axis=0)
        out.append({"id": sid, "faces": len(faces), "points": len(pts),
                    "lo": lo, "hi": hi, "size": hi - lo})
    return sorted(out, key=lambda d: -float(np.prod(np.maximum(d["size"], 0.1))))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    if len(args) != 1:
        raise SystemExit(__doc__)
    text = open(args[0], encoding="utf-8", errors="replace").read()
    if "ISO-10303-21" not in text[:200]:
        raise SystemExit("不是 STEP(ISO-10303-21) 文件")

    points = np.array(POINT.findall(text), float)
    if not len(points):
        raise SystemExit("没找到 CARTESIAN_POINT")
    lo, hi = points.min(axis=0), points.max(axis=0)
    size = hi - lo
    print(f"坐标点 {len(points)} 个（单位按 mm 解释，SolidWorks 默认）")
    print(f"外廓 min = {np.round(lo, 1)}   max = {np.round(hi, 1)}")
    print(f"外廓尺寸 x,y,z = {np.round(size, 1)} mm")
    print(f"外廓中心 = {np.round((hi + lo) / 2, 1)}")
    axis = int(np.argmax(size))
    print(f"最长方向 = {'xyz'[axis]}（{size[axis]:.1f} mm），"
          f"另两向 {np.round(np.delete(size, axis), 1)} mm")

    radii = np.round(np.array(CYLINDER.findall(text), float), 2)
    if len(radii):
        print(f"\n圆柱面 {len(radii)} 个，半径（出现次数多的就是主体）:")
        for radius, count in Counter(radii).most_common(8):
            print(f"   r = {radius:8.2f} mm  (Ø{2 * radius:7.2f})  ×{count}")

    cones = CONE.findall(text)
    if cones:
        print(f"\n圆锥面 {len(cones)} 个（探头前端多半是倒角/锥面）:")
        for radius, angle in cones[:6]:
            print(f"   r = {float(radius):7.2f} mm  半角 {np.degrees(float(angle)):5.1f}°")

    dirs = np.round(np.array(DIRECTION.findall(text), float), 3)
    if len(dirs):
        print("\n主方向（出现次数）:")
        for vec, count in Counter(map(tuple, dirs)).most_common(6):
            print(f"   {vec}  ×{count}")

    print("\n面数：ADVANCED_FACE =", text.count("ADVANCED_FACE"),
          " 实体 MANIFOLD_SOLID_BREP =", text.count("MANIFOLD_SOLID_BREP"))

    if "--parts" in flags:
        print("\n按实体拆开（几何量，用来认“哪个是探头本体、哪个是支架/手柄”）:")
        for part in solid_parts(parse_entities(text))[:12]:
            print(f"  #{part['id']:<5} 面 {part['faces']:3d}  "
                  f"尺寸 {np.round(part['size'], 1)}  "
                  f"min {np.round(part['lo'], 1)}  max {np.round(part['hi'], 1)}")


if __name__ == "__main__":
    main()
