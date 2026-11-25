#!/usr/bin/env python3
"""离线分析 inspect_device.py 的采集结果：生成关节对照表草稿。

不需要机器人、不需要 unitree_sdk2py；任何机器都能跑。
"""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from lowstate import summary  # noqa: E402
from profile import load as load_profile  # noqa: E402


def urdf_limits(path):
    """从 URDF 读关节限位与 effort；文件名按 _joint 结尾筛选。"""
    import xml.etree.ElementTree as ET
    table = {}
    for joint in ET.parse(path).getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
        table[joint.get("name")] = {
            "type": joint.get("type"),
            "lower": float(limit.get("lower", "nan")),
            "upper": float(limit.get("upper", "nan")),
            "effort": float(limit.get("effort", "nan")),
        }
    return table


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path, help="inspect_device.py 生成的 device.json")
    parser.add_argument("--profile", type=Path, help="robot_profile.json（可选，用于名称匹配）")
    parser.add_argument("--urdf", type=Path, help="R1-D URDF（可选，用于填限位列）")
    parser.add_argument("--report-out", type=Path, default=None)
    parser.add_argument("--markdown", type=Path, default=ROOT / "out" / "joint_map_draft.md")
    args = parser.parse_args()

    data = json.loads(args.report.read_text(encoding="utf-8"))
    rows = summary(data["slots"])
    motor = [r for r in rows if r["signal"] == "motor"]
    power = [r for r in rows if r["signal"] == "power"]
    silent = [r for r in rows if r["signal"] == "silent"]

    print(f"采集：{data.get('captured_at', '?')}  网卡 {data.get('network', '?')}  "
          f"有效包 {data.get('valid_packets', '?')}  mode_machine {data.get('mode_machine', '?')}")
    print(f"槽位总数 {len(rows)}：电机信号 {len(motor)}，仅供电 {len(power)}，全零 {len(silent)}")
    print(f"  电机信号槽位：{[r['slot'] for r in motor]}")
    print(f"  仅供电槽位：{[r['slot'] for r in power]}")

    slot_names, slot_roles = {}, {}
    if args.profile:
        profile = load_profile(args.profile)
        slot_names = {slot: name for name, slot in profile.slot_map().items()}
        slot_roles = {slot: "左臂（本程序控制）" for slot in profile.slot_map().values()}
        for joint in getattr(profile, "hold_joints", ()):
            slot_names[joint.slot] = joint.name
            slot_roles[joint.slot] = "右臂（本程序写固定值保持）"
        if not slot_names:
            print("profile 里还没有已确认的槽位映射，本次只输出待填草稿。")

    limits = urdf_limits(args.urdf) if args.urdf else {}

    print(f"\n{'槽位':>4} {'关节名(profile)':<28} {'信号':>6} {'mode':>4} {'q(deg)':>9} "
          f"{'dq':>8} {'tau':>7} {'temp':>5} {'err':>4} {'URDF限位(deg)':>20}")
    for row in rows:
        if row["signal"] == "silent":
            continue
        name = slot_names.get(row["slot"], "")
        limit = limits.get(name)
        window = (f"[{limit['lower']*57.2958:+.1f}, {limit['upper']*57.2958:+.1f}]"
                  if limit else "")
        print(f"{row['slot']:>4} {name:<28} {row['signal']:>6} {row['mode']:>4} "
              f"{row['q_deg']:>9.3f} {row['dq']:>8.3f} {row['tau_est']:>7.3f} "
              f"{row['temp_C']:>5.1f} {row['error']:>4} {window:>20}")

    flagged = [r for r in rows if r["signal"] != "silent" and (r["error"] != 0 or r["temp_C"] >= 70
                                                              or abs(r["q_deg"]) > 150)]
    if flagged:
        print("\n需要现场注意的槽位：")
        for row in flagged:
            print(f"  槽位 {row['slot']}: q={row['q_deg']:+.1f}°, err={row['error']}, "
                  f"temp={row['temp_C']:.1f}°C")
    print(f"\n全零槽位：{[r['slot'] for r in silent]}")
    print("提示：全零不等于电机不存在（未供电、未使能、或不属于本机型配置）。")

    # 六列对照表草稿：只有槽位和实测角是机器给的，其余三列必须人工填。
    lines = ["# 关节对照表（草稿，待现场逐项确认）", "",
             f"数据来源：`{args.report}`；采集时间 {data.get('captured_at', '?')}；"
             f"mode_machine {data.get('mode_machine', '?')}", "",
             "| 关节名（人工填） | DDS 槽位 | 实测正方向（人工填） | URDF 限位(deg) | "
             "控制范围（我们敢用） | 归属（腰/左臂/右臂/其他） |",
             "|---|---|---|---|---|---|"]
    for row in motor:
        name = slot_names.get(row["slot"], "")
        limit = limits.get(name)
        window = (f"{limit['lower']*57.2958:+.1f} ~ {limit['upper']*57.2958:+.1f}"
                  if limit else "（待填）")
        lines.append(f"| {name or '（待填）'} | {row['slot']} | （待填） | {window} | （待填） | "
                     f"{slot_roles.get(row['slot'], '（待填）')} |")
    lines += ["", "当前姿态实测（关节坐标系原始值）：", "",
              "| 槽位 | q(deg) | dq | tau(N·m) | temp(°C) | error |",
              "|---|---|---|---|---|---|"]
    for row in motor:
        lines.append(f"| {row['slot']} | {row['q_deg']:+.2f} | {row['dq']:+.3f} | "
                     f"{row['tau_est']:+.3f} | {row['temp_C']:.1f} | {row['error']} |")
    if power:
        lines += ["", f"另有 {len(power)} 个槽位只有供电/温度、没有控制信号："
                      f"{[r['slot'] for r in power]}（不列入对照表，需现场确认是否本机型配置）。"]
    args.markdown.parent.mkdir(parents=True, exist_ok=True)
    args.markdown.write_text("\n".join(lines) + "\n", encoding="utf-8")

    result = {"source": str(args.report), "slot_count": len(rows),
              "motor_slots": [r["slot"] for r in motor],
              "power_only_slots": [r["slot"] for r in power],
              "silent_slots": [r["slot"] for r in silent],
              "flagged": [{"slot": r["slot"], "q_deg": round(r["q_deg"], 3),
                           "error": r["error"], "temp_C": r["temp_C"]} for r in flagged],
              "rows": rows, "markdown": str(args.markdown)}
    out = args.report_out or args.report.with_name("joint_map_draft.json")
    out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"\n草稿表：{args.markdown}\n结构化：{out}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as exc:
        print(f"分析失败: {exc}", file=sys.stderr)
        raise SystemExit(2)
