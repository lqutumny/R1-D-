#!/usr/bin/env python3
"""升降柱演示：让立柱升、停、降。

用法：

    # 终端 1：仿真器托管 agv 服务（--lift）
    cd ~/r1d_chassis
    python3 r1d_scan/sim.py --network lo --domain 215 --lift

    # 终端 2：立柱升 10 cm、停 1 秒、降 10 cm
    python3 tools/lift_demo.py --network lo --domain 215 --plan up:0.10,wait:1,down:0.10

    # 只算不发，先看每一步要多久
    python3 tools/lift_demo.py --plan up:0.10 --dry-run

动作语法（逗号分隔）：

    up:0.10     上升 0.10 m
    down:0.10   下降 0.10 m
    wait:1.0    停 1 s

三个必须知道的点位：

1. `HeightAdjust(vz)` 的 vz 是**归一化** [-1, 1]，× 76.5 mm/s 才是速度。
   `--vz 0.5` 就是 38 mm/s，不是 0.5 m/s。
2. 真机**不报立柱位置**，所以这里是开环：按"速度 × 时间"算走了多少。
   想闭环得等到有位置反馈（或者外接一个测距）。
3. 行程范围（默认 ±0.15 m）是**假设值**。现场把立柱从最低升到最高量一次，
   然后改 `models/chassis.json` 的 `height.range_m`。
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from chassis import Chassis, Limits  # noqa: E402
from dds_network import initialize  # noqa: E402

DEFAULT_PLAN = "up:0.10,wait:1.0,down:0.10"
COLUMN_MAX_M_S = 0.0765


def parse_plan(text):
    plan = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        if ":" not in item:
            raise SystemExit(f"动作格式应为 name:value，看不懂 {item!r}")
        name, value = item.split(":", 1)
        plan.append((name.strip().lower(), float(value)))
    return plan


def build_steps(plan, vz):
    """翻译成 (名字, 归一化速度, 秒数, 预期位移 m)。"""
    speed = abs(vz) * COLUMN_MAX_M_S
    steps = []
    for name, value in plan:
        if name == "up":
            steps.append((name, abs(vz), abs(value) / speed, abs(value)))
        elif name == "down":
            steps.append((name, -abs(vz), abs(value) / speed, abs(value)))
        elif name == "wait":
            steps.append((name, 0.0, abs(value), 0.0))
        else:
            raise SystemExit(f"不认识的动作 {name!r}，可用：up/down/wait")
    return steps


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--plan", default=DEFAULT_PLAN,
                        help="动作序列，如 up:0.10,wait:1,down:0.10")
    parser.add_argument("--vz", type=float, default=0.5,
                        help="归一化速度 0–1（默认 0.5 = 38 mm/s）")
    parser.add_argument("--dry-run", action="store_true", help="只算不发")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "lift_demo.json")
    args = parser.parse_args()

    limits = Limits.load()
    steps = build_steps(parse_plan(args.plan), args.vz)
    total = sum(s[2] for s in steps)
    net = sum((s[3] if s[1] > 0 else -s[3]) for s in steps)

    print(f"计划：{args.plan}")
    print(f"  速度 {abs(args.vz)*COLUMN_MAX_M_S*1000:.1f} mm/s（vz={args.vz}），"
          f"共 {len(steps)} 段、{total:.1f} s，净位移 {net*100:+.1f} cm")
    lo, hi = limits.height_range
    print(f"  可用间隙 {lo*100:.0f} … {hi*100:.0f} cm（离底盘板上表面；实测值）")
    print(f"  机械行程 {limits.height_mechanical[0]*100:.0f} … {limits.height_mechanical[1]*100:.0f} cm，"
          f"低于 {lo*100:.0f} cm 手臂会打到底盘，所以不给下去")

    # 开环提醒：真机不报位置，程序不知道现在在哪，只能保证"别一次走太远"
    span = hi - lo
    over = [f"{name}:{value:.2f} m" for name, _vz, sec, value in steps
            if name in ("up", "down") and value > span]
    if over:
        print(f"\n拒绝执行：单段位移超过可用行程 {span*100:.0f} cm（{', '.join(over)}）。"
              f"\n真机是开环控制，分几段走更安全。")
        return 2
    for name, vz, sec, meters in steps:
        print(f"    {name:6s} vz={vz:+.2f} → {sec:5.2f} s  预期 {meters*100:.1f} cm")

    if args.dry_run:
        print("\n--dry-run：没有初始化 DDS，也没有发任何指令。")
        return 0
    if not limits.height_enabled:
        print("\nheight.enabled = false，拒绝发送（避免“发了但没动”）。"
              "先把 models/chassis.json 的 height.enabled 改成 true。")
        return 2

    initialize(args.network, args.domain)
    chassis = Chassis(limits=limits, dry_run=False)
    report = {"plan": args.plan, "network": args.network, "domain": args.domain,
              "vz": args.vz, "steps": [], "limits": chassis.summary()}
    ok = True
    try:
        ok_probe, code = chassis.probe_height()
        print(f"  零速探针：{'通过' if ok_probe else '失败'}（返回码 {code}）", flush=True)
        if not ok_probe:
            print("  服务端没接 1002 —— 仿真器是不是没带 --lift 启动？", flush=True)
            return 2
        for name, vz, sec, meters in steps:
            t0 = time.monotonic()
            n = chassis.lift(vz, sec) if sec > 0 else 0
            elapsed = time.monotonic() - t0
            report["steps"].append({"name": name, "vz": vz, "seconds": round(elapsed, 2),
                                    "commands": n, "expected_m": meters,
                                    "last_code": chassis.last_code})
            print(f"  {name:6s} 完成 {elapsed:5.2f} s（{n} 条指令，末次返回码 {chassis.last_code}）",
                  flush=True)
            if chassis.last_code not in (0, None):
                ok = False
                print(f"    返回码 {chassis.last_code} ≠ 0，停止后续动作", flush=True)
                if chassis.last_code == 2:
                    print("    -> 2 = 服务端明确拒绝：同域可能还有另一个没带 --lift 的仿真器，"
                          "或 height.enabled 为 false", flush=True)
                break
    finally:
        chassis.close()
        report["summary"] = chassis.summary()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n",
                               encoding="utf-8")
        print(f"报告 {args.report}")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())