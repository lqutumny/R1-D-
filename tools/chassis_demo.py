#!/usr/bin/env python3
"""底盘最小演示：按脚本走几段直线/转向，末尾一定停。

用法：

    # 只生成场景（移动版，只需一次）
    python3 tools/build_scene.py --mobile

    # 终端 1：仿真器托管 agv 服务
    python3 r1d_scan/sim.py --network lo --mobile

    # 终端 2：让底盘走
    python3 tools/chassis_demo.py --network lo --plan forward:0.4,left:90,forward:0.4

    # 只算不发，先看看会走多久
    python3 tools/chassis_demo.py --plan forward:0.4 --dry-run

动作语法（逗号分隔，可组合）：

    forward:0.4   前进 0.4 m
    back:0.2      后退 0.2 m
    left:90       原地左转 90°
    right:90      原地右转 90°
    strafe:0.2    向左平移 0.2 m（vy>0；真机能否横移待确认）
    wait:1.0      停 1 s

为什么每条都"发 N 次 + 最后一定 stop"：
底盘服务是**速度**指令，不是位移指令。发一次就走不停，所以按 20 Hz 持续发，
到时间就发零速。仿真里的 agv 服务端也有 0.5 s 兜底（没新指令就按零速算），两层。
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

DEFAULT_PLAN = "forward:0.4,wait:1.0,back:0.2,left:90,forward:0.3"


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


def build_steps(plan, speed, omega, strafe_speed):
    """把动作翻译成 (名字, vx, vy, vyaw, 秒数, 预期位移) 的列表。"""
    steps = []
    for name, value in plan:
        if name == "forward":
            steps.append((name, speed, 0.0, 0.0, abs(value) / speed, abs(value)))
        elif name == "back":
            steps.append((name, -speed, 0.0, 0.0, abs(value) / speed, abs(value)))
        elif name == "left":
            sec = abs(value) * 3.141592653589793 / 180.0 / omega
            steps.append((name, 0.0, 0.0, omega, sec, abs(value)))
        elif name == "right":
            sec = abs(value) * 3.141592653589793 / 180.0 / omega
            steps.append((name, 0.0, 0.0, -omega, sec, abs(value)))
        elif name == "strafe":
            steps.append((name, 0.0, strafe_speed, 0.0, abs(value) / strafe_speed, abs(value)))
        elif name == "wait":
            steps.append((name, 0.0, 0.0, 0.0, abs(value), 0.0))
        else:
            raise SystemExit(f"不认识的动作 {name!r}，可用：forward/back/left/right/strafe/wait")
    return steps


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--plan", default=DEFAULT_PLAN,
                        help="动作序列，如 forward:0.4,left:90,forward:0.4")
    parser.add_argument("--speed", type=float, default=0.15, help="直线速度 m/s")
    parser.add_argument("--omega", type=float, default=0.30, help="转向角速度 rad/s")
    parser.add_argument("--strafe-speed", type=float, default=0.10)
    parser.add_argument("--dry-run", action="store_true", help="只算不发")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "chassis_demo.json")
    args = parser.parse_args()
    plan_text = args.plan

    limits = Limits.load()
    steps = build_steps(parse_plan(plan_text), args.speed, args.omega, args.strafe_speed)

    total = sum(s[4] for s in steps)
    print(f"计划：{plan_text}")
    print(f"  共 {len(steps)} 段，预计 {total:.1f} s；限幅 vx≤{limits.max_vx} m/s, "
          f"w≤{limits.max_w} rad/s")
    for name, vx, vy, vyaw, sec, expected in steps:
        unit = "°" if name in ("left", "right") else "m"
        print(f"    {name:8s} vx={vx:+.2f} vy={vy:+.2f} w={vyaw:+.2f} → {sec:5.2f} s  "
              f"预期 {expected:.2f} {unit}")

    if args.dry_run:
        print("\n--dry-run：没有初始化 DDS，也没有发任何指令。")
        return 0

    initialize(args.network, args.domain)
    chassis = Chassis(limits=limits, dry_run=False)
    report = {"plan": plan_text, "network": args.network, "domain": args.domain,
              "steps": [], "limits": chassis.summary()["limits"]}
    ok = True
    try:
        for name, vx, vy, vyaw, sec, expected in steps:
            t0 = time.monotonic()
            n = chassis.drive(vx, vy, vyaw, sec, hz=20.0)
            elapsed = time.monotonic() - t0
            report["steps"].append({"name": name, "vx": vx, "vy": vy, "vyaw": vyaw,
                                    "seconds": round(elapsed, 2), "commands": n,
                                    "expected": expected, "last_code": chassis.last_code})
            print(f"  {name:8s} 完成 {elapsed:5.2f} s（{n} 条指令，末次返回码 {chassis.last_code}）",
                  flush=True)
            if chassis.last_code not in (0, None):
                ok = False
                print(f"    返回码 {chassis.last_code} ≠ 0：服务端没接受，停止后续动作", flush=True)
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