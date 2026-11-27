#!/usr/bin/env python3
"""底盘通道自检：限幅 / 拒绝 / 服务缺失 / 端到端位移。

为什么这几项要单独测：底盘走的是**速度**服务，不是位移指令，出错的方式和手臂完全不同——

- 限幅写错 → 真机上第一拍就是全速；
- 服务端不在却"调用成功" → 你以为在遥控，其实没人接；
- 兜底超时不生效 → 程序崩了底盘还在走，这是最危险的失效模式。

用法：
    python3 tools/test_chassis.py            # 全部
    python3 tools/test_chassis.py --unit     # 只跑不需要 DDS 的部分
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

DOMAIN_SIM = 217      # 端到端：仿真器 + 演示脚本
DOMAIN_EMPTY = 218    # 服务缺失：这条域上没有 agv 服务端


def case(results, name, ok, detail=""):
    results.append({"case": name, "status": "通过" if ok else "失败", "detail": detail})
    print(f"  {'通过' if ok else '失败'}  {name}" + (f" — {detail}" if detail else ""), flush=True)
    return ok


def unit_tests(results):
    import chassis as chassis_mod

    print("[A] 不需要 DDS 的部分", flush=True)
    lim = chassis_mod.Limits(max_vx=0.2, max_vy=0.1, max_w=0.4, watchdog=0.5)
    c = chassis_mod.Chassis(limits=lim, dry_run=True, auto_watchdog=False)
    got = c.clamp(10.0, -10.0, 5.0)
    case(results, "超限请求被夹住", got == (0.2, -0.1, 0.4), f"10/-10/5 → {got}")
    got2 = c.clamp(0.1, 0.02, -0.1)
    case(results, "未超限原样通过", got2 == (0.1, 0.02, -0.1), f"{got2}")

    lim_off = chassis_mod.Limits(height_enabled=False)
    c2 = chassis_mod.Chassis(limits=lim_off, dry_run=True, auto_watchdog=False)
    case(results, "升降柱未启用时明确拒绝", c2.height(0.5) == -9, "返回 -9，不是假装成功")

    # 加载真实配置文件，确认字段名没写错（写错会在真机现场才炸）
    loaded = chassis_mod.Limits.load()
    case(results, "models/chassis.json 可解析",
         loaded.max_vx > 0 and loaded.max_w > 0 and loaded.watchdog > 0,
         f"vx≤{loaded.max_vx} m/s, w≤{loaded.max_w} rad/s, 看门狗 {loaded.watchdog} s")


def missing_service_test(results):
    """没有服务端时，探针要返回失败而不是抛异常或假装成功。"""
    print("[B] 服务缺失（这条域上没有 agv 服务端）", flush=True)
    from dds_network import initialize
    import chassis as chassis_mod

    initialize("lo", DOMAIN_EMPTY)
    t0 = time.monotonic()
    c = chassis_mod.Chassis(dry_run=False, auto_watchdog=False)
    ok, code = c.probe()
    dt = time.monotonic() - t0
    case(results, "无服务端时探针返回失败", not ok, f"返回码 {code}，耗时 {dt:.1f} s")
    case(results, "无服务端时不抛异常", True, "程序还能继续跑（可以打印提示并退出）")


def end_to_end(results):
    print("[C] 端到端：仿真器托管 agv 服务 + 演示脚本走一圈", flush=True)
    log = ROOT / "out" / "test_chassis_sim.log"
    sim_report = ROOT / "out" / "test_chassis_sim.json"
    log.parent.mkdir(parents=True, exist_ok=True)
    for f in (log, sim_report):
        f.unlink(missing_ok=True)

    sim = subprocess.Popen(
        [sys.executable, str(ROOT / "r1d_scan" / "sim.py"), "--network", "lo",
         "--domain", str(DOMAIN_SIM), "--mobile", "--no-viewer",
         "--report", str(sim_report)],
        cwd=str(ROOT), stdout=open(log, "w"), stderr=subprocess.STDOUT)
    try:
        ready = False
        for _ in range(120):
            time.sleep(0.5)
            if log.exists() and "SIM_READY" in log.read_text(encoding="utf-8", errors="ignore"):
                ready = True
                break
        case(results, "仿真器就绪并托管 agv 服务", ready)
        if not ready:
            return

        plan = "forward:0.4,back:0.2,left:90,forward:0.3"
        demo = subprocess.run(
            [sys.executable, str(ROOT / "tools" / "chassis_demo.py"), "--network", "lo",
             "--domain", str(DOMAIN_SIM), "--plan", plan,
             "--report", str(ROOT / "out" / "test_chassis_demo.json")],
            cwd=str(ROOT), capture_output=True, text=True, timeout=120)
        out = demo.stdout + demo.stderr
        case(results, "演示脚本退出码 0", demo.returncode == 0, f"returncode={demo.returncode}")
        case(results, "每段都被服务端接受（返回码 0）",
             demo.returncode == 0 and "末次返回码 0" in out)
    finally:
        sim.terminate()
        try:
            sim.wait(timeout=15)
        except subprocess.TimeoutExpired:
            sim.kill()

    if not sim_report.exists():
        case(results, "仿真器写了报告", False)
        return
    data = json.loads(sim_report.read_text(encoding="utf-8"))
    case(results, "报告记录了底盘模式", data.get("base_mode") == "mobile",
         str(data.get("base_mode")))
    x, y, yaw = data.get("base_pose", [0, 0, 0])
    # 规划：前进 0.4 → 后退 0.2 → 左转 90° → 前进 0.3，终点应在 (0.2, 0.3)、偏航 +90°
    err_pos = math.hypot(x - 0.2, y - 0.3)
    err_yaw = abs(math.degrees(yaw) - 90.0)
    case(results, "终点位置对（±30 mm）", err_pos <= 0.030,
         f"实测 ({x:.3f}, {y:.3f})，目标 (0.200, 0.300)，误差 {err_pos*1000:.1f} mm")
    case(results, "偏航对（±2°）", err_yaw <= 2.0, f"实测 {math.degrees(yaw):.1f}°，误差 {err_yaw:.2f}°")
    case(results, "累计行程对（±5%）", abs(data.get("base_travel_m", 0) - 0.9) <= 0.045,
         f"实测 {data.get('base_travel_m')} m，指令 0.9 m")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--unit", action="store_true", help="只跑不需要 DDS 的部分")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "test_chassis.json")
    args = parser.parse_args()

    results = []
    t0 = time.monotonic()
    unit_tests(results)
    if not args.unit:
        missing_service_test(results)
        end_to_end(results)

    passed = sum(1 for r in results if r["status"] == "通过")
    total = len(results)
    payload = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"), "results": results,
               "passed": passed, "total": total, "seconds": round(time.monotonic() - t0, 1)}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                           encoding="utf-8")
    print(f"\n合计 {passed}/{total} 通过，用时 {payload['seconds']} s，报告 {args.report}")
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())