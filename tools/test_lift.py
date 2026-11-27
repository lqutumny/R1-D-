#!/usr/bin/env python3
"""升降柱自检：限幅 / 没开 --lift 要拒绝 / 速度准确 / 停住不滑 / 顶到限位夹住。

和 `test_chassis.py`（平移）分开，因为立柱的失效方式和轮子完全不一样：

- 速度指令是**归一化**的，写错单位就是 6.5 倍速；
- 真机不报位置，走多远只能开环算——仿真里必须证明"速度×时间"真的对得上；
- 停住时不许自己滑下去（重力吃掉速度的坑我踩过一次）；
- 到行程端头必须夹住，而不是继续发速度让固件顶着。

用法：
    python3 tools/test_lift.py
    python3 tools/test_lift.py --unit     # 只跑不需要 DDS 的部分
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

# 两个用例必须共用一个域：SDK 的 ChannelFactory 是单例，一个进程里第二次
# ChannelFactoryInitialize 会被忽略（实测表现为第二个用例全部超时 3104）。
# 两个仿真器是先后启动的，共用一个域不会撞车。
DOMAIN_LIFT = 219        # 端到端：--lift
DOMAIN_NO_LIFT = DOMAIN_LIFT   # agv 服务在、但没开 --lift
COLUMN_MAX = 0.0765      # m/s，官方注释里的立柱最高速度


def case(results, name, ok, detail=""):
    results.append({"case": name, "status": "通过" if ok else "失败", "detail": detail})
    print(f"  {'通过' if ok else '失败'}  {name}" + (f" — {detail}" if detail else ""), flush=True)
    return ok


def start_sim(domain, lift, log, report):
    for f in (log, report):
        f.unlink(missing_ok=True)
    args = [sys.executable, str(ROOT / "r1d_scan" / "sim.py"), "--network", "lo",
            "--domain", str(domain), "--no-viewer", "--report", str(report)]
    args.append("--lift" if lift else "--mobile")
    proc = subprocess.Popen(args, cwd=str(ROOT), stdout=open(log, "w"),
                            stderr=subprocess.STDOUT)
    for _ in range(120):
        time.sleep(0.5)
        if log.exists() and "SIM_READY" in log.read_text(encoding="utf-8", errors="ignore"):
            return proc, True
    return proc, False


def stop_sim(proc):
    proc.terminate()
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()


def unit_tests(results):
    import chassis as chassis_mod
    print("[A] 不需要 DDS 的部分", flush=True)
    lim = chassis_mod.Limits.load()
    case(results, "配置里升降柱已启用且已实测", lim.height_enabled is True,
         f"可用 {list(lim.height_range)} m，机械 {list(lim.height_mechanical)} m")
    case(results, "可用区间在机械行程之内",
         lim.height_mechanical[0] <= lim.height_range[0] < lim.height_range[1] <= lim.height_mechanical[1],
         f"可用 {lim.height_range}，机械 {lim.height_mechanical}")
    c = chassis_mod.Chassis(limits=lim, dry_run=True, auto_watchdog=False)
    case(results, "dry_run 下 height() 直接返回 0", c.height(0.5) == 0)
    case(results, "travel() 按速度算秒数",
         abs(c.travel(0.1, up=True, vz=0.5)[1] - 0.1 / (0.5 * COLUMN_MAX)) < 1e-6,
         f"0.1 m / 38.2 mm/s = {0.1/(0.5*COLUMN_MAX):.2f} s")
    off = chassis_mod.Limits(height_enabled=False)
    c2 = chassis_mod.Chassis(limits=off, dry_run=True, auto_watchdog=False)
    case(results, "未启用时 height() 返回 -9 而不是假装成功", c2.height(0.5) == -9)


def reject_without_lift(results):
    """agv 服务在、但场景没开 --lift：1002 必须被明确拒绝（返回 2），不能假装成功。"""
    print("[B] 没开 --lift 时立柱指令必须被拒绝", flush=True)
    log = ROOT / "out" / "test_lift_nolift.log"
    report = ROOT / "out" / "test_lift_nolift.json"
    proc, ready = start_sim(DOMAIN_NO_LIFT, False, log, report)
    try:
        case(results, "仿真器就绪（移动底盘模式，无立柱）", ready)
        if not ready:
            return
        from dds_network import initialize
        import chassis as chassis_mod
        initialize("lo", DOMAIN_NO_LIFT)
        c = chassis_mod.Chassis(dry_run=False, auto_watchdog=False)
        ok, code = c.probe_height()
        case(results, "HeightAdjust 被拒绝（返回 2）", (not ok) and code == 2, f"返回码 {code}")
    finally:
        stop_sim(proc)


def end_to_end(results):
    print("[C] 端到端：速度、停住不滑、顶到限位", flush=True)
    log = ROOT / "out" / "test_lift_sim.log"
    report = ROOT / "out" / "test_lift_sim.json"
    proc, ready = start_sim(DOMAIN_LIFT, True, log, report)
    try:
        case(results, "仿真器就绪并托管 agv 服务（--lift）", ready)
        if not ready:
            return
        from dds_network import initialize
        import chassis as chassis_mod
        initialize("lo", DOMAIN_LIFT)
        c = chassis_mod.Chassis(dry_run=False)
        ok, code = c.probe_height()
        case(results, "零速探针通过", ok, f"返回码 {code}")

        usable, mechanical = gap_limits()
        lo, hi = usable
        h0 = read_height(report, log)
        case(results, "起始间隙在可用范围内", lo <= h0 <= hi,
             f"实测 {h0*1000:.0f} mm，可用 {lo*1000:.0f}–{hi*1000:.0f} mm")

        # 1) 以 0.5 归一化速度升 1 s：应走 0.5×76.5 = 38.2 mm
        c.lift(0.5, 1.0)
        time.sleep(0.7)           # HEIGHT 每 0.5 s 才打一行，读太快会读到半路的值
        h1 = read_height(report, log)
        expect = 0.5 * COLUMN_MAX
        case(results, "1 s 上行位移对（±20%）", abs((h1 - h0) - expect) <= 0.20 * expect,
             f"走了 {(h1-h0)*1000:.1f} mm，理论 {expect*1000:.1f} mm")

        # 2) 停 1 s 不许下滑
        c.lift(0.0, 1.5)
        time.sleep(0.7)
        h2 = read_height(report, log)
        case(results, "停住 1 s 不下滑（≤1 mm）", abs(h2 - h1) <= 0.001,
             f"{h1*1000:.1f} → {h2*1000:.1f} mm")

        # 3) 一直上升，必须被上限夹住（90 cm，机械上限，不会越界）
        c.lift(1.0, 3.0)
        time.sleep(0.3)
        h3 = read_height(report, log)
        case(results, "顶到上限后夹住", abs(h3 - hi) <= 0.002,
             f"实测 {h3*1000:.1f} mm，上限 {hi*1000:.1f} mm")

        # 4) 一直下降，必须停在 40 cm——再低手臂就打到底盘
        #    注意 vz 会被 max_vz（0.5）夹住，实际只有 38.2 mm/s，所以给足 16 s
        #    （0.90→0.40 只需 6.6 s，这里留余量）
        c.lift(-1.0, 16.0)
        time.sleep(0.3)
        h4 = read_height(report, log)
        case(results, "降到下限停住（不进入手臂磕碰区）", abs(h4 - lo) <= 0.002,
             f"实测 {h4*1000:.1f} mm，下限 {lo*1000:.1f} mm（机械行程可到 "
             f"{mechanical[0]*1000:.0f} mm，但会磕手臂）")
    finally:
        c.close()
        stop_sim(proc)


def gap_limits():
    """可用间隙范围（离底盘板上表面），已扣掉手臂会磕碰的那段。"""
    raw = json.loads((ROOT / "models" / "chassis.json").read_text(encoding="utf-8"))
    height = raw["height"]
    return tuple(height["usable_gap_m"]), tuple(height["mechanical_gap_m"])


def read_height(report, log):
    """读不到报告时返回 NaN，让用例判失败而不是崩。"""
    # 仿真器只在退出时写报告，所以这里改读日志里的 trace 行
    text = log.read_text(encoding="utf-8", errors="ignore") if log.exists() else ""
    for line in reversed(text.splitlines()):
        if line.startswith("HEIGHT "):
            return float(line.split()[1])
    return float("nan")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--unit", action="store_true", help="只跑不需要 DDS 的部分")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "test_lift.json")
    args = parser.parse_args()

    results = []
    t0 = time.monotonic()
    unit_tests(results)
    if not args.unit:
        reject_without_lift(results)
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