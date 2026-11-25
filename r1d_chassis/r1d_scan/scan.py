#!/usr/bin/env python3
"""阶段 4：接 DDS 跑一轮扫描。仿真用 --network lo，真机只换网卡名。

流程：等 LowState → 启动保持 2 s → 接近扫描起点 → 上行 → 下行 → 回初始姿态 → 保持 2 s → 退出

退出码：0 = 走完整轮；2 = 保护触发，已停止发送（机器人保持最后一条指令）；1 = 其它错误。
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))

from controller import Controller, Limits, load_motion  # noqa: E402
from dds_network import initialize  # noqa: E402
from profile import load as load_profile  # noqa: E402
from runner import Runner  # noqa: E402
from transport import Codec, ControlFault  # noqa: E402

FAULT_EXIT = 2


def joint_array(profile_path, field):
    import json as _json
    raw = _json.loads(Path(profile_path).read_text(encoding="utf-8"))
    return np.array([joint[field] for joint in raw["joints"]], float)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "scan_run.json")
    parser.add_argument("--log", type=Path, default=ROOT / "out" / "scan_run.csv")
    parser.add_argument("--hold", type=float, default=2.0, help="结束后保持秒数")
    parser.add_argument("--passes", type=int, default=1, help="往返趟数（默认 1 趟 = 上下各一次）")
    parser.add_argument("--timeout", type=float, default=10.0, help="等首帧 LowState 的秒数")
    parser.add_argument("--dry-run", action="store_true",
                        help="只规划、不发任何指令：用实测姿态把整轮时间轴离线检查一遍"
                             "（限位/角速度/直线度/轴线/离箱面距离），报告 published_commands=0")
    args = parser.parse_args()

    profile = load_profile()
    # 真机准入锁：非回环网卡必须使用已核对的 profile，否则一条指令都不发。
    # 仿真（lo）不受此限，因为它本来就跑在暂定槽位上。
    if args.network != "lo":
        gaps = profile.gaps()
        if gaps:
            # 预检一条指令都不发，所以闸门只当提醒；真正的闸门在写指令之前（下面这个 raise）
            if args.dry_run:
                print("提示：profile 仍有未核对项，预检继续（本次不会发任何指令）：\n  - "
                      + "\n  - ".join(gaps), flush=True)
            else:
                raise SystemExit("真机准入检查未通过，拒绝启动（一条指令都不会发）：\n  - "
                                 + "\n  - ".join(gaps)
                                 + "\n请先用 tools/inspect_device.py 只读核对，"
                                   "再把 robot_profile.json 的 slot_source 改成 verified。")
    raw_path = HERE / "robot_profile.json"
    raw = json.loads(raw_path.read_text(encoding="utf-8"))
    lo = joint_array(raw_path, "lower")
    hi = joint_array(raw_path, "upper")
    effort = joint_array(raw_path, "effort")
    codec = Codec(profile=profile)
    scan_data, motion = load_motion(passes=max(1, args.passes))
    # 预检要用同一份几何定义，避免文档里的数字和代码里的对不上
    profile_geom = {key: scan_data[key]
                    for key in ("tcp_x_m", "y_m", "z_bottom_m", "z_top_m", "standoff_m")}

    initialize(args.network, args.domain)
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

    cache = {"state": None, "good": 0, "bad": 0, "error": ""}

    def on_state(msg):
        try:
            cache["state"] = codec.decode(msg, time.monotonic(), cache["state"])
            cache["good"] += 1
        except ControlFault as exc:
            # 坏包只记账、不采用：状态停在上一帧有效值，由 0.2 s 超时保护兜底
            cache["bad"] += 1
            cache["error"] = str(exc)

    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(on_state, 1)
    deadline = time.monotonic() + args.timeout
    while cache["good"] < 20 and time.monotonic() < deadline:
        time.sleep(0.01)
    if cache["good"] < 20:
        raise SystemExit(f"没有有效 LowState（good={cache['good']}, bad={cache['bad']}, "
                         f"{cache['error']}）；仿真器是否已启动？")
    captured = cache["state"].q.copy()
    print(f"捕获姿态: {np.round(np.degrees(captured), 1).tolist()} deg", flush=True)
    # 保持关节（右臂）：照启动时的实测角写回，等于"原地不动"，不会把手臂拽向理论值
    hold_captured = None
    if codec.hold_slots.size:
        hold_captured = np.asarray(cache["state"].hold_q, float).copy()
        # 槽位配错时，这里会读到别的电机（真机上见过某槽位报 86 rad）。
        # 照原样写回去 = 让那个关节全速冲向一个荒谬角度，所以在发第一个包之前先验一遍。
        insane = [f"槽位{s}={np.degrees(v):.0f}°" for s, v in zip(codec.hold_slots, hold_captured)
                  if not -np.pi <= v <= np.pi]
        if insane:
            raise SystemExit("保持关节的实测角不合理，槽位可能配错，拒绝启动：\n  "
                             + "、".join(insane)
                             + "\n请先用 tools/inspect_device.py 只读核对槽位。")
        codec.hold_targets = hold_captured
        names = ", ".join(f"{n}={np.degrees(v):.1f}°" for n, v in zip(codec.hold_names, hold_captured))
        print(f"保持关节（写固定值）: {names}", flush=True)
    # 自检：捕获姿态应当是"双臂下垂"，否则多半连到了旧的仿真器进程
    expected = {name: angle for name, angle in raw["initial"].items() if name != "note"}
    mismatch = [f"{name}={np.degrees(captured[profile.names().index(name)]):.1f}°"
                for name, angle in expected.items()
                if abs(captured[profile.names().index(name)] - angle) > np.radians(5)]
    if mismatch:
        print(f"提示：起始姿态与 profile 里的典型下垂姿态不同（{', '.join(mismatch)}）。"
              "程序以实测姿态为起点，不影响执行；如果这不是你想要的位置，先手动摆好再启动。", flush=True)

    # publisher 必须在 Controller 之前 Init：它在 DDS 上建通道约 200 ms，
    # 而时间基准是在 Controller 构造之后才对齐的，这段延迟才不会算进第一拍。
    # （把这一行挪到对齐之后，第一拍就会测出 ~210 ms 停顿并被"控制周期"保护拦下。）
    publisher = None
    if not args.dry_run:
        publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        publisher.Init()

    controller = Controller(captured, motion, lo, hi, codec.kp, codec.kd, effort,
                            limits=Limits(startup_hold=2.0), start=time.monotonic(),
                            expected_mode_machine=cache["state"].mode_machine)
    # __init__ 里加载 MuJoCo 模型耗时 1–2 s，必须在构造完之后重新对齐时间基准
    controller.previous = time.monotonic()
    controller.start = controller.previous + controller.limits.startup_hold

    if args.dry_run:
        # 预检：一切照真跑的时间轴算，但一条指令都不发。时间基准按相对时间给，
        # 否则 target(now) 会因为 absolute start 而永远停在 startup-hold。
        from kinematics import ArmKinematics
        import preflight
        controller.start = controller.limits.startup_hold
        result = preflight.run(controller, motion, profile_geom, controller.limits,
                               ArmKinematics())
        payload = result.report()
        payload.update({"network": args.network, "domain": args.domain,
                        "captured_q_deg": np.degrees(captured).round(2).tolist()})
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                               encoding="utf-8")
        print(f"\n预检：{result.seconds:.1f} s 时间轴，发出指令 {result.published_commands} 条", flush=True)
        print(f"  关节限位最小余量 {result.min_limit_margin_deg:.1f}°（{profile.names()[result.binding_joint]}）",
              flush=True)
        print(f"  目标角速度峰值 {result.max_joint_speed_deg_s:.1f}°/s", flush=True)
        print(f"  接近段最大行程 {result.approach_delta_deg:.1f}°", flush=True)
        print(f"  扫描段横向偏差 {result.max_transverse_mm:.2f} mm，轴线 {result.max_axis_deg:.2f}°，"
              f"z {result.z_range_m[0]:.3f}–{result.z_range_m[1]:.3f} m", flush=True)
        print(f"  全程离箱面最小 {result.min_standoff_mm:.1f} mm", flush=True)
        if result.violations:
            print("预检不通过：", flush=True)
            for v in result.violations[:20]:
                print(f"  - {v}", flush=True)
            print(f"报告 {args.report}", flush=True)
            return FAULT_EXIT
        print("预检通过：可以从这个姿态开始真跑。", flush=True)
        print(f"报告 {args.report}", flush=True)
        return 0

    runner = Runner(controller, codec, publisher, profile,
                    log_path=args.log, hold=args.hold).open_log()
    fault = None
    fault_detected_at = None
    try:
        runner.run(lambda: cache["state"])
    except ControlFault as exc:
        fault = str(exc)
        fault_detected_at = time.monotonic()
        print(f"\n保护触发，停止发送指令：{fault}", flush=True)
        print("机器人不会再收到新的运动指令；它仍在执行上一条指令（KP/KD 未清零）。", flush=True)
    finally:
        runner.close()
        subscriber.Close()
        publisher.Close()

    report = {"network": args.network, "domain": args.domain,
              "captured_q_deg": np.degrees(captured).round(2).tolist(),
              "hold_joints": codec.hold_names,
              "hold_captured_deg": (np.degrees(hold_captured).round(2).tolist()
                                    if hold_captured is not None else []),
              "commands_sent": runner.writes,
              "command_rate_hz": round(runner.writes / max(runner.duration, 1e-9), 1),
              "loop_dt_max_ms": round(runner.dt_max * 1000, 1),
              "states_good": cache["good"],
              "states_bad": cache["bad"],
              "last_state_error": cache["error"],
              "duration_s": round(runner.duration, 2),
              "phases": runner.phase_history,
              "max_tracking_error_rad": round(runner.max_error, 4),
              "model_scan_seconds": round(motion.duration, 2),
              "final_q_deg": np.degrees(controller.last_q).round(2).tolist(),
              "complete": controller.complete,
              "fault": fault,
              # 三个绝对时刻（CLOCK_MONOTONIC，全机可比）：注入方靠它们量"多久停的"
              "fault_detected_at": fault_detected_at,
              "last_command_at": runner.last_write_at}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return FAULT_EXIT if fault else 0


if __name__ == "__main__":
    raise SystemExit(main())
