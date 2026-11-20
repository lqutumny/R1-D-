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
    parser.add_argument("--timeout", type=float, default=10.0, help="等首帧 LowState 的秒数")
    args = parser.parse_args()

    profile = load_profile()
    # 真机准入锁：非回环网卡必须使用已核对的 profile，否则一条指令都不发。
    # 仿真（lo）不受此限，因为它本来就跑在暂定槽位上。
    if args.network != "lo":
        gaps = profile.gaps()
        if gaps:
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
    _, motion = load_motion()

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
        codec.hold_targets = hold_captured
        names = ", ".join(f"{n}={np.degrees(v):.1f}°" for n, v in zip(codec.hold_names, hold_captured))
        print(f"保持关节（写固定值）: {names}", flush=True)
    # 自检：捕获姿态应当是"双臂下垂"，否则多半连到了旧的仿真器进程
    expected = {name: angle for name, angle in raw["initial"].items() if name != "note"}
    mismatch = [f"{name}={np.degrees(captured[profile.names().index(name)]):.1f}°"
                for name, angle in expected.items()
                if abs(captured[profile.names().index(name)] - angle) > np.radians(5)]
    if mismatch:
        print(f"警告：捕获姿态与预期下垂姿态不符（{', '.join(mismatch)}）。"
              "如果刚刚改过模型，请确认仿真器是重新启动的。", flush=True)

    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    controller = Controller(captured, motion, lo, hi, codec.kp, codec.kd, effort,
                            limits=Limits(startup_hold=2.0), start=time.monotonic(),
                            expected_mode_machine=cache["state"].mode_machine)
    # __init__ 里加载 MuJoCo 模型耗时 1–2 s，必须在构造完之后重新对齐时间基准
    controller.previous = time.monotonic()
    controller.start = controller.previous + controller.limits.startup_hold

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
