"""第一列上扫、顶部底盘换列、第二列下扫的最小真机 Demo。"""
import argparse
from contextlib import ExitStack
import csv
import json
from pathlib import Path
import signal
import sys
from threading import Event
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from continuous_demo import CONFIG_PATH, load_config
from controller import load_motion, side_scan_limits
from dds_network import initialize
from lift_dds import CMD_TOPIC, SIM_ZERO_GAP_M, Twist_
from profile import load as load_profile
from sync_scan import (
    ENDPOINT_SPEED_M_S, FEEDBACK_TIMEOUT, LiftChannel, PROFILE, lift_target,
    position_lift_at_low, prepare, verify_start,
)
from transport import Codec, ControlFault


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "models" / "r1_a7_scene_side_sync.xml"
DIRECTION_MARKER = ROOT / "out" / "base_direction_confirmation.json"
BASE_STOP_SECONDS = 0.5
BASE_VELOCITY_TOL = 0.03
TOP_HOLD_SECONDS = 0.3
DIRECTION_STOW_TOL_DEG = 20.0
KNOWN_FALSE_MODEL_PAIRS = tuple(
    ("probe_body", f"lift_sleeve_{side}")
    for side in ("front", "back", "left", "right")
)


def direction_marker():
    try:
        payload = json.loads(DIRECTION_MARKER.read_text(encoding="utf-8"))
        age = time.time() - float(payload["confirmed_unix_s"])
        valid = bool(payload.get("confirmed")) and 0 <= age <= 12 * 3600
        return valid, payload, age
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
        return False, None, None


class ShiftEngine:
    """单次上扫、换列、下扫状态机；所有 cmd_vel 由 LiftChannel 独占发布。"""

    def __init__(self, controller, codec, arm_publisher, arm_profile, lift, geometry,
                 config, directory, state_getter, move_command, move_seconds, abort):
        self.c = controller
        self.codec = codec
        self.arm_publisher = arm_publisher
        self.arm_profile = arm_profile
        self.lift = lift
        self.geometry = geometry
        self.cfg = config
        self.directory = directory
        self.state_getter = state_getter
        self.move_command = float(move_command)
        self.move_seconds = float(move_seconds)
        self.abort = abort
        self.phase = "startup-hold"
        self.phase_started = time.monotonic()
        self.started = self.phase_started
        self.done = False
        self.stow_complete = False
        self.writes = 0
        self.dt_max = 0.0
        self.max_arm_error = 0.0
        self.max_forward_feedback = 0.0
        self.phase_history = []
        self.endpoints = {}
        self.coverage = {
            "column_1_up": [float("inf"), float("-inf")],
            "column_2_down": [float("inf"), float("-inf")],
        }
        self.base_stop = None

    def enter(self, phase, now):
        self.phase = phase
        self.phase_started = now
        item = {"phase": phase, "t_s": round(now - self.started, 6)}
        self.phase_history.append(item)
        print(f"换列 Demo：phase={phase}", flush=True)

    def endpoint(self, state, q_target, height, vertical_speed, wanted_height):
        arm_error = float(np.max(np.abs(state.q - q_target)))
        ok = (arm_error <= self.cfg["arm_endpoint_tolerance_rad"]
              and abs(height - wanted_height) <= self.cfg["endpoint_tolerance_m"]
              and abs(vertical_speed) <= float(self.cfg["endpoint_speed_m_s"]))
        return ok, arm_error

    def chassis(self, now):
        item = self.lift.chassis_velocity
        if item is None or now - item[0] > FEEDBACK_TIMEOUT:
            raise ControlFault("rt/state_vel 底盘速度反馈缺失或超时")
        return tuple(float(value) for value in item[1:])

    def record_endpoint(self, name, height, vertical_speed, arm_error, base):
        self.endpoints[name] = {
            "height_mm": round(height * 1000.0, 3),
            "vertical_speed_m_s": round(vertical_speed, 6),
            "arm_error_deg": round(float(np.degrees(arm_error)), 3),
            "base_velocity": [round(value, 6) for value in base],
        }

    def update_coverage(self, state, height):
        key = {"scan-up": "column_1_up", "scan-down": "column_2_down"}.get(self.phase)
        if key is None:
            return
        z = float(self.geometry.position(state.q, height)[2])
        self.coverage[key][0] = min(self.coverage[key][0], z)
        self.coverage[key][1] = max(self.coverage[key][1], z)

    def run(self):
        names = self.arm_profile.names()
        q_top = self.c.motion.sample(self.c.motion.one_way_duration)
        low, high = self.cfg["low_m"], self.cfg["high_m"]
        last_csv = 0.0
        last_flush = self.started
        deadline = time.monotonic()
        self.c.previous = deadline
        self.c.last_q = self.c.captured.copy()
        with (self.directory / "arm.csv").open("w", newline="", encoding="utf-8") as arm_file, \
                (self.directory / "cmd_vel.csv").open("w", newline="", encoding="utf-8") as cmd_file:
            arm_writer, cmd_writer = csv.writer(arm_file), csv.writer(cmd_file)
            arm_writer.writerow(["t_s", "phase"] + [f"q_cmd_{n}" for n in names]
                                + [f"q_act_{n}" for n in names])
            cmd_writer.writerow(["t_s", "phase", "target_height_m", "height_m",
                                 "linear_x_cmd", "linear_z_cmd", "vx_feedback",
                                 "vy_feedback", "yaw_feedback", "vz_m_s"])
            while not self.done:
                now = time.monotonic()
                if self.abort.is_set():
                    raise ControlFault("操作者中止换列 Demo")
                phase_before = self.phase
                state = self.state_getter()
                dt = now - self.c.previous
                self.dt_max = max(self.dt_max, dt)
                elapsed = now - self.phase_started
                target_v = 0.0
                direction = 0
                base_command = 0.0

                if self.phase == "startup-hold":
                    q_target, target_height = self.c.captured, low
                    if elapsed >= float(self.cfg["startup_hold_s"]):
                        self.enter("approach", now)
                elif self.phase == "approach":
                    target_height = low
                    q_target = (self.c.q_scan_start if elapsed >= self.c.approach_seconds
                                else self.c._transfer_at(self.c.approach, elapsed))
                    if elapsed >= self.c.approach_seconds:
                        self.enter("scan-ready", now)
                elif self.phase == "scan-ready":
                    q_target, target_height = self.c.q_scan_start, low
                elif self.phase == "scan-up":
                    scan_t = min(elapsed, self.c.motion.one_way_duration)
                    q_target = self.c.motion.sample(scan_t)
                    target_height, target_v = lift_target(self.c.motion, scan_t, low, high)
                    direction = 1
                elif self.phase in ("top-hold", "column-shift", "base-stop"):
                    q_target, target_height = q_top, high
                    if self.phase == "column-shift":
                        base_command = self.move_command
                elif self.phase == "scan-down":
                    scan_t = min(elapsed, self.c.motion.one_way_duration)
                    motion_t = self.c.motion.one_way_duration + scan_t
                    q_target = self.c.motion.sample(motion_t)
                    target_height, target_v = lift_target(self.c.motion, motion_t, low, high)
                    direction = -1
                elif self.phase == "bottom-hold":
                    q_target, target_height = self.c.q_scan_start, low
                elif self.phase == "return":
                    target_height = low
                    q_target = (self.c.stow if elapsed >= self.c.return_seconds
                                else self.c._transfer_at(self.c.retract, elapsed))
                else:
                    q_target, target_height = self.c.stow, low

                self.lift.set_context(self.phase, target_height)
                height, vertical_speed = self.lift.feedback(now)
                vx, vy, yaw = self.chassis(now)
                if self.phase in ("scan-up", "scan-down"):
                    if abs(target_height - height) > self.cfg["tracking_tolerance_m"]:
                        raise ControlFault(
                            f"立柱跟踪误差过大：目标 {target_height*1000:.1f} mm，"
                            f"实际 {height*1000:.1f} mm")
                elif (abs(target_height - height) > 0.010
                      or abs(vertical_speed) > ENDPOINT_SPEED_M_S):
                    raise ControlFault(
                        f"{self.phase} 阶段立柱未保持端点：{height*1000:.1f} mm，"
                        f"{vertical_speed:.4f} m/s")

                self.c.phase = self.phase
                feedforward = self.c.gravity(q_target)
                self.c.check(q_target, state, now, dt, feedforward)
                command = self.codec.encode(q_target, state, feedforward)
                if not self.arm_publisher.Write(command):
                    raise ControlFault("换列 Demo LowCmd 写入失败")
                self.writes += 1
                arm_error = float(np.max(np.abs(q_target - state.q)))
                self.max_arm_error = max(self.max_arm_error, arm_error)
                self.c.last_q = np.asarray(q_target).copy()
                self.c.previous = now
                self.lift.target = (now, target_height, target_v, direction)
                self.lift.base_command = base_command
                self.max_forward_feedback = max(self.max_forward_feedback, vx)

                if now - last_csv >= 0.02:
                    arm_writer.writerow([now - self.started, self.phase, *q_target, *state.q])
                    cmd_writer.writerow([now - self.started, self.phase, target_height, height,
                                         base_command, self.lift.normalized_command,
                                         vx, vy, yaw, vertical_speed])
                    self.update_coverage(state, height)
                    last_csv = now
                if now - last_flush >= 1.0:
                    arm_file.flush(); cmd_file.flush()
                    last_flush = now

                endpoint_ok, endpoint_arm_error = self.endpoint(
                    state, q_target, height, vertical_speed, target_height)
                base_stopped = max(abs(vx), abs(vy), abs(yaw)) <= BASE_VELOCITY_TOL
                if phase_before == "scan-ready" and elapsed >= self.c.ready_seconds:
                    if not endpoint_ok or not base_stopped:
                        raise ControlFault("开始上扫前手臂、立柱或底盘未停稳")
                    self.record_endpoint("column_1_start", height, vertical_speed,
                                         endpoint_arm_error, (vx, vy, yaw))
                    self.enter("scan-up", now)
                elif (phase_before == "scan-up"
                      and elapsed >= self.c.motion.one_way_duration):
                    self.lift.target = (now, high, 0.0, 0)
                    self.enter("top-hold", now)
                elif phase_before == "top-hold" and elapsed >= TOP_HOLD_SECONDS:
                    if not endpoint_ok or not base_stopped:
                        if elapsed > float(self.cfg["endpoint_settle_timeout_s"]):
                            raise ControlFault("顶部手臂、立柱或底盘未按时同步停稳")
                    else:
                        self.record_endpoint("column_1_top", height, vertical_speed,
                                             endpoint_arm_error, (vx, vy, yaw))
                        self.enter("column-shift", now)
                elif phase_before == "column-shift" and elapsed >= self.move_seconds:
                    self.lift.base_command = 0.0
                    self.lift.target = (now, high, 0.0, 0)
                    self.enter("base-stop", now)
                elif phase_before == "base-stop" and elapsed >= BASE_STOP_SECONDS:
                    if not endpoint_ok or not base_stopped:
                        if elapsed > BASE_STOP_SECONDS + float(self.cfg["endpoint_settle_timeout_s"]):
                            raise ControlFault("换列后底盘、立柱或顶部手臂未按时停稳")
                    else:
                        self.base_stop = {
                            "zero_command_seconds": round(elapsed, 3),
                            "velocity": [round(vx, 6), round(vy, 6), round(yaw, 6)],
                        }
                        self.record_endpoint("column_2_top", height, vertical_speed,
                                             endpoint_arm_error, (vx, vy, yaw))
                        self.enter("scan-down", now)
                elif (phase_before == "scan-down"
                      and elapsed >= self.c.motion.one_way_duration):
                    self.lift.target = (now, low, 0.0, 0)
                    self.enter("bottom-hold", now)
                elif phase_before == "bottom-hold":
                    if endpoint_ok and base_stopped:
                        self.record_endpoint("column_2_bottom", height, vertical_speed,
                                             endpoint_arm_error, (vx, vy, yaw))
                        self.enter("return", now)
                    elif elapsed > float(self.cfg["endpoint_settle_timeout_s"]):
                        raise ControlFault("第二列下端点未按时同步停稳")
                elif phase_before == "return" and elapsed >= self.c.return_seconds:
                    self.enter("finished-hold", now)
                elif phase_before == "finished-hold":
                    stow_error = float(np.max(np.abs(state.q - self.c.stow)))
                    if (elapsed >= float(self.cfg["final_hold_s"])
                            and stow_error <= self.cfg["arm_endpoint_tolerance_rad"]):
                        self.stow_complete = True
                        self.done = True
                    elif elapsed > float(self.cfg["final_settle_timeout_s"]):
                        raise ControlFault("收纳未按时到位")

                deadline += 0.005
                slack = deadline - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    deadline = time.monotonic()
            arm_file.flush(); cmd_file.flush()

    def report(self, planned, entry, fault, args):
        coverage = {}
        for key, (low, high) in self.coverage.items():
            coverage[key] = None if not np.isfinite([low, high]).all() else high - low
        return {
            "mode": "scan-shift-scan",
            "complete": fault is None and self.done and self.stow_complete,
            "fault": fault,
            "network": args.network,
            "domain": args.domain,
            "move_command": args.move_command,
            "move_seconds": args.move_seconds,
            "move_rate_hz": args.move_rate,
            "open_loop_distance": "not estimated",
            "phases": self.phase_history,
            "endpoints": self.endpoints,
            "base_stop": self.base_stop,
            "max_forward_velocity_feedback": self.max_forward_feedback,
            "coverage_m": coverage,
            "stow_complete": self.stow_complete,
            "arm_commands": self.writes,
            "shared_twist_commands": self.lift.command_writes,
            "arm_loop_max_ms": self.dt_max * 1000.0,
            "arm_max_tracking_deg": float(np.degrees(self.max_arm_error)),
            "lift_entry": entry,
            "planned": planned,
            "feedback_health": self.lift.feedback_health(),
            "stale_events": list(self.lift.stale_events),
        }


def run_direction_test(c, codec, arm_publisher, lift, state_getter, args, abort, directory):
    tolerance = np.radians(DIRECTION_STOW_TOL_DEG)
    if np.max(np.abs(state_getter().q - c.stow)) > tolerance:
        raise ControlFault("底盘方向测试只允许在左臂收纳姿态下运行")
    # 方向测试只保持进入测试时的实测收纳姿态，不把手臂拉向另一个目标。
    hold_q = c.captured.copy()
    started = time.monotonic()
    stop_started = None
    max_forward = 0.0
    writes = 0
    c.previous = started
    c.last_q = hold_q.copy()
    deadline = started
    with (directory / "direction_test.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t_s", "phase", "linear_x_cmd", "vx", "vy", "yaw"])
        while True:
            now = time.monotonic()
            if abort.is_set():
                raise ControlFault("操作者中止底盘方向测试")
            elapsed = now - started
            moving = elapsed < args.move_seconds
            phase = "column-shift" if moving else "base-stop"
            command_x = args.move_command if moving else 0.0
            if not moving and stop_started is None:
                stop_started = now
            lift.set_context(phase, lift.height[1])
            height, vertical_speed = lift.feedback(now)
            _, vx, vy, yaw = lift.chassis_velocity
            if abs(vertical_speed) > ENDPOINT_SPEED_M_S:
                raise ControlFault("底盘方向测试期间立柱发生移动")
            state = state_getter()
            c.phase = "base-direction-test"
            feedforward = c.gravity(hold_q)
            c.check(hold_q, state, now, now - c.previous, feedforward)
            if not arm_publisher.Write(codec.encode(hold_q, state, feedforward)):
                raise ControlFault("方向测试 LowCmd 写入失败")
            writes += 1
            c.previous = now
            c.last_q = hold_q.copy()
            lift.target = (now, height, 0.0, 0)
            lift.base_command = command_x
            max_forward = max(max_forward, float(vx))
            if writes % 4 == 0:
                writer.writerow([elapsed, phase, command_x, vx, vy, yaw])
            stopped = max(abs(vx), abs(vy), abs(yaw)) <= BASE_VELOCITY_TOL
            if stop_started is not None and now - stop_started >= BASE_STOP_SECONDS and stopped:
                break
            if stop_started is not None and now - stop_started > BASE_STOP_SECONDS + 3.0:
                raise ControlFault("方向测试结束后底盘未停稳")
            deadline += 0.005
            time.sleep(max(0.0, deadline - time.monotonic()))
    lift.base_command = 0.0
    lift.target = (time.monotonic(), lift.height[1], 0.0, 0)
    return {"complete": True, "arm_stowed": True, "move_command": args.move_command,
            "move_seconds": args.move_seconds, "max_forward_velocity_feedback": max_forward,
            "arm_commands": writes}


def write_json(path, payload):
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain", type=int)
    parser.add_argument("--output", type=Path, default=ROOT / "out" / "scan_shift_scan")
    parser.add_argument("--model-offset-mm", type=float, default=SIM_ZERO_GAP_M * 1000.0)
    parser.add_argument("--move-command", type=float, default=0.05)
    parser.add_argument("--move-seconds", type=float, default=1.0)
    parser.add_argument("--move-rate", type=float, default=50.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--base-direction-test", action="store_true")
    args = parser.parse_args()
    if not 0.0 < args.move_command <= 0.20:
        parser.error("--move-command 必须在 (0, 0.20]；距离不足应优先增加时间")
    if not 0.1 <= args.move_seconds <= 10.0:
        parser.error("--move-seconds 必须在 0.1–10.0 秒")
    if not 20.0 <= args.move_rate <= 100.0:
        parser.error("--move-rate 必须在 20–100 Hz")
    real = args.network != "lo"
    if args.domain is None:
        args.domain = 0 if real else 220
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    low, high = cfg["low_m"], cfg["high_m"]
    offset = args.model_offset_mm / 1000.0
    if not np.isfinite(offset) or low + offset < 0.40 or high + offset > 0.96:
        parser.error("模型高度偏移使 330–830 mm 目标超出组合模型范围")

    profile, motion = load_motion(PROFILE)
    raw = json.loads((ROOT / "r1d_scan" / "robot_profile.json").read_text(encoding="utf-8"))
    arm = load_profile()
    if real and arm.gaps():
        raise SystemExit("真机配置未就绪：" + "；".join(arm.gaps()))
    codec = Codec(profile=arm)
    limits = side_scan_limits(profile)
    initialize(args.network, args.domain)
    state = None

    def on_arm(message):
        nonlocal state
        try:
            state = codec.decode(message, time.monotonic(), state)
        except ControlFault:
            pass

    with ExitStack() as channels, ExitStack() as watchers:
        competing = set()
        watch_end = time.monotonic()
        if not args.dry_run:
            for topic, kind in (("rt/lowcmd", LowCmd_), (CMD_TOPIC, Twist_)):
                reader = ChannelSubscriber(topic, kind)
                reader.Init(lambda _message, name=topic: competing.add(name), 1)
                watchers.callback(reader.Close)
            watch_end = time.monotonic() + 3.0
        lift = LiftChannel(channels)
        arm_reader = ChannelSubscriber("rt/lowstate", LowState_)
        arm_reader.Init(on_arm, 1)
        channels.callback(arm_reader.Close)
        try:
            deadline = time.monotonic() + 8.0
            stable_since = None
            while True:
                now = time.monotonic()
                complete = not any(value is None
                                   for value in (state, lift.height, lift.alarm,
                                                 lift.velocity, lift.chassis_velocity))
                base = () if lift.chassis_velocity is None else lift.chassis_velocity[1:]
                stable = (complete and now - state.received <= limits.state_timeout
                          and np.max(np.abs(state.dq)) <= 0.10
                          and abs(lift.velocity[1]) <= ENDPOINT_SPEED_M_S
                          and max(abs(value) for value in base) <= BASE_VELOCITY_TOL)
                stable_since = (stable_since or now) if stable else None
                if stable_since is not None and now - stable_since >= 0.25:
                    break
                if now >= deadline:
                    raise ControlFault("等待手臂、立柱和底盘完整静止反馈超时")
                time.sleep(0.02)
            initial = state
            initial_height, _ = verify_start(initial, initial, lift, limits, low_m=low)
            c, geometry, space, planned = prepare(
                initial, initial_height, raw, arm, codec, profile, motion, limits, offset,
                fast_preflight=real, low_m=low, high_m=high,
                ignored_base_pairs=KNOWN_FALSE_MODEL_PAIRS)
            planned.update(
                mode="base-direction-test" if args.base_direction_test else "scan-shift-scan",
                phases=["scan-up", "top-hold", "column-shift", "base-stop",
                        "scan-down", "stow"],
                move_command=args.move_command, move_seconds=args.move_seconds,
                move_rate_hz=args.move_rate, shared_cmd_vel_publisher=True,
                direction_stow_tolerance_deg=DIRECTION_STOW_TOL_DEG,
                model_collision_exclusion_reason=(
                    "负责人确认立柱300 mm以上实物无干涉；替代件包络在330 mm对"
                    "probe_body/lift_sleeve_*产生已复现的保守包络误报"),
                config=CONFIG_PATH.name)
            verify_start(state, initial, lift, limits, low_m=low)
            if (args.base_direction_test
                    and np.max(np.abs(state.q - c.stow))
                    > np.radians(DIRECTION_STOW_TOL_DEG)):
                raise ControlFault("底盘方向测试要求左臂已经处于收纳姿态")
            marker_valid, marker_payload, marker_age = direction_marker()
            preflight = {"ok": True, "published_commands": 0, "planned": planned,
                         "direction_marker_valid": marker_valid,
                         "direction_marker_age_s": marker_age}
            if args.dry_run:
                write_json(directory / "preflight.json", preflight)
                print(f"换列 Demo 只读预检通过，发布运动指令 0 条：{directory/'preflight.json'}")
                return 0
            if real:
                if not sys.stdin.isatty():
                    raise ControlFault("真机运行必须在交互终端回车确认")
                label = "收纳状态底盘正方向测试" if args.base_direction_test else "上扫-换列-下扫 Demo"
                print(f"即将运行：{label}；linear.x={args.move_command:.3f}，"
                      f"持续 {args.move_seconds:.2f} s。", flush=True)
                if input("确认前方清空、急停可用后直接按回车（其他输入取消）：") != "":
                    raise ControlFault("操作者取消")
                verify_start(state, initial, lift, limits, low_m=low)
            time.sleep(max(0.0, watch_end - time.monotonic()))
            if competing:
                raise ControlFault(f"检测到其他控制发布者：{', '.join(sorted(competing))}")
            arm_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
            arm_publisher.Init(); channels.callback(arm_publisher.Close)
            lift.open_publisher(channels)
            watchers.close()
            verify_start(state, initial, lift, limits, low_m=low)
            write_json(directory / "preflight.json", preflight)
        except ControlFault as exc:
            write_json(directory / "preflight.json",
                       {"ok": False, "published_commands": 0, "fault": str(exc)})
            print(f"换列 Demo 预检失败，发布运动指令 0 条：{exc}", flush=True)
            return 2

        abort = Event()

        def interrupt(_signum, _frame):
            abort.set()

        previous = signal.signal(signal.SIGINT, interrupt)
        previous_term = signal.signal(signal.SIGTERM, interrupt)
        fault = None
        entry = None
        report = None
        direction_result = None
        lift.allow_base_motion = True
        lift.command_period = 1.0 / args.move_rate
        lift.command_cap = float(cfg["lift_command_cap"])
        with ExitStack() as execution:
            execution.callback(signal.signal, signal.SIGINT, previous)
            execution.callback(signal.signal, signal.SIGTERM, previous_term)
            try:
                if args.base_direction_test:
                    lift.start(execution)
                    result = run_direction_test(
                        c, codec, arm_publisher, lift, lambda: state, args, abort, directory)
                    confirmed = not real
                    if real:
                        answer = input("观察确认正 linear.x 沿机器人正前方移动，请输入 FORWARD：")
                        confirmed = answer.strip().upper() == "FORWARD"
                    direction_result = dict(
                        result, confirmed=confirmed, confirmed_unix_s=time.time(),
                        network=args.network, domain=args.domain)
                else:
                    entry = position_lift_at_low(
                        c, codec, arm_publisher, lift, lambda: state, low_m=low,
                        stop_event=abort)
                    verify_start(state, initial, lift, limits, require_low=True,
                                 pose_tolerance_deg=3.0, low_m=low)
                    geometry.set_height(low)
                    issue = space.issue(state.q)
                    plate_gap, tube_gap = geometry.base_clearances()
                    if issue or plate_gap <= 0 or tube_gap <= 0:
                        raise ControlFault("330 mm归位后的组合碰撞复核失败："
                                           + (issue or "模型间隙不足"))
                    lift.start(execution)
                    engine = ShiftEngine(
                        c, codec, arm_publisher, arm, lift, geometry, cfg, directory,
                        lambda: state, args.move_command, args.move_seconds, abort)
                    engine.run()
            except ControlFault as exc:
                fault = str(exc)
            except KeyboardInterrupt:
                fault = "操作者立即中止换列 Demo"
        if lift.fault and lift.fault != fault:
            fault = f"{fault}；{lift.fault}" if fault else lift.fault
        if args.base_direction_test:
            if direction_result is not None and fault is None:
                marker = direction_result
            else:
                marker = {"complete": False, "confirmed": False, "fault": fault,
                          "confirmed_unix_s": time.time(), "network": args.network,
                          "domain": args.domain}
            write_json(DIRECTION_MARKER, marker)
            write_json(directory / "direction_report.json", marker)
            return 0 if marker.get("complete") and marker.get("confirmed") else 2
        if not args.base_direction_test:
            if "engine" in locals():
                report = engine.report(planned, entry, fault, args)
            else:
                report = {"mode": "scan-shift-scan", "complete": False,
                          "fault": fault, "lift_entry": entry,
                          "feedback_health": lift.feedback_health(),
                          "stale_events": list(lift.stale_events)}
            write_json(directory / "scan_shift_report.json", report)
            print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
            return 0 if report["complete"] else 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
