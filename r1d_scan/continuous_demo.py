"""一次自检和就位后，连续执行左臂与升降柱竖直往返扫描。"""
import argparse
from collections import deque
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

from controller import load_motion, side_scan_limits
from dds_network import initialize
from lift_dds import (
    CMD_TOPIC, COMMAND_CAP, MAX_SPEED_M_S, SIM_ZERO_GAP_M, Twist_, vertical_twist,
)
from profile import load as load_profile
from sync_scan import (
    AUTO_ENTRY_MAX_M, AUTO_ENTRY_MIN_M, ENDPOINT_SPEED_M_S, LiftChannel, PROFILE,
    SCENE_SIDE_SYNC, lift_target, position_lift_at_low, prepare, verify_start,
)
from transport import Codec, ControlFault


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "models" / "continuous_demo.json"


def load_config():
    raw = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    cfg = dict(raw)
    cfg["low_m"] = float(raw["low_height_mm"]) / 1000.0
    cfg["high_m"] = float(raw["high_height_mm"]) / 1000.0
    cfg["entry_tolerance_m"] = float(raw["entry_tolerance_mm"]) / 1000.0
    cfg["endpoint_tolerance_m"] = float(raw["endpoint_tolerance_mm"]) / 1000.0
    cfg["tracking_tolerance_m"] = float(raw["tracking_tolerance_mm"]) / 1000.0
    cfg["arm_endpoint_tolerance_rad"] = np.radians(float(raw["arm_endpoint_tolerance_deg"]))
    if not (AUTO_ENTRY_MIN_M <= cfg["low_m"] < cfg["high_m"] <= AUTO_ENTRY_MAX_M):
        raise ValueError("连续演示端点必须位于现有 300–850 mm 可信起始高度范围")
    if not np.isclose(cfg["high_m"] - cfg["low_m"], 0.500, atol=1e-9):
        raise ValueError("连续演示立柱单程必须为 500 mm")
    if not COMMAND_CAP <= float(cfg["lift_command_cap"]) <= 1.0:
        raise ValueError("连续演示立柱归一化命令上限必须位于现有单轮上限与 1.0 之间")
    if not 1.0 < float(cfg["conservative_time_scale"]) <= 2.0:
        raise ValueError("保守档时间倍率必须大于 1 且不超过 2")
    return cfg


class ContinuousEngine:
    """有界内存的连续控制状态机；周期明细直接流式写盘。"""

    def __init__(self, controller, codec, publisher, arm_profile, lift, geometry, config,
                 directory, state_getter, max_cycles, graceful_stop, hard_stop):
        self.c = controller
        self.codec = codec
        self.publisher = publisher
        self.arm_profile = arm_profile
        self.lift = lift
        self.lift.command_cap = float(config["lift_command_cap"])
        self.geometry = geometry
        self.cfg = config
        self.directory = directory
        self.state_getter = state_getter
        self.max_cycles = max_cycles
        self.time_scale = float(config["active_time_scale"])
        self.graceful_stop = graceful_stop
        self.hard_stop = hard_stop
        self.phase = "startup-hold"
        self.phase_started = time.monotonic()
        self.started = self.phase_started
        self.cycles_completed = 0
        self.writes = 0
        self.dt_max = 0.0
        self.max_arm_error = 0.0
        self.observed_height = [float("inf"), float("-inf")]
        self.stop_mode = None
        self.stow_complete = False
        self.current_cycle = None
        self.recent_cycles = deque(maxlen=int(config["recent_cycle_limit"]))
        self.lift.stale_events = deque(
            self.lift.stale_events, maxlen=int(config["stale_event_limit"]))
        self.last_csv = 0.0
        self.last_flush = self.started
        self.done = False

    def _enter(self, phase, now):
        self.phase = phase
        self.phase_started = now
        cycle_number = (self.cycles_completed if phase in ("return", "finished-hold")
                        and self.cycles_completed else self.cycles_completed + 1)
        print(f"连续演示：cycle={cycle_number} phase={phase}", flush=True)
        if phase == "scan-up":
            self.current_cycle = {
                "cycle": self.cycles_completed + 1,
                "started_t_s": round(now - self.started, 6),
                "max_lift_tracking_error_mm": 0.0,
                "max_arm_tracking_error_deg": 0.0,
                "min_tcp_z_m": None,
                "max_tcp_z_m": None,
            }

    def _endpoint_ok(self, state, q_target, height, speed, target_height):
        arm_error = float(np.max(np.abs(state.q - q_target)))
        return (arm_error <= self.cfg["arm_endpoint_tolerance_rad"]
                and abs(height - target_height) <= self.cfg["endpoint_tolerance_m"]
                and abs(speed) <= float(self.cfg["endpoint_speed_m_s"])), arm_error

    def _sample_cycle(self, state, height, target_height, arm_error):
        if self.current_cycle is None:
            return
        self.current_cycle["max_lift_tracking_error_mm"] = max(
            self.current_cycle["max_lift_tracking_error_mm"],
            1000.0 * abs(target_height - height))
        self.current_cycle["max_arm_tracking_error_deg"] = max(
            self.current_cycle["max_arm_tracking_error_deg"],
            float(np.degrees(arm_error)))
        z = float(self.geometry.position(state.q, height)[2])
        low = self.current_cycle["min_tcp_z_m"]
        high = self.current_cycle["max_tcp_z_m"]
        self.current_cycle["min_tcp_z_m"] = z if low is None else min(low, z)
        self.current_cycle["max_tcp_z_m"] = z if high is None else max(high, z)

    def _write_cycle(self, now, height, speed, arm_error):
        cycle = self.current_cycle
        cycle.update({
            "finished_t_s": round(now - self.started, 6),
            "duration_s": round(now - self.started - cycle["started_t_s"], 6),
            "bottom": {
                "target_height_mm": round(self.cfg["low_m"] * 1000.0, 3),
                "actual_height_mm": round(height * 1000.0, 3),
                "height_error_mm": round((height - self.cfg["low_m"]) * 1000.0, 3),
                "speed_m_s": round(speed, 6),
                "arm_error_deg": round(float(np.degrees(arm_error)), 3),
            },
        })
        low, high = cycle["min_tcp_z_m"], cycle["max_tcp_z_m"]
        cycle["actual_coverage_m"] = None if low is None else float(high - low)
        with (self.directory / "cycles.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(cycle, ensure_ascii=False) + "\n")
            stream.flush()
        self.recent_cycles.append(cycle)
        self.cycles_completed += 1
        print(f"完成循环 {self.cycles_completed}：低端 {height * 1000:.2f} mm，"
              f"覆盖 {cycle['actual_coverage_m'] * 1000:.1f} mm", flush=True)
        self.current_cycle = None

    def run(self):
        names = self.arm_profile.names()
        with (self.directory / "arm.csv").open("w", newline="", encoding="utf-8") as arm_file, \
                (self.directory / "lift.csv").open("w", newline="", encoding="utf-8") as lift_file:
            arm_writer, lift_writer = csv.writer(arm_file), csv.writer(lift_file)
            arm_writer.writerow(["t_s", "cycle", "phase", "phase_t_s"]
                                + [f"q_cmd_{n}" for n in names]
                                + [f"q_act_{n}" for n in names]
                                + [f"tau_{n}" for n in names])
            lift_writer.writerow(["t_s", "cycle", "phase", "phase_t_s",
                                  "target_height_m", "height_m", "target_v_m_s", "v_m_s",
                                  "normalized_cmd"])
            deadline = time.monotonic()
            self.c.previous = deadline
            self.c.last_q = self.c.captured.copy()
            while not self.done:
                now = time.monotonic()
                if self.hard_stop.is_set():
                    self.stop_mode = "second-ctrl-c"
                    raise ControlFault("操作者第二次中止，立即安全停止")
                state = self.state_getter()
                dt = now - self.c.previous
                self.dt_max = max(self.dt_max, dt)
                elapsed = now - self.phase_started
                low, high = self.cfg["low_m"], self.cfg["high_m"]
                direction = 0
                target_v = 0.0

                if self.phase == "startup-hold":
                    q_target, target_height = self.c.captured, low
                    if elapsed >= float(self.cfg["startup_hold_s"]):
                        self._enter("approach", now)
                elif self.phase == "approach":
                    target_height = low
                    q_target = (self.c.q_scan_start if elapsed >= self.c.approach_seconds
                                else self.c._transfer_at(self.c.approach, elapsed))
                    if elapsed >= self.c.approach_seconds:
                        self._enter("scan-ready", now)
                elif self.phase == "scan-ready":
                    q_target, target_height = self.c.q_scan_start, low
                    if elapsed >= self.c.ready_seconds:
                        ok, _ = self._endpoint_ok(state, q_target, *self.lift.feedback(now), low)
                        if not ok:
                            raise ControlFault("连续演示开始前手臂或立柱未在扫描下端点停稳")
                        self._enter("scan-up", now)
                elif self.phase == "scan-up":
                    scan_t = min(elapsed / self.time_scale, self.c.motion.one_way_duration)
                    q_target = self.c.motion.sample(scan_t)
                    target_height, target_v = lift_target(self.c.motion, scan_t, low, high)
                    target_v /= self.time_scale
                    direction = 1
                    if elapsed >= self.c.motion.one_way_duration * self.time_scale:
                        self._enter("top-settle", now)
                elif self.phase == "top-settle":
                    q_target, target_height = self.c.motion.sample(self.c.motion.one_way_duration), high
                elif self.phase == "scan-down":
                    scan_t = min(elapsed / self.time_scale, self.c.motion.one_way_duration)
                    motion_t = self.c.motion.one_way_duration + scan_t
                    q_target = self.c.motion.sample(motion_t)
                    target_height, target_v = lift_target(self.c.motion, motion_t, low, high)
                    target_v /= self.time_scale
                    direction = -1
                    if elapsed >= self.c.motion.one_way_duration * self.time_scale:
                        self._enter("bottom-settle", now)
                elif self.phase == "bottom-settle":
                    q_target, target_height = self.c.q_scan_start, low
                elif self.phase == "return":
                    target_height = low
                    q_target = (self.c.stow if elapsed >= self.c.return_seconds
                                else self.c._transfer_at(self.c.retract, elapsed))
                    if elapsed >= self.c.return_seconds:
                        self._enter("finished-hold", now)
                else:
                    q_target, target_height = self.c.stow, low
                    stow_error = float(np.max(np.abs(state.q - self.c.stow)))
                    if (elapsed >= float(self.cfg["final_hold_s"])
                            and stow_error <= self.cfg["arm_endpoint_tolerance_rad"]):
                        self.stow_complete = True
                        self.done = True
                    elif elapsed > float(self.cfg["final_settle_timeout_s"]):
                        raise ControlFault(
                            f"收纳未按时到位：手臂误差 {np.degrees(stow_error):.2f} deg")

                self.lift.set_context(self.phase, target_height)
                height, speed = self.lift.feedback(now)
                self.observed_height[0] = min(self.observed_height[0], height)
                self.observed_height[1] = max(self.observed_height[1], height)
                if self.phase in ("scan-up", "scan-down"):
                    if abs(target_height - height) > self.cfg["tracking_tolerance_m"]:
                        raise ControlFault(f"连续演示立柱跟踪误差过大：目标 {target_height*1000:.1f} mm，"
                                           f"实际 {height*1000:.1f} mm")
                elif abs(target_height - height) > 0.010 or abs(speed) > ENDPOINT_SPEED_M_S:
                    if self.phase not in ("top-settle", "bottom-settle"):
                        raise ControlFault(f"连续演示非扫描阶段立柱未停在端点：{height*1000:.1f} mm")

                self.c.phase = self.phase
                feedforward = self.c.gravity(q_target)
                self.c.check(q_target, state, now, dt, feedforward)
                command = self.codec.encode(q_target, state, feedforward)
                if not self.publisher.Write(command):
                    raise ControlFault("连续演示 LowCmd 写入失败")
                self.writes += 1
                arm_error = float(np.max(np.abs(q_target - state.q)))
                self.max_arm_error = max(self.max_arm_error, arm_error)
                self.c.last_q = np.asarray(q_target).copy()
                self.c.previous = now
                if now - self.last_csv >= float(self.cfg["csv_sample_period_s"]):
                    phase_t = now - self.phase_started
                    arm_writer.writerow([now - self.started, self.cycles_completed + 1,
                                         self.phase, phase_t, *q_target, *state.q, *state.tau])
                    lift_writer.writerow([now - self.started, self.cycles_completed + 1,
                                          self.phase, phase_t, target_height, height, target_v,
                                          speed, self.lift.normalized_command])
                    self._sample_cycle(state, height, target_height, arm_error)
                    self.last_csv = now
                if now - self.last_flush >= float(self.cfg["csv_flush_period_s"]):
                    arm_file.flush(); lift_file.flush()
                    self.last_flush = now

                if self.phase == "top-settle":
                    settle_elapsed = now - self.phase_started
                    ok, endpoint_arm_error = self._endpoint_ok(
                        state, self.c.motion.sample(self.c.motion.one_way_duration),
                        height, speed, high)
                    if ok:
                        self.current_cycle["top"] = {
                            "target_height_mm": high * 1000.0,
                            "actual_height_mm": round(height * 1000.0, 3),
                            "height_error_mm": round((height - high) * 1000.0, 3),
                            "speed_m_s": round(speed, 6),
                            "arm_error_deg": round(float(np.degrees(endpoint_arm_error)), 3),
                        }
                        self._enter("scan-down", now)
                    elif settle_elapsed > float(self.cfg["endpoint_settle_timeout_s"]):
                        raise ControlFault(f"顶部未按时同步到位：实际高度 {height*1000:.1f} mm")
                elif self.phase == "bottom-settle":
                    settle_elapsed = now - self.phase_started
                    ok, endpoint_arm_error = self._endpoint_ok(
                        state, self.c.q_scan_start, height, speed, low)
                    if ok:
                        self._write_cycle(now, height, speed, endpoint_arm_error)
                        stop_now = self.graceful_stop.is_set()
                        reached_limit = (self.max_cycles is not None
                                         and self.cycles_completed >= self.max_cycles)
                        if stop_now or reached_limit:
                            self.stop_mode = "ctrl-c" if stop_now else "max-cycles"
                            self._enter("return", now)
                        else:
                            self._enter("scan-up", now)
                    elif settle_elapsed > float(self.cfg["endpoint_settle_timeout_s"]):
                        raise ControlFault(f"底部未按时同步到位：实际高度 {height*1000:.1f} mm")

                # 端点确认可能在本拍切换阶段；按新阶段重置方向和时间基准，
                # 避免把换向前的旧方向多发送一拍。
                if self.phase == "scan-up":
                    target_height = low if direction != 1 else target_height
                    target_v, direction = (0.0, 1) if direction != 1 else (target_v, direction)
                elif self.phase == "scan-down":
                    target_height = high if direction != -1 else target_height
                    target_v, direction = (0.0, -1) if direction != -1 else (target_v, direction)
                else:
                    target_v, direction = 0.0, 0
                self.lift.set_context(self.phase, target_height)
                self.lift.target = (now, target_height, target_v, direction)

                deadline += 0.005
                slack = deadline - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    deadline = time.monotonic()
            arm_file.flush(); lift_file.flush()

    def report(self, planned, entry, fault, network, domain):
        observed = [None if not np.isfinite(v) else round(v * 1000.0, 3)
                    for v in self.observed_height]
        return {
            "mode": "continuous-demo",
            "network": network,
            "domain": domain,
            "complete": fault is None and self.done and self.stow_complete,
            "fault": fault,
            "stop_mode": self.stop_mode,
            "cycles_completed": self.cycles_completed,
            "incomplete_cycle_phase": None if self.done else self.phase,
            "target_height_mm": [self.cfg["low_height_mm"], self.cfg["high_height_mm"]],
            "lift_command_cap": self.lift.command_cap,
            "speed_mode": "conservative" if self.time_scale > 1.0 else "demo",
            "scan_time_scale": self.time_scale,
            "scan_one_way_s": self.c.motion.one_way_duration * self.time_scale,
            "observed_height_mm": observed,
            "arm_commands": self.writes,
            "arm_loop_max_ms": self.dt_max * 1000.0,
            "arm_max_tracking_deg": float(np.degrees(self.max_arm_error)),
            "stow_complete": self.stow_complete,
            "preflight": {"ok": True, "published_commands": 0},
            "lift_entry": entry,
            "planned": planned,
            "feedback_health": self.lift.feedback_health(),
            "stale_events": list(self.lift.stale_events),
            "recent_cycles": list(self.recent_cycles),
            "cycle_summaries": "cycles.jsonl",
        }


def write_preflight(directory, args, planned, fault=None):
    payload = {"ok": fault is None, "mode": "continuous-demo", "network": args.network,
               "domain": args.domain, "published_commands": 0, "fault": fault,
               "max_cycles": args.max_cycles, "planned": planned}
    (directory / "preflight.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain", type=int)
    parser.add_argument("--output", type=Path, default=ROOT / "out" / "continuous_demo")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-cycles", type=int)
    parser.add_argument("--conservative", action="store_true",
                        help="首轮真机验收档：复用同一轨迹并按集中配置放慢时间轴")
    parser.add_argument("--model-offset-mm", type=float, default=SIM_ZERO_GAP_M * 1000.0)
    args = parser.parse_args()
    if args.max_cycles is not None and args.max_cycles < 1:
        parser.error("--max-cycles 必须为正整数")
    real = args.network != "lo"
    if args.domain is None:
        args.domain = 0 if real else 218
    directory = args.output.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    config = load_config()
    config["active_time_scale"] = (float(config["conservative_time_scale"])
                                   if args.conservative else 1.0)
    low, high = config["low_m"], config["high_m"]
    offset = args.model_offset_mm / 1000.0
    if not np.isfinite(offset) or low + offset < 0.40 or high + offset > 0.96:
        parser.error("模型高度偏移使 330–830 mm 目标超出组合模型范围")

    profile, motion = load_motion(PROFILE)
    raw = json.loads((ROOT / "r1d_scan" / "robot_profile.json").read_text(encoding="utf-8"))
    arm = load_profile()
    if real and arm.gaps():
        raise SystemExit("真机连续演示配置未就绪：" + "；".join(arm.gaps()))
    codec = Codec(profile=arm)
    limits = side_scan_limits(profile)
    initialize(args.network, args.domain)
    state = None

    def on_arm(msg):
        nonlocal state
        try:
            state = codec.decode(msg, time.monotonic(), state)
        except ControlFault:
            pass

    with ExitStack() as channels, ExitStack() as watchers:
        competing = set()
        watch_end = time.monotonic()
        if not args.dry_run:
            for topic, kind in (("rt/lowcmd", LowCmd_), (CMD_TOPIC, Twist_)):
                reader = ChannelSubscriber(topic, kind)
                reader.Init(lambda _msg, name=topic: competing.add(name), 1)
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
                complete = not any(v is None for v in (state, lift.height, lift.alarm, lift.velocity))
                stable = (complete and now - state.received <= limits.state_timeout
                          and np.max(np.abs(state.dq)) <= .10
                          and abs(lift.velocity[1]) <= ENDPOINT_SPEED_M_S)
                stable_since = (stable_since or now) if stable else None
                if stable_since is not None and now - stable_since >= .25:
                    break
                if now >= deadline:
                    raise ControlFault("等待手臂和立柱的完整静止反馈超时")
                time.sleep(.02)
            initial = state
            initial_height, _ = verify_start(initial, initial, lift, limits, low_m=low)
            c, geometry, space, planned = prepare(
                initial, initial_height, raw, arm, codec, profile, motion, limits, offset,
                fast_preflight=real, low_m=low, high_m=high)
            planned.update(continuous_demo_config=CONFIG_PATH.name,
                           nominal_combined_coverage_m=profile["length_m"] + high - low,
                       speed_mode="conservative" if args.conservative else "demo",
                       scan_time_scale=config["active_time_scale"],
                       effective_scan_one_way_s=(motion.one_way_duration
                                                 * config["active_time_scale"]),
                           max_cycles=args.max_cycles)
            verify_start(state, initial, lift, limits, low_m=low)
            if args.dry_run:
                write_preflight(directory, args, planned)
                print(f"连续演示只读预检通过，发布运动指令 0 条：{directory/'preflight.json'}")
                return 0
            if real:
                print(f"连续演示预检通过：立柱 {low*1000:.0f}↔{high*1000:.0f} mm，"
                      f"最大轮数={args.max_cycles or '无限'}。第一次 Ctrl+C 完成本轮后收纳。")
                if not sys.stdin.isatty():
                    raise ControlFault("真机连续演示必须在交互终端运行")
                if input("确认运动范围清空、急停可用后直接按回车启动（其他输入取消）：") != "":
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
            write_preflight(directory, args, planned)
        except ControlFault as exc:
            write_preflight(directory, args, locals().get("planned"), str(exc))
            print(f"连续演示预检失败，发布运动指令 0 条：{exc}")
            return 2

        entry = None
        try:
            entry = position_lift_at_low(c, codec, arm_publisher, lift, lambda: state, low_m=low)
            verify_start(state, initial, lift, limits, require_low=True,
                         pose_tolerance_deg=3.0, low_m=low)
            geometry.set_height(low)
            issue = space.issue(state.q)
            plate_gap, tube_gap = geometry.base_clearances()
            if issue or plate_gap <= 0 or tube_gap <= 0:
                raise ControlFault("330 mm 自动归位后的组合碰撞复核失败："
                                   + (issue or "底座或升降管间隙不足"))
            entry.update(post_entry_min_arm_to_base_plate_m=float(plate_gap),
                         post_entry_min_arm_to_lift_tube_m=float(tube_gap))
            (directory / "lift_entry.json").write_text(
                json.dumps(entry, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        except (ControlFault, KeyboardInterrupt) as exc:
            reason = str(exc) if isinstance(exc, ControlFault) else "操作者中止自动归位"
            (directory / "lift_entry.json").write_text(
                json.dumps({"complete": False, "fault": reason}, ensure_ascii=False, indent=2)
                + "\n", encoding="utf-8")
            return 2

        graceful_stop, hard_stop = Event(), Event()
        interrupts = {"count": 0}

        def on_interrupt(_signum, _frame):
            interrupts["count"] += 1
            if interrupts["count"] == 1:
                graceful_stop.set()
                print("已请求正常停止：完成当前循环、回到 330 mm 后收纳。再次 Ctrl+C 将立即停止。",
                      flush=True)
            else:
                hard_stop.set()

        engine = ContinuousEngine(c, codec, arm_publisher, arm, lift, geometry, config,
                                  directory, lambda: state, args.max_cycles,
                                  graceful_stop, hard_stop)
        fault = None
        with ExitStack() as execution:
            previous = signal.signal(signal.SIGINT, on_interrupt)
            execution.callback(signal.signal, signal.SIGINT, previous)
            lift.start(execution)
            try:
                engine.run()
            except ControlFault as exc:
                fault = str(exc)
                engine.stop_mode = engine.stop_mode or "protection"
            except KeyboardInterrupt:
                fault = "操作者立即中止连续演示"
                engine.stop_mode = engine.stop_mode or "immediate-interrupt"
        if lift.fault and fault != lift.fault:
            fault = f"{fault}；{lift.fault}" if fault else lift.fault
        report = engine.report(planned, entry, fault, args.network, args.domain)
        (directory / "continuous_report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        return 0 if report["complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
