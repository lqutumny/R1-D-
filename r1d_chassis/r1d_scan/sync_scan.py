"""用同一条时间轴同步运行左臂与升降柱。"""
import argparse
from contextlib import ExitStack
import csv
import json
from math import expm1
from pathlib import Path
import signal
import sys
from threading import Event, Thread
import time

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

from controller import Controller, load_motion, side_scan_limits
from dds_network import initialize
from kinematics import ArmKinematics
from lift_dds import (
    ALARM_TOPIC, CMD_TOPIC, HEIGHT_TOPIC, VELOCITY_TOPIC, COMMAND_CAP,
    MAX_SPEED_M_S, SIM_ZERO_GAP_M,
    Error_, Point32_, Twist_, raw_height_mm, vertical_twist,
)
import preflight
from profile import load as load_profile
from runner import Runner
from transfer import Clearance, plan
from transport import Codec, ControlFault

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / "models" / "side_trial_60_51_profile.json"
SCENE_SIDE_SYNC = ROOT / "models" / "r1_a7_scene_side_sync.xml"
PERIOD = 0.02
LIFT_CORRECTION_TAU_S = 0.060
FEEDBACK_TIMEOUT = 0.20
LOW_M, HIGH_M = .400, .800
AUTO_ENTRY_MIN_M = .300
AUTO_ENTRY_MAX_M = .850
AUTO_ENTRY_TOL_M = .002
AUTO_ENTRY_STILL_S = .40
AUTO_ENTRY_COMMAND_CAP = .35
ENTRY_TOL_M = 0.010
ENDPOINT_TOL_M = 0.005
TRACKING_TOL_M = 0.020
ENDPOINT_SPEED_M_S = 0.005


def lift_target(motion, elapsed):
    progress, rate = motion.path_state(elapsed)
    travel = HIGH_M - LOW_M
    return LOW_M + travel * float(progress), travel * float(rate)


class LiftChannel:
    """在独立线程闭环跟踪高度。主循环只传递最新目标。"""

    def __init__(self, resources):
        self.height = None
        self.alarm = None
        self.velocity = None
        self.target = None
        self.fault = None
        self.normalized_command = 0.0
        self.stop = Event()
        for topic, kind, callback in (
            (HEIGHT_TOPIC, Point32_, self.on_height),
            (ALARM_TOPIC, Error_, self.on_alarm),
            (VELOCITY_TOPIC, Twist_, self.on_velocity),
        ):
            reader = ChannelSubscriber(topic, kind)
            reader.Init(callback, 1)
            resources.callback(reader.Close)
    def on_height(self, message):
        try:
            self.height = (time.monotonic(), raw_height_mm(message) / 1000.0)
        except ValueError as exc:
            self.fault = str(exc)

    def on_alarm(self, message):
        self.alarm = (time.monotonic(), int(message.state))
        if message.state:
            self.fault = f"立柱告警 source={message.source} state={message.state}"

    def on_velocity(self, message):
        x, y, z, yaw = (float(message.linear.x), float(message.linear.y),
                        float(message.linear.z), float(message.angular.z))
        if not np.isfinite([x, y, z, yaw]).all():
            self.fault = "底盘速度反馈含非有限值"
        elif max(abs(x), abs(y), abs(yaw)) > 0.03:
            self.fault = "检测到轮式底盘移动"
        self.velocity = (time.monotonic(), z * MAX_SPEED_M_S)

    def feedback(self, now, timeout=FEEDBACK_TIMEOUT):
        if self.fault:
            raise ControlFault(self.fault)
        readings = (("高度", self.height), ("告警", self.alarm), ("速度", self.velocity))
        stale = []
        for name, item in readings:
            if item is None:
                stale.append(f"{name}=未收到")
            elif now - item[0] > timeout:
                stale.append(f"{name}={1000 * (now - item[0]):.0f} ms")
        if stale:
            raise ControlFault(f"升降反馈缺失或超时（阈值 {1000 * timeout:.0f} ms："
                               + "，".join(stale) + "）")
        height, velocity = readings[0][1][1], readings[2][1][1]
        if not -0.020 <= height <= 0.860:
            raise ControlFault(f"立柱读数 {height * 1000:.1f} mm 超出反馈可信范围")
        return height, velocity

    def open_publisher(self, resources):
        self.publisher = ChannelPublisher(CMD_TOPIC, Twist_)
        self.publisher.Init()
        resources.callback(self.publisher.Close)

    def start(self, resources):
        thread = Thread(target=self.run, name="lift-dds", daemon=True)
        thread.start()

        def finish():
            self.stop.set()
            thread.join()
        resources.callback(finish)

    def run(self):
        correction = 0.0
        last_direction = 0
        last_tick = time.monotonic()
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                height, _ = self.feedback(now)
                target = self.target
                command = 0.0
                direction = 0
                if target is not None:
                    issued, wanted, speed, direction = target
                    if now - issued > FEEDBACK_TIMEOUT:
                        raise ControlFault("手臂扫描进度停止更新")
                    if direction != last_direction:
                        correction = 0.0
                    elif direction:
                        # 按实际周期滤掉 DDS 采样和调度造成的短时高度误差波动。
                        weight = -expm1(-(now - last_tick) / LIFT_CORRECTION_TAU_S)
                        correction += weight * (4.0 * (wanted - height) - correction)
                    if direction:
                        normalized = (speed + correction) / MAX_SPEED_M_S
                        # 到达误差容差以内时允许等待追上目标，避免换向前反向补偿。
                        command = direction * float(np.clip(direction * normalized, 0.0, COMMAND_CAP))
                if not self.publisher.Write(vertical_twist(command)):
                    raise ControlFault("升降速度发送失败")
                self.normalized_command = command
                last_direction = direction
                last_tick = now
                self.stop.wait(PERIOD)
        except Exception as exc:
            self.fault = str(exc)
        finally:
            sent_zero = False
            for attempt in range(3):
                try:
                    sent_zero = self.publisher.Write(vertical_twist(0.0)) or sent_zero
                except Exception:
                    pass
                if attempt < 2:
                    time.sleep(PERIOD)
            if not sent_zero:
                reason = "停止升降的零速指令发送失败"
                self.fault = f"{self.fault}；{reason}" if self.fault else reason
            self.normalized_command = 0.0


class SyncRunner(Runner):
    def __init__(self, *args, lift, **kwargs):
        super().__init__(*args, **kwargs)
        self.lift = lift
        self.lift_phase = None
        self.lift_rows = []
        self.endpoints = []
        self.sample_at = 0.0

    def step(self, state, now):
        height, speed = self.lift.feedback(now)
        c = self.controller
        elapsed = now - c.start - c.approach_seconds - c.ready_seconds
        target, velocity = lift_target(c.motion, elapsed)
        moving = 0 <= elapsed < c.motion.duration
        if elapsed < 0:
            phase = "before-scan"
        elif elapsed < c.motion.one_way_duration:
            phase = "scan-up"
        elif moving:
            phase = "scan-down"
        else:
            phase = "after-scan"
        if phase != self.lift_phase:
            endpoint = HIGH_M if phase == "scan-down" else LOW_M
            tolerance = ENDPOINT_TOL_M if phase in ("scan-down", "after-scan") else ENTRY_TOL_M
            if abs(endpoint - height) > tolerance or abs(speed) > ENDPOINT_SPEED_M_S:
                raise ControlFault(f"{phase}: 立柱未按时到位并停稳"
                                   f" ({height * 1000:.1f} mm, {speed:.4f} m/s)")
            self.endpoints.append({"phase": phase, "t_s": round(elapsed, 4),
                                   "height_mm": round(height * 1000, 3),
                                   "speed_m_s": round(speed, 6)})
            self.lift_phase = phase
        if abs(target - height) > TRACKING_TOL_M:
            raise ControlFault(f"立柱跟踪误差过大：目标 {target * 1000:.1f} mm，"
                               f"实际 {height * 1000:.1f} mm")
        if not moving and (abs(height - LOW_M) > ENTRY_TOL_M or abs(speed) > ENDPOINT_SPEED_M_S):
            raise ControlFault("非扫描阶段检测到立柱离开低位或仍在移动")

        keep_going = super().step(state, now)
        # 只有这一拍的手臂指令成功写入后，才推进立柱目标。
        direction = {"scan-up": 1, "scan-down": -1}.get(phase, 0)
        self.lift.target = (now, target, velocity, direction)
        if now >= self.sample_at:
            self.sample_at = now + PERIOD
            self.lift_rows.append((elapsed, phase, target, height, velocity, speed,
                                   self.lift.normalized_command, state.q.copy()))
        return keep_going


class Geometry:
    def __init__(self, held, reference_offset_m):
        self.kin = ArmKinematics(SCENE_SIDE_SYNC)
        self.model, self.data = self.kin.model, self.kin.data
        slide = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "lift_slide")
        self.slide = self.model.jnt_qposadr[slide]
        manifest = json.loads((ROOT / "models" / "simulation_lift.json").read_text())
        self.nominal = float(manifest["lift_nominal_gap_m"])
        self.reference_offset_m = reference_offset_m
        self.wall_axis = 1
        wall = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "wall")
        if wall < 0:
            raise ControlFault("侧向升降模型缺少箱面")
        self.wall_pos = float(self.model.geom_pos[wall, self.wall_axis]
                              - self.model.geom_size[wall, self.wall_axis])
        self.plate = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "base_plate")
        middle = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "lift_middle_visual")
        self.middle_mocap = int(self.model.body_mocapid[middle])
        self.middle_nominal_z = float(self.model.body_pos[middle, 2])
        self.tube_geoms = [
            i for i in range(self.model.ngeom)
            if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or "")
            .startswith(("lift_sleeve_", "lift_middle_", "lift_mast"))
        ]
        self.arm_geoms = [i for i, body in enumerate(self.model.geom_bodyid)
                          if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body) or "")
                          .startswith(("left_", "right_", "radiate_probe"))]
        for name, q in held.items():
            joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint >= 0:
                self.data.qpos[self.model.jnt_qposadr[joint]] = q

    def set_height(self, height):
        slide = height + self.reference_offset_m - self.nominal
        self.data.qpos[self.slide] = slide
        self.data.mocap_pos[self.middle_mocap, 2] = self.middle_nominal_z + 0.5 * slide
    def position(self, q, height):
        self.set_height(height)
        return self.kin.forward(q)[0]

    def base_clearances(self):
        # 底盘外形不参加物理接触，预检显式测量手臂及探头到各段的间距。
        mujoco.mj_fwdPosition(self.model, self.data)

        def minimum(geoms):
            return min(mujoco.mj_geomDistance(self.model, self.data, base, arm, 2.0, None)
                       for base in geoms for arm in self.arm_geoms)
        return minimum([self.plate]), minimum(self.tube_geoms)


def planned_geometry(controller, profile, geometry, space):
    c = controller
    motion = c.motion
    scan_start = c.start + c.approach_seconds + c.ready_seconds
    end = scan_start + motion.duration + c.return_seconds
    times = np.unique(np.r_[np.arange(0, end, 0.04), scan_start,
                            scan_start + motion.one_way_duration, scan_start + motion.duration, end])
    points, plate, tube, scan_z = [], [], [], []
    for t in times:
        phase, q = c.target(t)
        height, _ = lift_target(motion, t - scan_start)
        point = geometry.position(q, height)
        issue = space.issue(q)
        if issue:
            raise ControlFault(f"组合轨迹 {phase} t={t:.2f}s：{issue}")
        points.append(point)
        plate_gap, tube_gap = geometry.base_clearances()
        plate.append(plate_gap)
        tube.append(tube_gap)
        if phase in ("scan-up", "scan-down"):
            scan_z.append(float(point[2]))
    wall_axis = int(profile.get("wall_axis", 1))
    clearance = geometry.wall_pos - np.max(np.asarray(points)[:, wall_axis])
    result = {"coverage_m": float(np.ptp(scan_z)), "min_wall_gap_m": float(clearance),
              "min_arm_to_base_plate_m": float(min(plate)),
              "min_arm_to_lift_tube_m": float(min(tube))}
    if clearance < 0.1 or min(plate) <= 0 or min(tube) <= 0:
        raise ControlFault(f"组合轨迹预检不通过: {result}")
    return result


def planned_lift_entry(q, start_height, geometry, space):
    """手臂保持当前姿态时，检查当前立柱高度到 400 mm 的整段组合几何。"""
    count = max(2, int(np.ceil(abs(start_height - LOW_M) / .01)) + 1)
    plate, tube = [], []
    for height in np.linspace(start_height, LOW_M, count):
        geometry.set_height(float(height))
        issue = space.issue(q)
        if issue:
            raise ControlFault(f"立柱自动归位在 {height * 1000:.1f} mm 不可用：{issue}")
        plate_gap, tube_gap = geometry.base_clearances()
        if plate_gap <= 0 or tube_gap <= 0:
            raise ControlFault(f"立柱自动归位在 {height * 1000:.1f} mm 存在组合碰撞")
        plate.append(plate_gap)
        tube.append(tube_gap)
    geometry.set_height(LOW_M)
    return {
        "start_height_mm": round(start_height * 1000, 3),
        "target_height_mm": round(LOW_M * 1000, 3),
        "travel_mm": round(abs(start_height - LOW_M) * 1000, 3),
        "min_arm_to_base_plate_m": float(min(plate)),
        "min_arm_to_lift_tube_m": float(min(tube)),
    }


def position_lift_at_low(controller, codec, arm_publisher, lift, state_getter):
    """保持双臂当前姿态，把立柱闭环移到 400 mm 并确认停稳。"""
    target_q = controller.captured.copy()
    started = time.monotonic()
    start_height, _ = lift.feedback(started)
    timeout = max(8.0, abs(start_height - LOW_M)
                  / max(MAX_SPEED_M_S * AUTO_ENTRY_COMMAND_CAP, 1e-6) * 1.8 + 5.0)
    deadline = started + timeout
    stable_since = None
    previous = started
    next_lift_write = started
    arm_commands = lift_commands = 0
    peak_command = 0.0
    try:
        while True:
            now = time.monotonic()
            dt = now - previous
            if dt <= 0 or dt > controller.limits.loop_stall_stop:
                raise ControlFault(f"立柱自动归位控制周期异常 {dt * 1000:.1f} ms")
            state = state_getter()
            controller.phase = "lift-entry"
            feedforward = controller.gravity(target_q)
            controller.check(target_q, state, now, dt, feedforward)
            command = codec.encode(target_q, state, feedforward)
            if not arm_publisher.Write(command):
                raise ControlFault("立柱自动归位时手臂保持指令发送失败")
            arm_commands += 1
            controller.last_q = target_q.copy()
            controller.previous = now
            previous = now

            height, speed = lift.feedback(now)
            error = LOW_M - height
            if now >= next_lift_write:
                next_lift_write = now + PERIOD
                normalized = float(np.clip(4.0 * error / MAX_SPEED_M_S,
                                           -AUTO_ENTRY_COMMAND_CAP,
                                           AUTO_ENTRY_COMMAND_CAP))
                if abs(error) <= AUTO_ENTRY_TOL_M:
                    normalized = 0.0
                if not lift.publisher.Write(vertical_twist(normalized)):
                    raise ControlFault("立柱自动归位速度指令发送失败")
                lift.normalized_command = normalized
                peak_command = max(peak_command, abs(normalized))
                lift_commands += 1

            if abs(error) <= AUTO_ENTRY_TOL_M and abs(speed) <= ENDPOINT_SPEED_M_S:
                stable_since = stable_since or now
                if now - stable_since >= AUTO_ENTRY_STILL_S:
                    return {
                        "complete": True,
                        "start_height_mm": round(start_height * 1000, 3),
                        "end_height_mm": round(height * 1000, 3),
                        "duration_s": round(now - started, 3),
                        "arm_hold_commands": arm_commands,
                        "lift_commands": lift_commands,
                        "peak_normalized_command": round(peak_command, 4),
                    }
            else:
                stable_since = None
            if now >= deadline:
                raise ControlFault(f"立柱自动归位超时：当前 {height * 1000:.1f} mm，"
                                   f"目标 {LOW_M * 1000:.0f} mm")
            time.sleep(.005)
    finally:
        sent_zero = False
        for attempt in range(3):
            try:
                sent_zero = lift.publisher.Write(vertical_twist(0.0)) or sent_zero
            except Exception:
                pass
            if attempt < 2:
                time.sleep(PERIOD)
        lift.normalized_command = 0.0
        if not sent_zero and lift.fault is None:
            lift.fault = "立柱自动归位结束时零速指令发送失败"


def save_report(directory, runner, geometry, profile, planned, fault, network, domain):
    samples = runner.lift_rows
    xyz = [geometry.position(row[-1], row[3]) for row in samples]
    with (directory / "lift.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("scan_t_s", "phase", "target_height_m", "height_m", "target_v_m_s",
                         "v_m_s", "normalized_cmd", "tcp_x_m", "tcp_y_m", "tcp_z_m"))
        writer.writerows((*row[:-1], *point) for row, point in zip(samples, xyz))
    scan = [(row, point) for row, point in zip(samples, xyz) if row[1].startswith("scan-")]
    result = {"network": network, "domain": domain,
              "complete": runner.controller.complete and fault is None, "fault": fault,
              "height_reference": "DDS rt/height_state.z; no offset for control",
              "target_height_mm": [LOW_M * 1000, HIGH_M * 1000],
              "lift_correction_tau_s": LIFT_CORRECTION_TAU_S,
              "model_reference_offset_m": geometry.reference_offset_m,
              "stow_target_deg": np.degrees(runner.controller.stow).round(2).tolist(),
              "approach_segments": len(runner.controller.approach[0]) - 1,
              "planned": planned, "lift_endpoints": runner.endpoints, "arm_phases": runner.phase_history,
              "arm_commands": runner.writes, "arm_max_tracking_deg": float(np.degrees(runner.max_error)),
              "arm_loop_max_ms": runner.dt_max * 1000}
    if samples:
        result["last_measured_arm_deg"] = np.degrees(samples[-1][-1]).round(2).tolist()
    if scan:
        points = np.asarray([p for _, p in scan])
        target_xy = [profile["tcp_x_m"], profile["y_m"]]
        result.update(model_fk_tcp_coverage_m=float(np.ptp(points[:, 2])),
                      model_fk_scan_max_xy_error_m=float(np.linalg.norm(points[:, :2] - target_xy,
                                                                        axis=1).max()),
                      lift_max_tracking_error_m=max(abs(r[2] - r[3]) for r, _ in scan),
                      lift_range_m=[min(r[3] for r, _ in scan), max(r[3] for r, _ in scan)])
        checkpoints = []
        for label, wanted_t in (("scan-start", 0.0),
                                ("top-reversal", runner.controller.motion.one_way_duration),
                                ("scan-return", runner.controller.motion.duration)):
            index = min(range(len(samples)), key=lambda i: abs(samples[i][0] - wanted_t))
            row, actual_point = samples[index], xyz[index]
            target_height, _ = lift_target(runner.controller.motion, wanted_t)
            target_q = runner.controller.motion.sample(wanted_t)
            target_point = geometry.position(target_q, target_height)
            checkpoints.append({
                "name": label,
                "sample_time_error_ms": round(1000 * (row[0] - wanted_t), 2),
                "lift_target_mm": round(1000 * target_height, 3),
                "lift_actual_mm": round(1000 * row[3], 3),
                "lift_error_mm": round(1000 * (row[3] - target_height), 3),
                "probe_target_z_mm": round(1000 * target_point[2], 3),
                "probe_actual_z_mm": round(1000 * actual_point[2], 3),
                "probe_z_error_mm": round(1000 * (actual_point[2] - target_point[2]), 3),
            })
        result["synchronization_checkpoints"] = checkpoints
    payload = json.dumps(result, ensure_ascii=False, indent=2)
    (directory / "sync_report.json").write_text(payload + "\n", encoding="utf-8")
    print(payload, flush=True)


def verify_start(state, initial, lift, limits, require_low=False, pose_tolerance_deg=1.0):
    now = time.monotonic()
    if state is None or now - state.received > limits.state_timeout:
        raise ControlFault("手臂反馈缺失或超时")
    if state.mode_pr != 0 or state.mode_machine != initial.mode_machine:
        raise ControlFault("手臂控制模式不可用或已变化")
    if np.any(state.errors) or np.max(state.temperature) >= 70:
        raise ControlFault("手臂有故障或温度过高")
    if (np.max(np.abs(state.dq)) > 0.10 or
            np.max(np.abs(state.q - initial.q)) > np.radians(pose_tolerance_deg) or
            np.any(np.abs(state.hold_q - initial.hold_q) > np.radians(pose_tolerance_deg))):
        raise ControlFault("手臂未停稳或预检姿态已变化")
    if not np.isfinite(state.hold_q).all() or np.any(np.abs(state.hold_q) > np.pi):
        raise ControlFault("保持关节反馈异常")
    height, speed = lift.feedback(now)
    if not AUTO_ENTRY_MIN_M <= height <= AUTO_ENTRY_MAX_M:
        raise ControlFault(f"立柱自动归位只接受 {AUTO_ENTRY_MIN_M * 1000:.0f}–"
                           f"{AUTO_ENTRY_MAX_M * 1000:.0f} mm 的起始高度，"
                           f"当前 {height * 1000:.1f} mm")
    if require_low and (abs(height - LOW_M) > AUTO_ENTRY_TOL_M
                        or abs(speed) > ENDPOINT_SPEED_M_S):
        raise ControlFault(f"立柱自动归位未停在 {LOW_M * 1000:.0f} mm："
                           f"当前 {height * 1000:.1f} mm，速度 {speed:.4f} m/s")
    return height, speed


def write_preflight(directory, args, planned=None, fault=None):
    report = {"ok": fault is None, "network": args.network, "domain": args.domain,
              "published_commands": 0, "fault": fault, "planned": planned}
    (directory / "preflight.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def prepare(initial, initial_height, raw, arm, codec, profile, motion, limits, offset_m,
            fast_preflight=False):
    lo, hi, effort = [np.array([j[key] for j in raw["joints"]])
                      for key in ("lower", "upper", "effort")]
    held = {k: v for k, v in raw["held"].items() if k != "note"}
    codec.hold_targets = initial.hold_q.copy()
    held.update(zip(codec.hold_names, codec.hold_targets))
    controller = Controller(initial.q, motion, lo, hi, codec.kp, codec.kd, effort,
                            limits=limits, expected_mode_machine=initial.mode_machine,
                            scene=SCENE_SIDE_SYNC)
    geometry = Geometry(held, offset_m)
    wall_axis = int(profile.get("wall_axis", 1))
    expected_wall = profile["y_m"] + profile["standoff_m"]
    if wall_axis != geometry.wall_axis or not np.isclose(geometry.wall_pos, expected_wall, atol=1e-6):
        raise ControlFault("侧向升降模型与 600 mm 轨迹的箱面位置不一致")
    space = Clearance(geometry.kin, lo + limits.margin, hi - limits.margin,
                      geometry.wall_pos, held, wall_axis=wall_axis)
    space.base = [geometry.plate, *geometry.tube_geoms]
    lift_entry = planned_lift_entry(initial.q, initial_height, geometry, space)
    geometry.set_height(LOW_M)
    stow = np.radians([raw["stow_deg"][name] for name in arm.names()])
    typical = np.array([raw["initial"].get(name, 0.0) for name in arm.names()])
    approach = plan(space, initial.q, controller.q_scan_start, "接近扫描线",
                    via=(typical, stow))
    retract = plan(space, motion.sample(motion.duration), stow, "竖直收纳",
                   via=(typical, initial.q))
    ready_seconds = float(profile.get("scan_ready_hold_s", 0.8))
    controller.set_transfers(approach, retract, ready_seconds=ready_seconds)
    preflight_kwargs = {"sample_dt": .005, "fk_dt": .020} if fast_preflight else {}
    check = preflight.run(controller, motion, profile, limits, geometry.kin,
                          clearance=space, **preflight_kwargs)
    if not check.ok:
        raise ControlFault(f"手臂预检失败: {check.violations[:5]}")
    planned = planned_geometry(controller, profile, geometry, space)
    planned.update(model_reference_offset_m=offset_m,
                   target_height_mm=[LOW_M * 1000, HIGH_M * 1000],
                   lift_entry=lift_entry,
                   arm_approach_waypoints_deg=np.degrees(approach).round(2).tolist(),
                   arm_stow_waypoints_deg=np.degrees(retract).round(2).tolist(),
                   scan_one_way_s=motion.one_way_duration)
    controller.complete = False
    return controller, geometry, space, planned


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain", type=int)
    parser.add_argument("--output", type=Path, default=ROOT / "out" / "sync")
    parser.add_argument("--dry-run", action="store_true", help="只读反馈并预检，零运动指令")
    parser.add_argument("--model-offset-mm", type=float, default=SIM_ZERO_GAP_M * 1000,
                        help="DDS 零位对应的模型机身间隙，仅用于碰撞预检与报告")
    args = parser.parse_args()
    real = args.network != "lo"
    if args.domain is None:
        args.domain = 0 if real else 217
    offset = args.model_offset_mm / 1000
    if (not np.isfinite(offset) or LOW_M + offset - ENTRY_TOL_M < 0.40
            or HIGH_M + offset + ENTRY_TOL_M > 0.96):
        parser.error("模型高度偏移须使 400–800 mm 的目标落在 0.40–0.96 m 的模型范围内")
    directory = args.output
    directory.mkdir(parents=True, exist_ok=True)

    profile, motion = load_motion(PROFILE)
    raw = json.loads((ROOT / "r1d_scan" / "robot_profile.json").read_text())
    arm = load_profile()
    if real and arm.gaps():
        raise SystemExit("真机联合扫描配置未就绪：" + "；".join(arm.gaps()))
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
                complete = not any(value is None
                                   for value in (state, lift.height, lift.alarm, lift.velocity))
                stable = (complete and now - state.received <= limits.state_timeout
                          and np.max(np.abs(state.dq)) <= .10
                          and abs(lift.velocity[1]) <= ENDPOINT_SPEED_M_S)
                if stable:
                    stable_since = stable_since or now
                    if now - stable_since >= .25:
                        break
                else:
                    stable_since = None
                if time.monotonic() >= deadline:
                    raise ControlFault("等待手臂和立柱的完整静止反馈超时")
                time.sleep(0.02)
            initial = state
            initial_height, _ = verify_start(initial, initial, lift, limits)
            c, geometry, space, planned = prepare(initial, initial_height, raw, arm, codec,
                                                  profile, motion, limits, offset,
                                                  fast_preflight=real)
            verify_start(state, initial, lift, limits)
            if args.dry_run:
                write_preflight(directory, args, planned)
                print(f"联合预检通过，发布运动指令 0 条。报告 {directory / 'preflight.json'}",
                      flush=True)
                return 0

            if real:
                print(f"联合预检通过：回车后立柱先从 {initial_height * 1000:.1f} mm "
                      "自动归位到 400 mm；随后手臂移动到 600 mm 扫描下端点，"
                      "正式上、下扫时立柱同步 400→800→400 mm。", flush=True)
                if not sys.stdin.isatty():
                    raise ControlFault("真机联动必须在交互终端运行并由操作者回车确认")
                try:
                    answer = input("确认运动范围清空、急停可用后，直接按回车启动（其他输入取消）：")
                except (EOFError, KeyboardInterrupt):
                    raise ControlFault("操作者取消")
                if answer != "":
                    raise ControlFault("操作者取消")
                verify_start(state, initial, lift, limits)

            time.sleep(max(0.0, watch_end - time.monotonic()))
            if competing:
                raise ControlFault(f"检测到其他控制发布者：{', '.join(sorted(competing))}")
            arm_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
            arm_publisher.Init()
            channels.callback(arm_publisher.Close)
            lift.open_publisher(channels)
            watchers.close()
            if competing:
                raise ControlFault(f"检测到其他控制发布者：{', '.join(sorted(competing))}")
            verify_start(state, initial, lift, limits)
            write_preflight(directory, args, planned)
        except ControlFault as exc:
            write_preflight(directory, args, fault=str(exc))
            print(f"联合预检未通过，发布运动指令 0 条。{exc}", flush=True)
            return 2

        try:
            entry = position_lift_at_low(c, codec, arm_publisher, lift, lambda: state)
            planned["lift_entry_result"] = entry
            verify_start(state, initial, lift, limits, require_low=True, pose_tolerance_deg=3.0)
            geometry.set_height(LOW_M)
            issue = space.issue(state.q)
            plate_gap, tube_gap = geometry.base_clearances()
            if issue or plate_gap <= 0 or tube_gap <= 0:
                raise ControlFault("立柱自动归位后的实测手臂姿态未通过组合碰撞复核："
                                   + (issue or "底座或升降管间隙不足"))
            entry["post_entry_arm_max_delta_deg"] = round(float(np.degrees(
                np.max(np.abs(state.q - initial.q)))), 3)
            entry["post_entry_min_arm_to_base_plate_m"] = float(plate_gap)
            entry["post_entry_min_arm_to_lift_tube_m"] = float(tube_gap)
            (directory / "lift_entry.json").write_text(
                json.dumps(entry, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            print(f"立柱已自动归位并停稳：{entry['start_height_mm']:.1f} → "
                  f"{entry['end_height_mm']:.1f} mm", flush=True)
        except (ControlFault, KeyboardInterrupt) as exc:
            reason = str(exc) if isinstance(exc, ControlFault) else "操作者中止立柱自动归位"
            payload = {"complete": False, "fault": reason,
                       "target_height_mm": LOW_M * 1000}
            (directory / "lift_entry.json").write_text(
                json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            print(f"立柱自动归位失败，已发送零速指令：{reason}", flush=True)
            return 2

        print(f"组合预检通过，DDS 立柱目标 400→800→400 mm：{planned}", flush=True)
        runner = SyncRunner(c, codec, arm_publisher, arm, lift=lift,
                            log_path=directory / "arm.csv", period=.005).open_log()
        channels.callback(runner.close)
        fault = None
        with ExitStack() as execution:
            def terminate(_signum, _frame):
                raise KeyboardInterrupt

            previous = signal.signal(signal.SIGTERM, terminate)
            execution.callback(signal.signal, signal.SIGTERM, previous)
            lift.start(execution)
            c.previous = time.monotonic()
            c.start = c.previous + c.limits.startup_hold
            try:
                runner.run(lambda: state)
            except ControlFault as exc:
                fault = str(exc)
            except KeyboardInterrupt:
                fault = "操作者中止联动扫描"
        if fault and lift.fault and fault != lift.fault:
            fault = f"{fault}；{lift.fault}"
        else:
            fault = fault or lift.fault
        save_report(directory, runner, geometry, profile, planned, fault, args.network, args.domain)
        return 2 if fault else 0


if __name__ == "__main__":
    raise SystemExit(main())
