"""MuJoCo DDS 仿真器：订阅 rt/lowcmd，按 PD 力矩积分物理，发布 rt/lowstate。"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

import mujoco
import numpy as np

from dds_network import initialize
from profile import load as load_profile

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "models" / "r1_a7_scene.xml"
SCENE_MOBILE = ROOT / "models" / "r1_a7_scene_mobile.xml"
SCENE_LIFT = ROOT / "models" / "r1_a7_scene_lift.xml"


class PhysicsBridge:
    def __init__(self, scene=SCENE, mode_machine=1, command_timeout=0.2, servo_hold=False,
                 mobile=False, lift=False, sync_lift=False):
        self.scene = Path(scene)
        self.profile = load_profile()
        self.joints = [j.name for j in self.profile.joints]
        self.slots = np.array([j.slot for j in self.profile.joints], int)
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.jids = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                              for n in self.joints])
        self.aids = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                                n.replace("_joint", "")) for n in self.joints])
        if np.any(self.jids < 0) or np.any(self.aids < 0):
            raise SystemExit(f"场景缺少关节或执行器: {self.joints}")
        self.qadr = self.model.jnt_qposadr[self.jids]
        self.dadr = self.model.jnt_dofadr[self.jids]
        self.ctrlrange = self.model.actuator_ctrlrange[self.aids]
        # 保持关节（右臂）：仿真里默认由等式约束锁住；servo_hold 时改成 PD 保持，
        # 用来验证"真机上每拍写固定值"这条路到底顶不顶得住。
        hold = [j for j in getattr(self.profile, "hold_joints", ())
                if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j.name) >= 0]
        self.hold_names = [j.name for j in hold]
        self.hold_slots = np.array([j.slot for j in hold], int)
        self.hold_jids = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                                   for n in self.hold_names], int)
        self.servo_hold = bool(servo_hold) and bool(self.hold_names)
        if self.servo_hold:
            self.hold_aids = np.array([mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                                                         n.replace("_joint", ""))
                                       for n in self.hold_names], int)
            self.hold_qadr = self.model.jnt_qposadr[self.hold_jids]
            self.hold_dadr = self.model.jnt_dofadr[self.hold_jids]
            if np.any(self.hold_aids < 0):
                missing = [n for n, a in zip(self.hold_names, self.hold_aids) if a < 0]
                raise SystemExit(f"场景没有保持关节的执行器: {missing}；"
                                 "先跑 python3 tools/build_scene.py 重建场景")
            self.hold_ctrlrange = self.model.actuator_ctrlrange[self.hold_aids]
            disabled = 0
            for jid in self.hold_jids:
                for eq in range(self.model.neq):
                    if (self.model.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT
                            and self.model.eq_obj1id[eq] == jid):
                        # MuJoCo 3.x：开关在 data.eq_active（model 里是初值 eq_active0）
                        self.data.eq_active[eq] = 0
                        disabled += 1
            print(f"保持模式: servo（关掉 {disabled} 个等式约束，用 PD 顶住右臂）", flush=True)
        # ---- 底盘（运动学模型）----
        # 场景在 --mobile 下给根 body 加了自由关节；这里每步把速度写进去、把姿态投影回水平。
        # 这不是"轮子带摩擦的物理底盘"：轮径/轮距/质量都还没实测，先保证接口和动作对。
        self.mobile = bool(mobile)
        self.mount_qpos = self.mount_dof = None
        if self.mobile:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "mount_free")
            if jid < 0:
                raise SystemExit("场景没有 mount_free 自由关节；"
                                 "先跑 python3 tools/build_scene.py --mobile")
            self.mount_qpos = int(self.model.jnt_qposadr[jid])
            self.mount_dof = int(self.model.jnt_dofadr[jid])
            self.base_z = float(self.data.qpos[self.mount_qpos + 2])
        # ---- 升降柱（滑动关节，运动学驱动）----
        # 真机立柱是螺杆驱动，很硬；这里每步写速度、到限位就夹住，不做"载重压回去"的动力学。
        # 速度上限 76.5 mm/s 是官方写在 HeightAdjust 注释里的，不是猜的。
        self.lift = bool(lift)
        self.sync_lift = sync_lift
        self.lift_speed = 0.0
        self.lift_qpos = self.lift_dof = None
        self.lift_visual_mocap = -1
        self.lift_range = (0.0, 0.0)
        self.height_cmd = 0.0
        self.height_at = None
        self.height_timeout = 0.5
        self.height_travel = 0.0
        self.column_pos = 0.0
        self.height_limit_hits = 0
        self._last_height = 0.0
        self.column_max_m_s = 0.0765
        if self.lift:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "lift_slide")
            if jid < 0:
                raise SystemExit("场景没有 lift_slide 滑动关节；"
                                 "先跑 python3 tools/build_scene.py --lift")
            self.lift_qpos = int(self.model.jnt_qposadr[jid])
            self.lift_dof = int(self.model.jnt_dofadr[jid])
            self.lift_range = (float(self.model.jnt_range[jid][0]),
                               float(self.model.jnt_range[jid][1]))
            self.column_pos = float(self.data.qpos[self.lift_qpos])
            visual = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "lift_middle_visual")
            if visual >= 0:
                self.lift_visual_mocap = int(self.model.body_mocapid[visual])
                self.lift_visual_nominal_z = float(self.model.body_pos[visual, 2])
            # 标称间隙：关节 q=0 时，上半身离底盘板上表面多少米（场景常数）
            self.lift_nominal_gap = 0.84
            manifest = ROOT / "models" / "simulation_lift.json"
            if manifest.exists():
                import json as _json
                self.lift_nominal_gap = float(
                    _json.loads(manifest.read_text(encoding="utf-8")).get(
                        "lift_nominal_gap_m", self.lift_nominal_gap))
            cfg_path = ROOT / "models" / "chassis.json"
            if cfg_path.exists():
                import json as _json
                height = _json.loads(cfg_path.read_text(encoding="utf-8")).get("height", {})
                self.column_max_m_s = float(height.get("column_max_m_s", self.column_max_m_s))

        if self.sync_lift:
            from lift_dds import LOW_GAP_M
            self.column_pos = LOW_GAP_M - self.lift_nominal_gap
            self._clamp_lift()
            wall = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "wall")
            # 旧正面模型的墙法向为 X；侧向模型法向为 Y，不应改写它的 X 坐标。
            if wall >= 0 and self.model.geom_size[wall, 0] < .10:
                self.model.geom_pos[wall, 0] = 0.64 + self.model.geom_size[wall, 0]

        self.chassis_cmd = (0.0, 0.0, 0.0)
        self.chassis_at = None
        self.chassis_timeout = 0.5     # 服务端自己的兜底：这么久没新指令就按零速算
        self.base_pose = (0.0, 0.0, 0.0)
        self.base_travel = 0.0
        self._last_xy = None
        self.mode_machine = mode_machine
        self.command_timeout = command_timeout
        self.command = None
        self.accepted = 0
        self.rejected = 0
        self.last_reject = ""
        self.steps = 0
        self.active_steps = 0
        self.timed_out = False
        self.hold_max_dev = 0.0
        self.initial_q = self.data.qpos[self.qadr].copy()
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.utils.crc import CRC
        self.state_factory = unitree_hg_msg_dds__LowState_
        self.crc = CRC()
        mujoco.mj_forward(self.model, self.data)

    def apply_pose(self, pose):
        """把关节角写进初始状态（例如双臂自然下垂）。"""
        for name, angle in pose.items():
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self.data.qpos[self.model.jnt_qposadr[jid]] = float(angle)
        mujoco.mj_forward(self.model, self.data)

    def accept(self, msg, now=None):
        from transport import Codec
        try:
            if not hasattr(self, "_codec"):
                self._codec = Codec(profile=self.profile)
            if int(msg.crc) != self._codec.crc.Crc(msg):
                raise ValueError("CRC 错误")
            if int(msg.mode_pr) != 0 or int(msg.mode_machine) != self.mode_machine:
                raise ValueError(f"模式不匹配 mode_pr={msg.mode_pr} mode_machine={msg.mode_machine}")
            q = np.array([msg.motor_cmd[int(s)].q for s in self.slots], float)
            dq = np.array([msg.motor_cmd[int(s)].dq for s in self.slots], float)
            tau = np.array([msg.motor_cmd[int(s)].tau for s in self.slots], float)
            kp = np.array([msg.motor_cmd[int(s)].kp for s in self.slots], float)
            kd = np.array([msg.motor_cmd[int(s)].kd for s in self.slots], float)
            hold = None
            if self.servo_hold:
                hold = {key: np.array([getattr(msg.motor_cmd[int(s)], key) for s in self.hold_slots], float)
                        for key in ("q", "dq", "tau", "kp", "kd")}
            if not np.isfinite(np.concatenate((q, dq, tau, kp, kd))).all():
                raise ValueError("指令含非有限值")
            if hold is not None and not all(np.isfinite(v).all() for v in hold.values()):
                raise ValueError("保持关节指令含非有限值")
            self.command = {"q": q, "dq": dq, "tau": tau, "kp": kp, "kd": kd, "hold": hold,
                            "received": time.monotonic() if now is None else now}
            self.accepted += 1
            return True
        except Exception as exc:  # noqa: BLE001
            self.rejected += 1
            self.last_reject = str(exc)
            return False

    def set_height(self, vz_normalized, now=None):
        """agv 服务的 HeightAdjust 落在这里。vz 是**归一化** [-1, 1]，不是 m/s。"""
        if not self.lift:
            return False
        self.height_cmd = max(-1.0, min(1.0, float(vz_normalized)))
        self.height_at = time.monotonic() if now is None else now
        return True

    def _apply_lift(self, now):
        """运动学驱动：位置自己积分，然后每步把 qpos 写回去、速度清零。

        为什么不能"设 qvel 让物理去积分"：立柱是一根竖直滑轨，重力在每个 1 ms 步里
        会吃掉 g·dt = 9.81 mm/s 的速度——设 38.25 mm/s 实际只走 28.4 mm/s，
        停下时还会以约 1 cm/s 往下滑。真机是螺杆传动、几乎不回落，所以这里按运动学处理：
        **速度由指令决定，与重力无关**。代价同样明确：载重不会把立柱压回去。
        """
        lo, hi = self.lift_range
        dt = float(self.model.opt.timestep)
        vz = self.height_cmd
        if self.height_at is None or now - self.height_at > self.height_timeout:
            vz = 0.0
        speed = vz * self.column_max_m_s
        if self.sync_lift:
            # 小的一阶滞后用于验证反馈跟踪。不是实机辨识参数。
            self.lift_speed += (speed - self.lift_speed) * min(1.0, dt / 0.06)
            speed = self.lift_speed
        target = self.column_pos + speed * dt
        clamped = max(lo, min(hi, target))
        if clamped != target and abs(vz) > 0:
            self.height_limit_hits += 1
        delta = clamped - self.column_pos
        self.height_travel += abs(delta)
        self.column_pos = clamped
        self._clamp_lift()

    def _clamp_lift(self):
        """步进后再写一次：抹掉重力在这一步里造成的一点点位移。"""
        self.data.qpos[self.lift_qpos] = self.column_pos
        self.data.qvel[self.lift_dof] = 0.0
        if self.lift_visual_mocap >= 0:
            self.data.mocap_pos[self.lift_visual_mocap, 2] = (
                self.lift_visual_nominal_z + 0.5 * self.column_pos)

    def set_chassis(self, vx, vy, vyaw, now=None):
        """agv 服务的 Move 落在这里：只记目标速度，物理步里再执行。"""
        if not self.mobile:
            return False
        self.chassis_cmd = (float(vx), float(vy), float(vyaw))
        self.chassis_at = time.monotonic() if now is None else now
        return True

    def _apply_chassis(self, now):
        """把目标速度写进自由关节，并把底座投影回水平面（运动学底盘）。"""
        import math
        vx, vy, wz = self.chassis_cmd
        if self.chassis_at is None or now - self.chassis_at > self.chassis_timeout:
            vx = vy = wz = 0.0
        q = self.data.qpos
        yaw = math.atan2(2.0 * (q[self.mount_qpos + 3] * q[self.mount_qpos + 6]
                                + q[self.mount_qpos + 4] * q[self.mount_qpos + 5]),
                         1.0 - 2.0 * (q[self.mount_qpos + 5] ** 2 + q[self.mount_qpos + 6] ** 2))
        # 机体速度 -> 世界速度
        wx = vx * math.cos(yaw) - vy * math.sin(yaw)
        wy = vx * math.sin(yaw) + vy * math.cos(yaw)
        d = self.mount_dof
        self.data.qvel[d + 0] = wx
        self.data.qvel[d + 1] = wy
        self.data.qvel[d + 2] = 0.0
        self.data.qvel[d + 3] = 0.0
        self.data.qvel[d + 4] = 0.0
        self.data.qvel[d + 5] = wz

    def _project_base(self):
        """积分后把底座拉回水平、高度锁死。手臂的反作用力因此不会推着底盘跑，
        这是"运动学底盘"的代价，也是阶段 1 刻意选的简化。"""
        import math
        a = self.mount_qpos
        d = self.mount_dof
        yaw = math.atan2(2.0 * (self.data.qpos[a + 3] * self.data.qpos[a + 6]
                                + self.data.qpos[a + 4] * self.data.qpos[a + 5]),
                         1.0 - 2.0 * (self.data.qpos[a + 5] ** 2 + self.data.qpos[a + 6] ** 2))
        self.data.qpos[a + 2] = self.base_z
        self.data.qpos[a + 3] = math.cos(yaw / 2.0)
        self.data.qpos[a + 4] = 0.0
        self.data.qpos[a + 5] = 0.0
        self.data.qpos[a + 6] = math.sin(yaw / 2.0)
        self.data.qvel[d + 2] = 0.0
        self.data.qvel[d + 3] = 0.0
        self.data.qvel[d + 4] = 0.0
        xy = (float(self.data.qpos[a]), float(self.data.qpos[a + 1]))
        if self._last_xy is not None:
            self.base_travel += math.hypot(xy[0] - self._last_xy[0], xy[1] - self._last_xy[1])
        self._last_xy = xy
        self.base_pose = (xy[0], xy[1], yaw)

    def step(self, now):
        self.steps += 1
        command = self.command
        if command is None:
            # 还没收到指令：保持初始姿态，不积分。
            # 例外：移动底盘要能自己动（只测底盘时手臂指令一条都没有），所以仍然推进一步。
            if self.sync_lift:
                self._apply_lift(now)
                self.data.time += self.model.opt.timestep
                mujoco.mj_forward(self.model, self.data)
                return
            if self.mobile or self.lift:
                if self.mobile:
                    self._apply_chassis(now)
                if self.lift:
                    self._apply_lift(now)
                mujoco.mj_step(self.model, self.data)
                if self.mobile:
                    self._project_base()
                if self.lift:
                    self._clamp_lift()
                self.active_steps += 1
                return
            self.data.time += self.model.opt.timestep
            mujoco.mj_forward(self.model, self.data)
            return
        self.timed_out = now - command["received"] > self.command_timeout
        q, dq = self.data.qpos[self.qadr], self.data.qvel[self.dadr]
        target = np.clip(command["q"], self.model.jnt_range[self.jids, 0],
                         self.model.jnt_range[self.jids, 1])
        tau = command["tau"] + command["kp"] * (target - q) + command["kd"] * (command["dq"] - dq)
        self.data.ctrl[self.aids] = np.clip(tau, self.ctrlrange[:, 0], self.ctrlrange[:, 1])
        if self.servo_hold and command.get("hold") is not None:
            hold = command["hold"]
            hq = self.data.qpos[self.hold_qadr]
            hdq = self.data.qvel[self.hold_dadr]
            htarget = np.clip(hold["q"], self.model.jnt_range[self.hold_jids, 0],
                              self.model.jnt_range[self.hold_jids, 1])
            htau = hold["tau"] + hold["kp"] * (htarget - hq) + hold["kd"] * (hold["dq"] - hdq)
            self.data.ctrl[self.hold_aids] = np.clip(htau, self.hold_ctrlrange[:, 0],
                                                     self.hold_ctrlrange[:, 1])
            self.hold_max_dev = max(self.hold_max_dev,
                                    float(np.max(np.abs(hq - hold["q"]))))
        if self.mobile:
            self._apply_chassis(now)
        if self.lift:
            self._apply_lift(now)
        mujoco.mj_step(self.model, self.data)
        if self.mobile:
            self._project_base()
        if self.lift:
            self._clamp_lift()
        self.active_steps += 1

    def state(self):
        # 不再调用 mj_forward：step() 里的 mj_step / mj_forward 已经把 qpos/qvel/
        # qacc/qfrc_actuator 都算好了，每帧再算一遍纯属浪费（2 核虚拟机上这很致命）。
        state = self.state_factory()
        state.mode_pr = 0
        state.mode_machine = self.mode_machine
        state.tick = int(round(self.steps * self.model.opt.timestep * 1000)) % 2**32
        for k, slot in enumerate(self.slots):
            motor = state.motor_state[int(slot)]
            motor.mode = 1
            motor.q = float(self.data.qpos[self.qadr[k]])
            motor.dq = float(self.data.qvel[self.dadr[k]])
            motor.ddq = float(self.data.qacc[self.dadr[k]])
            motor.tau_est = float(self.data.qfrc_actuator[self.dadr[k]])
            motor.temperature = [30, 30]
            motor.vol = 48.0
        # 保持关节也要报实测值，否则控制端"照原样写回"就写成了 0°
        for k, slot in enumerate(self.hold_slots):
            motor = state.motor_state[int(slot)]
            motor.mode = 1
            motor.q = float(self.data.qpos[self.model.jnt_qposadr[self.hold_jids[k]]])
            motor.dq = float(self.data.qvel[self.model.jnt_dofadr[self.hold_jids[k]]])
            motor.tau_est = float(self.data.qfrc_actuator[self.model.jnt_dofadr[self.hold_jids[k]]])
            motor.temperature = [30, 30]
            motor.vol = 48.0
        state.crc = self.crc.Crc(state)
        return state

    def report(self):
        hold_q = [round(float(self.data.qpos[self.model.jnt_qposadr[j]]), 4) for j in self.hold_jids]
        return {"scene": str(self.scene), "mode_machine": self.mode_machine,
                "hold_mode": "servo" if self.servo_hold else "constraint",
                "hold_joints": self.hold_names,
                "final_hold_q_rad": hold_q,
                "hold_max_dev_deg": round(float(np.degrees(self.hold_max_dev)), 2),
                "commands_accepted": self.accepted, "commands_rejected": self.rejected,
                "last_reject": self.last_reject, "physics_steps": self.active_steps,
                "steps_including_idle": self.steps, "command_timed_out": self.timed_out,
                "final_q_rad": self.data.qpos[self.qadr].round(4).tolist(),
                **({"base_mode": "mobile",
                    "base_pose": [round(v, 4) for v in self.base_pose],
                    "base_travel_m": round(self.base_travel, 4),
                    "chassis_cmd": [round(v, 3) for v in self.chassis_cmd]}
                   if self.mobile else
                   ({"base_mode": "lift",
                     "column_height_m": round(self.column_pos, 4),
                     "column_gap_m": round(self.column_pos + self.lift_nominal_gap, 4),
                     "column_range_m": [round(v, 4) for v in self.lift_range],
                     "column_travel_m": round(self.height_travel, 4),
                     "column_limit_hits": self.height_limit_hits,
                     "column_max_m_s": self.column_max_m_s}
                    if self.lift else {"base_mode": "fixed"}))}


class FeedbackDrop:
    """按联动控制阶段暂停指定升降反馈话题，仅用于本机回环诊断。"""

    def __init__(self, topics, phase, duration_ms, started):
        self.topics = tuple(topics or ())
        self.phase = phase
        self.duration_s = float(duration_ms) / 1000.0
        self.started = started
        self.drop_started = None
        self.drop_ended = None
        self.phase_match_started = None
        # 立柱在下扫末端约提前 0.38 s 进入近零速；等待 0.5 s，确保控制器
        # 已从 scan-down 切到 after-scan（手臂收纳），避免把末端减速误标为收纳。
        self.trigger_delay_s = 0.500 if phase == "after-scan" else 0.0
        self.dropped_frames = {name: 0 for name in self.topics}

    def skip(self, topic, now, phase, armed):
        if not self.topics:
            return False
        if self.drop_started is None:
            if not armed or phase != self.phase:
                self.phase_match_started = None
            else:
                self.phase_match_started = self.phase_match_started or now
                if now - self.phase_match_started >= self.trigger_delay_s:
                    self.drop_started = now
                    self.drop_ended = now + self.duration_s
                    print("FEEDBACK_DROP_START "
                          f"topics={','.join(self.topics)} phase={phase} "
                          f"duration_ms={self.duration_s * 1000:.1f} "
                          f"t={now - self.started:.6f}", flush=True)
        active = (self.drop_started is not None and now < self.drop_ended)
        if active and topic in self.topics:
            self.dropped_frames[topic] += 1
            return True
        return False

    def report(self, now):
        actual_ms = None
        if self.drop_started is not None:
            actual_ms = 1000.0 * (min(now, self.drop_ended) - self.drop_started)
        return {
            "requested_topics": list(self.topics),
            "control_phase": self.phase,
            "requested_duration_ms": round(self.duration_s * 1000.0, 3),
            "trigger_delay_ms": round(self.trigger_delay_s * 1000.0, 3),
            "triggered": self.drop_started is not None,
            "start_t_s": (None if self.drop_started is None
                          else round(self.drop_started - self.started, 6)),
            "actual_duration_ms": None if actual_ms is None else round(actual_ms, 3),
            "dropped_frames": dict(self.dropped_frames),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--scene", type=Path, help="固定立柱的仿真模型文件")
    parser.add_argument("--duration", type=float, default=0.0, help="墙钟运行秒数，0=一直跑")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "sim.json")
    parser.add_argument("--trace", action="store_true", help="每 0.5 s 打印第一个关节的目标/实际/力矩")
    parser.add_argument("--mobile", action="store_true",
                        help="底盘可走：用 r1_a7_scene_mobile.xml，托管 agv 的 Move")
    parser.add_argument("--lift", action="store_true",
                        help="升降柱可伸缩：用 r1_a7_scene_lift.xml，托管 agv 的 HeightAdjust")
    parser.add_argument("--hold", choices=("constraint", "servo"), default="constraint",
                        help="右臂保持方式：constraint=等式约束（默认，仿真专用）；"
                             "servo=PD 保持（与真机一致，需要每拍收到固定值指令）")
    parser.add_argument("--sync-lift", action="store_true",
                        help="联动仿真：只允许 lo，使用 R1-A7-D 升降 DDS 话题")
    parser.add_argument("--initial-lift-height-mm", type=float,
                        help="仅联动仿真：覆盖初始 DDS 高度，用于验证自动归位")
    parser.add_argument("--drop-feedback-topics", nargs="+",
                        choices=("height", "velocity", "alarm"),
                        help="仅联动回环：暂停一个或多个升降反馈话题")
    parser.add_argument("--drop-feedback-phase",
                        choices=("lift-entry", "before-scan", "scan-up", "scan-down",
                                 "after-scan"),
                        help="仅联动回环：开始暂停反馈的控制阶段")
    parser.add_argument("--drop-feedback-duration-ms", type=float, default=0.0,
                        help="仅联动回环：暂停反馈的持续时间（ms）")
    args = parser.parse_args()
    if args.sync_lift:
        if args.network != "lo" or args.mobile:
            parser.error("--sync-lift 仅支持 lo 上的固定轮式底盘")
        args.lift = True
    drop_requested = bool(args.drop_feedback_topics or args.drop_feedback_phase
                          or args.drop_feedback_duration_ms)
    if drop_requested:
        if not args.sync_lift:
            parser.error("反馈话题中断只支持 --sync-lift 本机回环")
        if (not args.drop_feedback_topics or not args.drop_feedback_phase
                or not np.isfinite(args.drop_feedback_duration_ms)
                or args.drop_feedback_duration_ms <= 0):
            parser.error("反馈中断须同时指定话题、控制阶段和正数持续时间")

    if args.scene and (args.network != "lo" or args.mobile or (args.lift and not args.sync_lift)):
        parser.error("--scene 只可用于本机回环的固定底盘或升降 DDS 仿真")
    scene = args.scene or (SCENE_LIFT if args.lift else (SCENE_MOBILE if args.mobile else SCENE))
    bridge = PhysicsBridge(scene=scene, servo_hold=(args.hold == "servo"),
                           mobile=args.mobile, lift=args.lift, sync_lift=args.sync_lift)
    raw = json.loads((ROOT / "r1d_scan" / "robot_profile.json").read_text(encoding="utf-8"))
    pose = {k: v for k, v in raw.get("held", {}).items() if k != "note"}
    pose.update({k: v for k, v in raw.get("initial", {}).items() if k != "note"})
    bridge.apply_pose(pose)
    if args.initial_lift_height_mm is not None:
        if not args.sync_lift or not np.isfinite(args.initial_lift_height_mm):
            parser.error("--initial-lift-height-mm 只接受联动仿真的有限数值")
        from lift_dds import SIM_ZERO_GAP_M
        bridge.column_pos = (SIM_ZERO_GAP_M + args.initial_lift_height_mm / 1000.0
                             - bridge.lift_nominal_gap)
        bridge._clamp_lift()
    _pose_now = {name: round(float(np.degrees(bridge.data.qpos[
        bridge.model.jnt_qposadr[mujoco.mj_name2id(bridge.model, mujoco.mjtObj.mjOBJ_JOINT, name)]])), 1)
        for name in ("left_shoulder_pitch_joint", "left_elbow_joint",
                     "right_shoulder_pitch_joint", "right_elbow_joint")
        if mujoco.mj_name2id(bridge.model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0}
    print(f"初始姿态(deg): {_pose_now}", flush=True)
    from hashlib import sha256
    print(f"场景: {bridge.scene.name} sha256={sha256(bridge.scene.read_bytes()).hexdigest()[:8]} "
          f"构建时间={time.strftime('%H:%M:%S', time.localtime(bridge.scene.stat().st_mtime))}", flush=True)
    initialize(args.network, args.domain)
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    publisher = ChannelPublisher("rt/lowstate", LowState_)
    publisher.Init()
    subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
    subscriber.Init(bridge.accept, 1)

    lift_channels = []
    feedback_phase = {"name": "before-scan", "command_seen": False,
                      "entry_active": False, "scan_up": False, "scan_down": False}
    if args.sync_lift:
        from lift_dds import (ALARM_TOPIC, CMD_TOPIC, HEIGHT_TOPIC, VELOCITY_TOPIC,
                              SIM_ZERO_GAP_M, Error_, Point32_, Twist_, encode_height,
                              motion_twist)

        base_state = {"forward": 0.0}

        def on_lift_command(msg):
            values = (msg.linear.x, msg.linear.y, msg.linear.z,
                      msg.angular.x, msg.angular.y, msg.angular.z)
            if (not np.isfinite(values).all() or any(values[i] != 0 for i in (1, 3, 4, 5))
                    or abs(float(msg.linear.x)) > 1.0 or abs(float(msg.linear.z)) > 1.0
                    or (abs(float(msg.linear.x)) > 1e-6 and abs(float(msg.linear.z)) > 1e-6)):
                bridge.set_height(0.0)
                base_state["forward"] = 0.0
                return
            base_state["forward"] = float(msg.linear.x)
            command = float(msg.linear.z)
            gap = bridge.column_pos + bridge.lift_nominal_gap
            dds_height = gap - SIM_ZERO_GAP_M
            feedback_phase["command_seen"] = True
            epsilon = 1e-4
            if abs(base_state["forward"]) > epsilon:
                feedback_phase["name"] = "column-shift"
            elif not feedback_phase["scan_up"]:
                if feedback_phase["entry_active"]:
                    if abs(command) <= epsilon:
                        feedback_phase["entry_active"] = False
                        feedback_phase["name"] = "before-scan"
                    else:
                        feedback_phase["name"] = "lift-entry"
                elif abs(command) <= epsilon:
                    feedback_phase["name"] = "before-scan"
                elif abs(dds_height - 0.400) > 0.010 or command < 0:
                    feedback_phase["entry_active"] = True
                    feedback_phase["name"] = "lift-entry"
                else:
                    feedback_phase["scan_up"] = True
                    feedback_phase["name"] = "scan-up"
            elif command < -epsilon:
                feedback_phase["scan_down"] = True
                feedback_phase["name"] = "scan-down"
            elif command > epsilon and not feedback_phase["scan_down"]:
                feedback_phase["name"] = "scan-up"
            elif feedback_phase["scan_down"] and dds_height <= 0.415:
                feedback_phase["name"] = "after-scan"
            bridge.set_height(command)

        lift_reader = ChannelSubscriber(CMD_TOPIC, Twist_)
        lift_reader.Init(on_lift_command, 1)
        height_pub = ChannelPublisher(HEIGHT_TOPIC, Point32_)
        alarm_pub = ChannelPublisher(ALARM_TOPIC, Error_)
        velocity_pub = ChannelPublisher(VELOCITY_TOPIC, Twist_)
        for channel in (height_pub, alarm_pub, velocity_pub):
            channel.Init()
        lift_channels = [lift_reader, height_pub, alarm_pub, velocity_pub]

    agv_server = None
    if (args.mobile or args.lift) and not args.sync_lift:
        from unitree_sdk2py.rpc.server import Server

        class AgvServer(Server):
            """扮演真机上的 AGV 固件：收 Move 改底盘速度，收 HeightAdjust 明确拒绝。"""

            def __init__(self, bridge):
                super().__init__("agv")
                self.bridge = bridge

            def Init(self):
                self._RegistHandler(1001, self.Move, 0)
                self._RegistHandler(1002, self.HeightAdjust, 0)
                self._SetApiVersion("1.0.0.1")

            def Move(self, parameter):
                p = json.loads(parameter or "{}")
                self.bridge.set_chassis(p.get("vx", 0.0), p.get("vy", 0.0), p.get("vyaw", 0.0))
                return 0, ""

            def HeightAdjust(self, parameter):
                p = json.loads(parameter or "{}")
                if not self.bridge.lift:
                    # 没开 --lift 时明确拒绝，而不是假装成功——"发了但没动"最难查。
                    return 2, "lift scene not enabled (start sim.py with --lift)"
                self.bridge.set_height(p.get("data", 0.0))
                return 0, ""

        agv_server = AgvServer(bridge)
        agv_server.Init()
        agv_server.Start(False)
        mode = "lift" if args.lift else "mobile"
        print(f"AGV 服务已托管: service=agv api=1.0.0.1 (Move=1001, HeightAdjust=1002) mode={mode}", flush=True)

    stop = {"value": False}
    previous = {}

    def handler(*_):
        stop["value"] = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, handler)

    viewer = None
    if not args.no_viewer:
        import importlib
        viewer = importlib.import_module("mujoco.viewer").launch_passive(bridge.model, bridge.data)
        viewer.cam.lookat[:] = [0.30, 0.0, 1.02]
        viewer.cam.distance = 2.4
        viewer.cam.azimuth = 270     # 实测：270 才是 +y 侧（机器人左侧），90 是右侧
        viewer.cam.elevation = -8
        if args.scene:
            viewer.cam.lookat[:] = [0.10, 0.25, 0.78]
            viewer.cam.azimuth = 145
            viewer.cam.elevation = -12
    start = time.monotonic()
    feedback_drop = FeedbackDrop(args.drop_feedback_topics, args.drop_feedback_phase,
                                 args.drop_feedback_duration_ms, start)
    physics_due = publish_due = trace_due = viewer_due = lift_due = start
    published = 0
    lift_feedback_due = start
    print(f"SIM_READY network={args.network} domain={args.domain} "
          f"joints={len(bridge.joints)} timeout={bridge.command_timeout}s "
          f"base={('lift' if args.lift else ('mobile' if args.mobile else 'fixed'))}", flush=True)
    try:
        while not stop["value"]:
            now = time.monotonic()
            if args.duration and now - start >= args.duration:
                break
            if now - physics_due > 0.25:
                raise SystemExit("物理循环落后墙钟 0.25 s")
            count = 0
            while now >= physics_due and count < 20:
                bridge.step(now)
                physics_due += bridge.model.opt.timestep
                count += 1
            if now >= publish_due:
                if not publisher.Write(bridge.state()):
                    raise SystemExit("LowState 发送失败")
                published += 1
                publish_due = now + 0.002
            if args.sync_lift and now >= lift_feedback_due:
                gap = bridge.column_pos + bridge.lift_nominal_gap
                messages = (
                    ("height", height_pub, encode_height(gap)),
                    ("alarm", alarm_pub, Error_(0, int(bridge.height_limit_hits > 0))),
                    ("velocity", velocity_pub,
                     motion_twist(base_state["forward"],
                                  bridge.lift_speed / bridge.column_max_m_s)),
                )
                feedback = tuple(
                    True if feedback_drop.skip(name, now, feedback_phase["name"],
                                                feedback_phase["command_seen"])
                    else channel.Write(message)
                    for name, channel, message in messages
                )
                if not all(feedback):
                    raise SystemExit("升降状态发送失败")
                lift_feedback_due = now + 0.02
            if args.lift and now - lift_due >= 0.5:
                # 测试脚本按 "HEIGHT <米>" 解析，格式别随便改。
                lift_due = now
                hi = bridge.lift_range[1]
                gap = bridge.column_pos + bridge.lift_nominal_gap
                print(f"HEIGHT {gap:+.4f} VZ {bridge.height_cmd:+.2f} "
                      f"HIT {bridge.height_limit_hits} "
                      f"GAP {bridge.lift_range[0] + bridge.lift_nominal_gap:+.3f}"
                      f"..{hi + bridge.lift_nominal_gap:+.3f}",
                      flush=True)
            if args.trace and now - trace_due >= 0.5:
                trace_due = now
                command = bridge.command
                target = float(command["q"][0]) if command else float("nan")
                def angle(name):
                    jid = mujoco.mj_name2id(bridge.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                    return float(np.degrees(bridge.data.qpos[bridge.model.jnt_qposadr[jid]])) if jid >= 0 else float("nan")
                print(f"TRACE t={now - start:6.2f} Lsp_tgt={target:+.3f} Lsp={angle('left_shoulder_pitch_joint'):+7.2f} "
                      f"Rsp={angle('right_shoulder_pitch_joint'):+7.2f} Relb={angle('right_elbow_joint'):+7.2f} "
                      f"Leq={float(bridge.data.qfrc_constraint[bridge.dadr[0]]):+6.1f} "
                      f"equality_ok={len(bridge.data.efc_type)}", flush=True)
            if viewer is not None and now >= viewer_due:
                if not viewer.is_running():
                    break
                viewer.sync()
                viewer_due = now + 0.05     # 20 Hz 足够看，50 Hz 在软件渲染下太吃 CPU
            delay = min(physics_due, publish_due) - time.monotonic()
            if delay > 0:
                time.sleep(min(delay, 0.002))
    finally:
        if viewer is not None:
            try:
                viewer.close()
            except Exception:  # noqa: BLE001
                pass
        for channel in lift_channels:
            channel.Close()
        subscriber.Close()
        publisher.Close()
        for sig, fn in previous.items():
            signal.signal(sig, fn)
        result = bridge.report()
        result.update({"published_states": published,
                       "wall_seconds": round(time.monotonic() - start, 2),
                       "feedback_phase": feedback_phase["name"],
                       "feedback_drop": feedback_drop.report(time.monotonic())})
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                               encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False), flush=True)
        # 带窗口跑时，解释器退出阶段销毁 GLFW/GL 上下文会段错误（退出码 139）。
        # 已定位：报告在崩溃之前就落盘了，崩溃发生在 main 返回之后的析构里。
        # 这里直接跳过析构退出，让 run_demo.sh 的收尾输出干净——否则那行
        # "Segmentation fault" 会让人以为仿真跑挂了。无窗口时保持正常退出，
        # 免得把其它收尾问题也一起藏掉。
        if viewer is not None:
            sys.stdout.flush()
            os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
