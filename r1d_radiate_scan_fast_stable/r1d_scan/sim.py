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


class PhysicsBridge:
    def __init__(self, scene=SCENE, mode_machine=1, command_timeout=0.2, servo_hold=False):
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

    def step(self, now):
        self.steps += 1
        command = self.command
        if command is None:
            # 还没收到指令：保持初始姿态，不积分
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
        mujoco.mj_step(self.model, self.data)
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
                "final_q_rad": self.data.qpos[self.qadr].round(4).tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0, help="墙钟运行秒数，0=一直跑")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "sim.json")
    parser.add_argument("--trace", action="store_true", help="每 0.5 s 打印第一个关节的目标/实际/力矩")
    parser.add_argument("--hold", choices=("constraint", "servo"), default="constraint",
                        help="右臂保持方式：constraint=等式约束（默认，仿真专用）；"
                             "servo=PD 保持（与真机一致，需要每拍收到固定值指令）")
    args = parser.parse_args()

    bridge = PhysicsBridge(servo_hold=(args.hold == "servo"))
    raw = json.loads((ROOT / "r1d_scan" / "robot_profile.json").read_text(encoding="utf-8"))
    pose = {k: v for k, v in raw.get("held", {}).items() if k != "note"}
    pose.update({k: v for k, v in raw.get("initial", {}).items() if k != "note"})
    bridge.apply_pose(pose)
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
    start = time.monotonic()
    physics_due = publish_due = trace_due = viewer_due = start
    published = 0
    print(f"SIM_READY network={args.network} domain={args.domain} "
          f"joints={len(bridge.joints)} timeout={bridge.command_timeout}s", flush=True)
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
        subscriber.Close()
        publisher.Close()
        for sig, fn in previous.items():
            signal.signal(sig, fn)
        result = bridge.report()
        result.update({"published_states": published,
                       "wall_seconds": round(time.monotonic() - start, 2)})
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
