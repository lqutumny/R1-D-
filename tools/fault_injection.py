#!/usr/bin/env python3
"""阶段 4.6：保护项注入测试。回答一个问题——**保护触发时，程序到底有没有把指令发出去？**

三层，越往后越接近真机：

  A  进程内（不启 DDS）
     每个 Guard 造一条违反它的输入，用记录型 publisher 数 "Write 被调了几次"。
     断言：抛 ControlFault，且这一拍一条指令都没发（写失败那条除外，它考的是返回值）。

  B  进程级（真 DDS）
     假机器人（fake_robot.py）演坏状态：CRC 错、丢包、tick 倒退、NaN、故障字、高温、模式变化。
     真 scan.py 跑在其中，另有一个旁观者在总线上数 rt/lowcmd 帧。
     断言：scan.py 以返回码 2 退出，且故障时刻之后总线上不再出现新指令。

  C  仿真器对抗
     往真 MuJoCo 仿真器发坏 LowCmd（坏 CRC / 模式不匹配 / NaN），
     确认它**真的拒绝**、物理一步都没走；再发一条好指令确认它接受。
     没有这一层，A、B 的"闭环跑通"就没有说服力——仿真器要是来者不拒，
     前面所有"拒绝数 0"都是假的。

用法：
  python3 tools/fault_injection.py --all          # 全套（约 3 分钟）
  python3 tools/fault_injection.py --unit         # 只跑进程内（几秒）
  python3 tools/fault_injection.py --dds --sim    # 只跑进程级
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
import types

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from controller import Controller, Limits, load_motion          # noqa: E402
from profile import load as load_profile                        # noqa: E402
from runner import Runner                                       # noqa: E402
from transport import Codec, ControlFault, Snapshot             # noqa: E402

OUT = ROOT / "out" / "faults"
_DDS_READY = False


def init_dds():
    """本进程只初始化一次 DDS（A/B/C 三段共用同一个 participant）。"""
    global _DDS_READY
    if not _DDS_READY:
        from dds_network import initialize
        initialize("lo", 0)
        _DDS_READY = True


# --------------------------------------------------------------------------- #
# 基础设施
# --------------------------------------------------------------------------- #
class Recorder:
    """记录型 publisher：Write 只记不发，返回 True 假装成功。"""

    def __init__(self, ok=True):
        self.messages = []
        self.ok = ok

    def Write(self, msg):
        self.messages.append(msg)
        return self.ok


class Rig:
    def __init__(self):
        self.profile = load_profile()
        self.codec = Codec(profile=self.profile)
        raw = json.loads((ROOT / "r1d_scan" / "robot_profile.json").read_text(encoding="utf-8"))
        self.lo = np.array([j["lower"] for j in raw["joints"]], float)
        self.hi = np.array([j["upper"] for j in raw["joints"]], float)
        self.effort = np.array([j["effort"] for j in raw["joints"]], float)
        self.captured = np.array([raw["initial"].get(j["name"], 0.0) for j in raw["joints"]], float)
        # 保持关节的目标：测试里用 profile 里的"下垂"姿态，等同于真机启动时捕获的实测值
        held = raw.get("held", {})
        if self.codec.hold_slots.size:
            self.codec.hold_targets = np.array(
                [held.get(n, 0.0) for n in self.codec.hold_names], float)
        _, self.motion = load_motion()
        self.controller = Controller(self.captured, self.motion, self.lo, self.hi,
                                     self.codec.kp, self.codec.kd, self.effort,
                                     limits=Limits(), start=0.0, expected_mode_machine=1)
        self.t0 = self.controller.start                        # = startup_hold
        self.t_scan = self.t0 + self.controller.approach_seconds


def make_state(rig, q, received, **kw):
    q = np.asarray(q, float)
    n = q.size
    base = dict(q=q, dq=np.zeros(n), tau=np.zeros(n), errors=np.zeros(n, np.uint32),
                temperature=np.full(n, 30.0), present=tuple(), mode_machine=1, mode_pr=0,
                tick=0, received=received, progressed=received)
    base.update(kw)
    return Snapshot(**base)


def park(rig, now):
    """把控制器摆到 now 时刻的"正常运行中"状态，避免跨用例的时间跳变误触角速度保护。"""
    c = rig.controller
    c.previous = now - 0.002
    c.last_q = np.asarray(c.target(now)[1], float).copy()
    c.bad_since = None
    c.complete = False
    c.finished_at = None


class StubMotion:
    """固定姿态的假轨迹：把目标直接放到指定位置（例如贴着限位）。"""

    def __init__(self, q, duration=10.0, one_way_duration=5.0, step_at=None, delta=0.0):
        self.q = np.asarray(q, float)
        self.duration = duration
        self.one_way_duration = one_way_duration
        self.step_at = step_at
        self.delta = delta

    def sample(self, t):
        q = self.q.copy()
        if self.step_at is not None and t >= self.step_at:
            q = q + self.delta
        return q


def report(results, echo):
    for item in results:
        echo(f"  {item['status']}  {item['case']:<14} {item.get('detail', '')}")
    return results


# --------------------------------------------------------------------------- #
# A. 进程内注入
# --------------------------------------------------------------------------- #
class Attempt:
    """包一层 runner.step，记录"这一拍开始前已经发了几条"，用来判断失败那一拍有没有发。"""

    def __init__(self, runner, pub):
        self.runner = runner
        self.pub = pub
        self.before = 0

    def __call__(self, state, now):
        self.before = len(self.pub.messages)
        return self.runner.step(state, now)


def run_unit(rig, echo):
    echo("[A] 进程内注入（记录型 publisher，数 Write 次数）")
    results = []

    def case(name, guard, expect, fn, expect_write=False):
        pub = Recorder()
        runner = Runner(rig.controller, rig.codec, pub, rig.profile, hold=0.0,
                        echo=lambda *_a, **_k: None)
        saved = rig.controller.motion
        attempt = Attempt(runner, pub)

        try:
            fn(rig, attempt)
            results.append({"case": name, "guard": guard, "status": "失败",
                            "detail": "没有触发保护", "sent": len(pub.messages)})
        except ControlFault as exc:
            msg = str(exc)
            hit = expect in msg
            delta = len(pub.messages) - attempt.before
            want = 1 if expect_write else 0
            ok = hit and delta == want
            results.append({"case": name, "guard": guard, "status": "通过" if ok else "失败",
                            "detail": msg, "sent": delta})
        except Exception as exc:  # noqa: BLE001
            results.append({"case": name, "guard": guard, "status": "失败",
                            "detail": f"异常类型不对 {type(exc).__name__}: {exc}"})
        finally:
            rig.controller.motion = saved
            rig.controller.complete = False

    # ---- 状态类 ----
    def no_state(rig, attempt):
        now = rig.t0 + 0.5
        park(rig, now)
        attempt(None, now)

    def stale_state(rig, attempt):
        now = rig.t0 + 0.5
        park(rig, now)
        attempt(make_state(rig, rig.captured, now - 0.5), now)     # 状态已 0.5 s 陈旧

    def bad_mode_pr(rig, attempt):
        now = rig.t0 + 0.5
        park(rig, now)
        attempt(make_state(rig, rig.captured, now, mode_pr=1), now)

    def bad_mode_machine(rig, attempt):
        now = rig.t0 + 0.5
        park(rig, now)
        attempt(make_state(rig, rig.captured, now, mode_machine=9), now)

    def motor_error(rig, attempt):
        now = rig.t0 + 0.5
        park(rig, now)
        errors = np.zeros(len(rig.captured), np.uint32)
        errors[2] = 1
        attempt(make_state(rig, rig.captured, now, errors=errors), now)

    def hot_motor(rig, attempt):
        now = rig.t0 + 0.5
        park(rig, now)
        attempt(make_state(rig, rig.captured, now, temperature=np.full(len(rig.captured), 80.0)), now)

    def measured_torque(rig, attempt):
        now = rig.t0 + 0.5
        park(rig, now)
        tau = np.zeros(len(rig.captured))
        tau[0] = rig.effort[0] * 1.2
        attempt(make_state(rig, rig.captured, now, tau=tau), now)

    # ---- 指令类 ----
    def near_limit(rig, attempt):
        now = rig.t_scan + 3.0
        rig.controller.motion = StubMotion(rig.lo + 0.01)         # 贴下限，越过 0.02 余量
        park(rig, now)
        attempt(make_state(rig, rig.lo + 0.01, now), now)

    def fast_jump(rig, attempt):
        now = rig.t_scan + 3.0
        rig.controller.motion = StubMotion(rig.captured, step_at=2.999, delta=0.20)
        park(rig, now - 0.002)                                     # 上一拍目标还是原值
        attempt(make_state(rig, rig.captured, now), now)           # 这一拍跳 0.2 rad → 100 rad/s

    def torque_budget(rig, attempt):
        now = rig.t_scan + 3.0
        park(rig, now)
        target = np.asarray(rig.controller.target(now)[1], float)
        attempt(make_state(rig, target - 0.60, now), now)          # kp=100 → 60 N·m > 0.9*60

    def tracking(rig, attempt):
        now = rig.t_scan + 3.0
        park(rig, now)
        # 只在小臂 pitch 上加偏差：kp=100 × 0.25 = 25 N·m < 54 N·m 预算，
        # 这样力矩 Guard 不会抢先，测到的确实是"跟踪误差持续"这一项。
        offset = np.zeros(len(rig.captured))
        offset[0] = 0.25
        for _ in range(400):
            target = np.asarray(rig.controller.target(now)[1], float)
            attempt(make_state(rig, target + offset, now), now)     # 持续 0.25 rad 偏差
            now += 0.002

    def slow_loop(rig, attempt):
        now = rig.t0 + 0.5
        park(rig, now)
        attempt(make_state(rig, rig.captured, now), now - 0.30)     # dt = 0.30 s > 单次上限 0.15

    def repeated_hiccups(rig, attempt):
        """10 s 内反复停顿超过阈值 → 判环境不适合，停。"""
        now = rig.t0 + 0.5
        park(rig, now)
        for _ in range(6):
            attempt(make_state(rig, rig.captured, now), now)
            now += 0.08        # 每次 dt = 80 ms > warn 50 ms，但 < 单次上限 150 ms

    def nan_target(rig, attempt):
        now = rig.t_scan + 3.0
        q = rig.captured.copy()
        q[0] = float("nan")
        rig.controller.motion = StubMotion(q)
        park(rig, now)
        attempt(make_state(rig, q, now), now)                       # 最后一道防线：编码阶段

    def write_fails(rig, attempt):
        now = rig.t0 + 0.5
        park(rig, now)
        attempt.pub.ok = False          # 发送返回 False，等价于 DDS 写不进去
        attempt(make_state(rig, rig.captured, now), now)

    case("无状态", "没有 LowState", "没有 LowState", no_state)
    case("状态超时", "状态超时 0.2 s", "超时", stale_state)
    case("mode_pr≠0", "模式变化", "mode_pr", bad_mode_pr)
    case("mode_machine 变", "模式变化", "mode_machine", bad_mode_machine)
    case("故障字", "电机故障字", "故障字", motor_error)
    case("电机过热", "温度 ≥70°C", "温度", hot_motor)
    case("实测力矩超限", "实测力矩 >110%", "实测力矩", measured_torque)
    case("目标超限位", "限位余量 0.02", "限位", near_limit)
    case("目标角速度超限", "角速度 0.40 rad/s", "角速度", fast_jump)
    case("预计力矩超预算", "力矩预算 90%", "力矩", torque_budget)
    case("跟踪误差持续", "跟踪误差 0.2 rad/0.3 s", "跟踪误差", tracking)
    case("控制周期异常", "控制周期 >150 ms", "控制周期", slow_loop)
    case("反复停顿", "10 s 内 5 次 >50 ms", "反复停顿", repeated_hiccups)
    case("目标非有限值", "编码阶段拦截", "有限关节角", nan_target)
    case("Write 返回 False", "发送失败", "写入失败", write_fails, expect_write=True)
    return report(results, echo)


# --------------------------------------------------------------------------- #
# A'. 解码层（坏包本身被拒绝）
# --------------------------------------------------------------------------- #
def run_codec(rig, echo):
    echo("[A'] LowState 解码层：坏包必须被拒，不能更新状态缓存")
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
    codec = rig.codec
    results = []

    def valid(tick=0):
        msg = unitree_hg_msg_dds__LowState_()
        msg.mode_pr = 0
        msg.mode_machine = 1
        msg.tick = tick
        for i, slot in enumerate(codec.slots):
            motor = msg.motor_state[int(slot)]
            motor.mode = 1
            motor.q = float(rig.captured[i])
            motor.dq = 0.0
            motor.tau_est = 0.0
            motor.temperature = [30, 30]
            motor.vol = 48.0
        msg.crc = codec.crc.Crc(msg)
        return msg

    def expect_fault(name, guard, want, fn):
        try:
            fn()
            results.append({"case": name, "guard": guard, "status": "失败",
                            "detail": "没有拒绝"})
        except ControlFault as exc:
            ok = want in str(exc)
            results.append({"case": name, "guard": guard,
                            "status": "通过" if ok else "失败", "detail": str(exc)})

    try:
        now = time.monotonic()
        snap = codec.decode(valid(), now, None)
        good = np.allclose(snap.q, rig.captured)
        results.append({"case": "正常报文", "guard": "对照", "status": "通过" if good else "失败",
                        "detail": f"q 与初始姿态一致={good}"})
    except ControlFault as exc:
        results.append({"case": "正常报文", "guard": "对照", "status": "失败", "detail": str(exc)})

    def bad_crc():
        msg = valid()
        msg.crc = (int(msg.crc) + 1) % 2**32
        codec.decode(msg, time.monotonic(), None)

    def bad_tick():
        msg = valid(tick=1000)
        codec.decode(msg, time.monotonic(), codec.decode(valid(tick=10000), time.monotonic(), None))

    def bad_nan():
        msg = valid()
        msg.motor_state[int(codec.slots[0])].q = float("nan")
        msg.crc = codec.crc.Crc(msg)
        codec.decode(msg, time.monotonic(), None)

    def short_msg():
        mock = types.SimpleNamespace(motor_state=[types.SimpleNamespace()] * 5, crc=0,
                                     tick=0, mode_machine=1, mode_pr=0)
        codec.decode(mock, time.monotonic(), None)

    expect_fault("坏 CRC", "报文 CRC", "CRC", bad_crc)
    expect_fault("tick 倒退", "时间戳单调", "tick", bad_tick)
    expect_fault("q=NaN", "数值有效", "非有限", bad_nan)
    expect_fault("数组太短", "槽位存在性", "长度", short_msg)
    return report(results, echo)


# --------------------------------------------------------------------------- #
# B. 进程级注入（真 DDS）
# --------------------------------------------------------------------------- #
class Tail:
    """起一个子进程并把输出收进列表，可等某个关键字出现。"""

    def __init__(self, args, cwd=ROOT):
        self.proc = subprocess.Popen([str(a) for a in args], cwd=str(cwd),
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1,
                                     env={**os.environ, "PYTHONUNBUFFERED": "1"})
        self.lines = []
        self.thread = threading.Thread(target=self._pump, daemon=True)
        self.thread.start()

    def _pump(self):
        for line in self.proc.stdout:
            self.lines.append(line.rstrip("\n"))

    def wait_for(self, needle, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(needle in line for line in self.lines):
                return True
            if self.proc.poll() is not None:
                return False
            time.sleep(0.05)
        return False

    def finish(self, timeout=180):
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.thread.join(timeout=5)
        return self.proc.returncode

    def stop(self):
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=5)
        self.thread.join(timeout=5)

    def find(self, needle):
        for line in self.lines:
            if needle in line:
                return line
        return None


DDS_CASES = [
    # stale=True：状态变陈旧，由 0.2 s 超时保护兜底，停止延迟就该 ~0.2 s
    # bad_packets=True：坏包会被逐条拒绝，states_bad 必须 > 0
    {"name": "故障字", "fault": "error", "at": 5.0, "expect": "故障字",
     "stale": False, "bad_packets": False},
    {"name": "电机过热", "fault": "temp", "at": 5.0, "expect": "温度",
     "stale": False, "bad_packets": False},
    {"name": "模式变化", "fault": "mode", "at": 5.0, "expect": "mode_pr",
     "stale": False, "bad_packets": False},
    {"name": "坏 CRC", "fault": "crc", "at": 5.0, "expect": "超时",
     "stale": True, "bad_packets": True},
    {"name": "tick 倒退", "fault": "tick", "at": 5.0, "expect": "超时",
     "stale": True, "bad_packets": True},
    {"name": "q=NaN", "fault": "nan", "at": 5.0, "expect": "超时",
     "stale": True, "bad_packets": True},
    {"name": "链路中断", "fault": "timeout", "at": 5.0, "expect": "超时",
     "stale": True, "bad_packets": False},
]


def run_dds(rig, echo, healthy=True):
    echo("[B] 进程级注入（假机器人 + 真 scan.py + 总线旁观者）")
    OUT.mkdir(parents=True, exist_ok=True)
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

    init_dds()
    sent = []
    lock = threading.Lock()

    def on_cmd(msg):                       # 总线旁观者：只数帧、校验 CRC
        with lock:
            sent.append(time.monotonic())

    sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
    sub.Init(on_cmd, 1)

    results = []
    cases = ([{"name": "健康对照", "fault": "none", "at": 999.0, "expect": None,
               "bad_packets": False}] if healthy else []) + DDS_CASES

    for case in cases:
        slug = case["fault"]
        with lock:
            sent.clear()
        robot = Tail([sys.executable, "r1d_scan/fake_robot.py", "--network", "lo",
                      "--domain", "0", "--fault", slug, "--at", case["at"],
                      "--duration", "120", "--report", OUT / f"robot_{slug}.json"])
        if not robot.wait_for("FAKE_READY", 30):
            robot.stop()
            results.append({"case": case["name"], "status": "失败", "detail": "假机器人未就绪"})
            echo(f"  失败  {case['name']:<10} 假机器人未就绪")
            continue
        with lock:
            sent.clear()
        scan = Tail([sys.executable, "r1d_scan/scan.py", "--network", "lo", "--domain", "0",
                     "--report", OUT / f"scan_{slug}.json", "--log", OUT / f"scan_{slug}.csv"], )
        code = scan.finish(timeout=180)
        with lock:
            stamps = list(sent)
        robot.stop()

        fault_line = robot.find("FAULT_INJECTED")
        fault_time = float(fault_line.split("t=")[1].split()[0]) if fault_line else None
        item = {"case": case["name"], "fault": slug, "exit_code": code,
                "commands_on_bus": len(stamps), "fault_at": fault_time}
        problems = []
        if case["expect"] is None:                              # 健康对照
            if code != 0:
                problems.append(f"退出码 {code}（应 0）")
            rp = OUT / f"scan_{slug}.json"
            rep = json.loads(rp.read_text(encoding="utf-8")) if rp.exists() else {}
            item.update(complete=rep.get("complete"), fault=rep.get("fault"),
                        commands_sent=rep.get("commands_sent"),
                        command_rate_hz=rep.get("command_rate_hz"),
                        loop_dt_max_ms=rep.get("loop_dt_max_ms"),
                        states_bad=rep.get("states_bad"),
                        max_tracking_error_rad=rep.get("max_tracking_error_rad"))
            if not rep.get("complete"):
                problems.append("没有走完整轮")
            if rep.get("fault"):
                problems.append(f"误报保护：{rep['fault']}")
            if rep.get("states_bad"):
                problems.append(f"有 {rep['states_bad']} 个坏状态")
        else:
            if code != 2:
                problems.append(f"退出码 {code}（保护触发应为 2）")
            rp = OUT / f"scan_{slug}.json"
            rep = json.loads(rp.read_text(encoding="utf-8")) if rp.exists() else {}
            fault = rep.get("fault") or ""
            item.update(fault=fault, commands_sent=rep.get("commands_sent"),
                        states_bad=rep.get("states_bad"))
            if case["expect"] not in fault:
                problems.append(f"保护项不对：{fault!r}")
            # 主判据：进程内计时。故障注入时刻与 scan.py 的最后一条指令时刻都是
            # CLOCK_MONOTONIC，差值是"真正多久停下来的"，不受总线投递抖动影响。
            last_cmd = rep.get("last_command_at")
            detected = rep.get("fault_detected_at")
            if fault_time is None:
                problems.append("没有捕捉到故障注入时刻")
            elif last_cmd is None:
                problems.append("报告里没有最后一条指令的时刻")
            else:
                delay = last_cmd - fault_time
                limit = 0.30 if case.get("stale") else 0.05
                item["stop_delay_s"] = round(delay, 3)
                item["detect_delay_s"] = round(detected - fault_time, 3) if detected else None
                if delay > limit:
                    problems.append(f"停止太晚：最后一条指令在故障后 {delay*1000:.0f} ms")
            # 旁证：总线上故障 0.3 s 之后不应再成串出现指令
            if fault_time is not None and stamps:
                after = [t for t in stamps if t > fault_time + 0.30]
                item["commands_after_fault"] = len(after)
                if after:
                    problems.append(f"总线在故障 0.3 s 后仍有 {len(after)} 条指令")
            if case["bad_packets"] and not (rep.get("states_bad") or 0):
                problems.append("坏包计数为 0（没有被真正拒绝）")
            if len(stamps) < 100:
                problems.append(f"故障前只发了 {len(stamps)} 条，样本不足")
        item["status"] = "通过" if not problems else "失败"
        if case["expect"] is None:
            item["detail"] = "；".join(problems) if problems else (
                f"跑完整轮 {item.get('commands_sent')} 条指令，{item.get('command_rate_hz')} Hz，"
                f"最大跟踪误差 {item.get('max_tracking_error_rad')} rad，"
                f"控制周期最大 {item.get('loop_dt_max_ms')} ms")
        else:
            delay = item.get("stop_delay_s")
            text = f"退出码 {code}，故障后 0 条指令"
            if delay is not None:
                text += f"，停止延迟 {delay*1000:.0f} ms"
                if item.get("detect_delay_s") is not None:
                    text += f"（检测 {item['detect_delay_s']*1000:.0f} ms）"
            item["detail"] = "；".join(problems) if problems else text
        results.append(item)
        echo(f"  {item['status']}  {case['name']:<10} {item['detail']}")

    sub.Close()
    return results


# --------------------------------------------------------------------------- #
# C. 仿真器对抗
# --------------------------------------------------------------------------- #
def run_sim(rig, echo):
    echo("[C] 仿真器对抗：坏 LowCmd 必须被拒，物理不能动")
    OUT.mkdir(parents=True, exist_ok=True)
    from unitree_sdk2py.core.channel import ChannelPublisher
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_

    init_dds()
    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    proxy = types.SimpleNamespace(mode_machine=1)
    results = []

    def good_command():
        return rig.codec.encode(rig.captured, proxy)

    def one_round(name, senders, expect_accepted, expect_rejected):
        slug = name.replace(" ", "_")
        rp = OUT / f"sim_{slug}.json"
        sim = Tail([sys.executable, "r1d_scan/sim.py", "--network", "lo", "--domain", "0",
                    "--no-viewer", "--duration", "60", "--report", rp])
        if not sim.wait_for("SIM_READY", 60):
            sim.stop()
            echo(f"  失败  {name}：仿真器未就绪")
            return {"case": name, "status": "失败", "detail": "仿真器未就绪"}
        time.sleep(0.3)
        for sender in senders:
            publisher.Write(sender())
            time.sleep(0.35)
        time.sleep(0.4)
        sim.stop()
        rep = json.loads(rp.read_text(encoding="utf-8")) if rp.exists() else {}
        problems = []
        if rep.get("commands_accepted") != expect_accepted:
            problems.append(f"接受 {rep.get('commands_accepted')} 条（应 {expect_accepted}）")
        if rep.get("commands_rejected") != expect_rejected:
            problems.append(f"拒绝 {rep.get('commands_rejected')} 条（应 {expect_rejected}）")
        item = {"case": name, "accepted": rep.get("commands_accepted"),
                "rejected": rep.get("commands_rejected"),
                "last_reject": rep.get("last_reject"),
                "physics_steps": rep.get("physics_steps")}
        if expect_accepted == 0:
            if rep.get("physics_steps"):
                problems.append(f"物理走了 {rep['physics_steps']} 步")
            if not np.allclose(rep.get("final_q_rad", [0] * len(rig.captured)), rig.captured, atol=2e-3):
                problems.append(f"关节角变了 {rep.get('final_q_rad')}")
        item["status"] = "通过" if not problems else "失败"
        item["detail"] = "；".join(problems) if problems else (
            f"接受 {item['accepted']}、拒绝 {item['rejected']}"
            + (f"，最后一次拒绝：{item['last_reject']}" if item["last_reject"] else "")
            + (f"，物理 {item['physics_steps']} 步" if item["physics_steps"] else "，物理 0 步"))
        echo(f"  {item['status']}  {name:<12} {item['detail']}")
        return item

    def bad_crc():
        cmd = good_command()
        cmd.crc = (int(cmd.crc) + 1) % 2**32
        return cmd

    def bad_mode():
        cmd = good_command()
        cmd.mode_machine = 9
        cmd.crc = rig.codec.crc.Crc(cmd)
        return cmd

    def bad_nan():
        cmd = good_command()
        cmd.motor_cmd[int(rig.codec.slots[0])].q = float("nan")
        cmd.crc = rig.codec.crc.Crc(cmd)
        return cmd

    results.append(one_round("三种坏指令", [bad_crc, bad_mode, bad_nan], 0, 3))
    results.append(one_round("好指令对照", [good_command], 1, 0))
    publisher.Close()
    return results


# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--unit", action="store_true", help="A 进程内注入")
    parser.add_argument("--codec", action="store_true", help="A' 解码层")
    parser.add_argument("--dds", action="store_true", help="B 进程级注入（会占用 lo 上的 DDS）")
    parser.add_argument("--sim", action="store_true", help="C 仿真器对抗")
    parser.add_argument("--all", action="store_true", help="全部")
    parser.add_argument("--no-healthy", action="store_true", help="跳过 B 里的健康对照（省 50 s）")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "fault_injection.json")
    args = parser.parse_args()
    if not any((args.unit, args.codec, args.dds, args.sim, args.all)):
        args.all = True

    echo = print
    t_start = time.monotonic()
    rig = Rig()
    out = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"), "unit": [], "codec": [], "dds": [], "sim": []}

    if args.all or args.unit:
        out["unit"] = run_unit(rig, echo)
    if args.all or args.codec:
        out["codec"] = run_codec(rig, echo)
    if args.all or args.dds:
        try:
            out["dds"] = run_dds(rig, echo, healthy=not args.no_healthy)
        except Exception as exc:  # noqa: BLE001
            echo(f"[B] 整段失败：{type(exc).__name__}: {exc}")
            out["dds"] = [{"case": "进程级", "status": "失败", "detail": str(exc)}]
    if args.all or args.sim:
        try:
            out["sim"] = run_sim(rig, echo)
        except Exception as exc:  # noqa: BLE001
            echo(f"[C] 整段失败：{type(exc).__name__}: {exc}")
            out["sim"] = [{"case": "仿真器对抗", "status": "失败", "detail": str(exc)}]

    every = [item for section in ("unit", "codec", "dds", "sim") for item in out[section]]
    out["passed"] = sum(1 for item in every if item.get("status") == "通过")
    out["total"] = len(every)
    out["seconds"] = round(time.monotonic() - t_start, 1)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    echo(f"\n合计 {out['passed']}/{out['total']} 通过，用时 {out['seconds']} s，"
         f"报告 {args.report.relative_to(ROOT)}")
    return 0 if out["passed"] == out["total"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
