"""控制主循环：把"取状态 → 算目标 → 编码 → 发送"做成可单测的单元。

不含 DDS：publisher 只要有 `Write(msg) -> bool`，状态由外部传入。
这样"保护触发时到底有没有把指令发出去"可以用一个记录用的假 publisher 直接数，
不需要起仿真器。

**不变量（阶段 4 Gate 的唯一一条硬要求）**
    只有 controller.tick（内含全部 Guard）与 codec.encode 都成功，才调用 publisher.Write。
    任何一个 Guard 抛 ControlFault，本条指令就不发；机器人侧仍执行上一条指令
    （KP/KD 保持），也就是"停轨迹并保持最后目标"，不会被写进一条半成品指令。
"""
import csv
from pathlib import Path
import time

import numpy as np

from transport import ControlFault


class Runner:
    def __init__(self, controller, codec, publisher, profile, log_path=None, hold=2.0,
                 period=0.002, clock=time.monotonic, sleep=time.sleep, echo=print):
        self.controller = controller
        self.codec = codec
        self.publisher = publisher
        self.profile = profile
        self.hold = float(hold)
        self.period = float(period)
        self.clock = clock
        self.sleep = sleep
        self.echo = echo
        self.log_path = Path(log_path) if log_path else None
        self.log = None
        self.writer = None
        self.last_log = 0.0
        # 统计
        self.writes = 0                 # 真正写进 DDS 的指令条数
        self.max_error = 0.0
        self.dt_max = 0.0               # 控制周期最大间隔（真机排障用）
        self.last_write_at = None       # 最后一条指令写出的时刻（判"何时停的"）
        self.phase_history = []
        self.last_phase = None
        self.finished_at = None
        self.duration = 0.0

    # ---------- 日志 ----------
    def open_log(self):
        if self.log_path is None:
            return self
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log = self.log_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.log)
        names = self.profile.names()
        self.writer.writerow(["t", "phase"] + [f"q_cmd_{n}" for n in names]
                             + [f"q_act_{n}" for n in names] + [f"tau_{n}" for n in names])
        return self

    def close(self):
        if self.log is not None:
            self.log.close()
            self.log = None
            self.writer = None

    # ---------- 单步 ----------
    def step(self, state, now):
        """走一个控制周期。返回 True=继续，False=保持时间已满、可以收尾。

        任何保护项触发都会抛 ControlFault，且在 Write 之前抛出 → 本周期不发指令。
        """
        dt = now - self.controller.previous
        if dt > self.dt_max:
            self.dt_max = dt
        phase, q_target, tau_peak = self.controller.tick(state, now)
        command = self.codec.encode(q_target, state, self.controller.last_ff)
        if not self.publisher.Write(command):
            raise ControlFault("LowCmd 写入失败（publisher.Write 返回 False）")
        self.writes += 1
        self.last_write_at = now

        error = float(np.max(np.abs(q_target - state.q)))
        self.max_error = max(self.max_error, error)
        if phase != self.last_phase:
            self.last_phase = phase
            t = now - self.controller.start + self.controller.limits.startup_hold
            self.phase_history.append({"t": round(t, 2), "phase": phase})
            self.echo(f"[{t:6.2f}s] {phase} "
                      f"(|tau|peak {tau_peak:.1f} N·m, err {error:.3f} rad)")
        if self.writer is not None and now - self.last_log >= 0.02:
            self.writer.writerow([round(now - self.controller.start, 3), phase,
                                  *q_target, *state.q, *state.tau])
            self.last_log = now
        if self.controller.complete:
            if self.finished_at is None:
                self.finished_at = now
            elif now - self.finished_at >= self.hold:
                return False
        return True

    # ---------- 主循环 ----------
    def run(self, state_getter, stop=None):
        """跑到状态机收尾或 stop() 为真；保护触发时 ControlFault 原样抛出。"""
        deadline = self.clock()
        started = deadline
        try:
            while True:
                if stop is not None and stop():
                    break
                if not self.step(state_getter(), self.clock()):
                    break
                deadline += self.period
                slack = deadline - self.clock()
                if slack > 0:
                    self.sleep(slack)
                else:
                    deadline = self.clock()
        finally:
            # 保护触发时也要有真实时长，否则报告里的指令频率会算成天文数字
            self.duration = self.clock() - started
        return self
