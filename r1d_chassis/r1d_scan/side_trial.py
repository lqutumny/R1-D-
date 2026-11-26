"""左侧低速试扫时，只读检查底盘和固定立柱反馈。"""
import time

from sync_scan import LiftChannel
from transport import ControlFault


class FixedBaseTrial:
    MIN_HEIGHT_M = .390
    MAX_HEIGHT_M = .410
    MAX_HEIGHT_CHANGE_M = .003
    MAX_LIFT_SPEED_M_S = .003
    # 固定立柱阶段不发送升降指令。允许虚拟机 DDS 偶发少量抖动，
    # 但仍在半秒内停止手臂；联动控制继续使用 sync_scan.py 的 200 ms 阈值。
    FEEDBACK_TIMEOUT_S = .50
    STILL_SECONDS = .40

    def __init__(self, resources):
        self.lift = LiftChannel(resources)
        self.initial_height = None

    def wait_still(self):
        deadline = time.monotonic() + 3.0
        while any(value is None for value in (self.lift.height, self.lift.velocity, self.lift.alarm)):
            if self.lift.fault or time.monotonic() >= deadline:
                raise ControlFault(self.lift.fault or "未收到完整的底盘和立柱反馈")
            time.sleep(.02)
        self.initial_height = self.check()
        until = time.monotonic() + self.STILL_SECONDS
        while time.monotonic() < until:
            time.sleep(.02)
            self.check()
        return self.initial_height

    def check(self):
        height, speed = self.lift.feedback(time.monotonic(), timeout=self.FEEDBACK_TIMEOUT_S)
        if not self.MIN_HEIGHT_M <= height <= self.MAX_HEIGHT_M:
            raise ControlFault(f"固定立柱读数 {1000 * height:.1f} mm，试扫要求 390–410 mm")
        if abs(speed) > self.MAX_LIFT_SPEED_M_S:
            raise ControlFault("低速试扫期间检测到立柱在动")
        if (self.initial_height is not None
                and abs(height - self.initial_height) > self.MAX_HEIGHT_CHANGE_M):
            raise ControlFault("低速试扫期间立柱高度偏离起始值超过 3 mm")
        return height
