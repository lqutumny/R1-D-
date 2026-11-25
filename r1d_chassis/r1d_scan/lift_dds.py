"""R1-A7-D 升降 DDS 消息，以及联动仿真的高度参考面。"""
import math

from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import Point32_, Twist_, Vector3_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import Error_

CMD_TOPIC = "rt/cmd_vel"
HEIGHT_TOPIC = "rt/height_state"
ALARM_TOPIC = "rt/height_alarm"
VELOCITY_TOPIC = "rt/state_vel"
TARGET_LOW_MM = 400.0
TARGET_HIGH_MM = 700.0
# 仿真沿用现场机械低位 0.10 m，按官方参考面加 5 mm。
# 该偏移须在真机上标定，不能用于直接控制真机。
SIM_ZERO_GAP_M = 0.105
LOW_GAP_M = SIM_ZERO_GAP_M + TARGET_LOW_MM / 1000
MAX_SPEED_M_S = 0.0765
COMMAND_CAP = 0.5


def vertical_twist(normalized):
    return Twist_(Vector3_(0.0, 0.0, float(normalized)),
                  Vector3_(0.0, 0.0, 0.0))


def encode_height(gap_m):
    return Point32_(0.0, 0.0, (gap_m - SIM_ZERO_GAP_M) * 1000.0)


def raw_height_mm(message):
    z_mm = float(message.z)
    if not math.isfinite(z_mm):
        raise ValueError("高度反馈含非有限值")
    return z_mm


def decode_height(message):
    return SIM_ZERO_GAP_M + raw_height_mm(message) / 1000.0
