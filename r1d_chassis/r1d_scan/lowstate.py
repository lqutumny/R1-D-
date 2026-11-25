"""LowState → 普通字典。纯 Python（不 import SDK），方便离线分析。

槽位"存在"的判据：任一状态字段非零，或 mode 非 0，或故障字非 0。
不能只用 q==0 判断——零位静止的电机也会报 q=0。
"""

FIELDS = ("mode", "q", "dq", "ddq", "tau_est", "temperature", "vol", "motorstate", "reserve", "sensor")
MOTION_FIELDS = ("q", "dq", "ddq", "tau_est")


def slot_rows(msg):
    """把 SDK 的 LowState_ 消息转成 slot dict 列表（全部槽位，不做筛选）。"""
    rows = []
    for slot, motor in enumerate(msg.motor_state):
        row = {"slot": slot}
        for field in FIELDS:
            value = getattr(motor, field, None)
            if value is None:
                continue
            row[field] = list(value) if isinstance(value, (list, tuple)) else value
        rows.append(row)
    return rows


def _num(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def temperature_max(row):
    value = row.get("temperature")
    if isinstance(value, (list, tuple)):
        return max((_num(v) for v in value), default=0.0)
    return _num(value)


def signal_class(row):
    """槽位信号分三类，避免"有电压就算存在"这种误判。

    motor  ：有控制模式/故障字/运动量 → 大概率是真实电机槽位
    power  ：只有电压或温度 → 有供电但没在受控（可能是占位值或未使能）
    silent ：全零
    """
    if int(_num(row.get("mode"))) != 0 or int(_num(row.get("motorstate"))) != 0:
        return "motor"
    if any(abs(_num(row.get(field))) > 1e-6 for field in MOTION_FIELDS):
        return "motor"
    if _num(row.get("vol")) > 1.0 or temperature_max(row) > 1.0:
        return "power"
    return "silent"


def looks_present(row):
    """该槽位是否有任何信号（含仅供电）。"""
    return signal_class(row) != "silent"


def summary(rows):
    """按槽位给出供人判断的一行摘要。"""
    return [{"slot": int(row["slot"]),
             "signal": signal_class(row),
             "present": looks_present(row),
             "mode": int(_num(row.get("mode"))),
             "q_deg": _num(row.get("q")) * 57.29577951308232,
             "dq": _num(row.get("dq")),
             "tau_est": _num(row.get("tau_est")),
             "vol": _num(row.get("vol")),
             "temp_C": temperature_max(row),
             "error": int(_num(row.get("motorstate")))}
            for row in rows]
