"""机型 profile：关节名 / DDS 槽位 / KP-KD 的唯一来源。

设计原则：**未确认的槽位必须是 None**。任何需要槽位的后续阶段都先调 require_ready()，
这样"还没核对就先发指令"在代码层面就不可能发生。
"""
from dataclasses import dataclass
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROFILE_PATH = ROOT / "r1d_scan" / "robot_profile.json"
ROLES = ("waist", "left_arm", "right_arm")


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class Joint:
    name: str
    slot: int | None
    kp: float | None
    kd: float | None
    role: str


@dataclass(frozen=True)
class HoldJoint:
    """不由本程序规划、但每拍要"写固定值让它不动"的关节（真机上主要是右臂）。"""
    name: str
    slot: int | None
    kp: float | None
    kd: float | None


@dataclass(frozen=True)
class Profile:
    robot: str
    joints: tuple
    tool: dict
    verified: bool
    slot_source: str = "provisional"
    hold_joints: tuple = ()

    def names(self):
        return [j.name for j in self.joints]

    def by_role(self, role):
        return [j for j in self.joints if j.role == role]

    def slot_map(self):
        return {j.name: j.slot for j in self.joints if j.slot is not None}

    def hold_slot_map(self):
        return {j.name: j.slot for j in self.hold_joints if j.slot is not None}

    def gaps(self):
        """列出还不能进入写指令阶段的原因。空列表表示就绪。"""
        out = []
        for j in self.joints:
            if j.slot is None:
                out.append(f"{j.name}: 缺 DDS 槽位")
            if j.kp is None or j.kd is None:
                out.append(f"{j.name}: 缺 KP/KD")
        for j in self.hold_joints:
            if j.slot is None:
                out.append(f"{j.name}（保持）: 缺 DDS 槽位")
            if j.kp is None or j.kd is None:
                out.append(f"{j.name}（保持）: 缺 KP/KD")
        occupied = set(self.slot_map().values()) & set(self.hold_slot_map().values())
        for slot in sorted(occupied):
            out.append(f"槽位 {slot} 同时出现在控制集和保持集")
        if self.tool.get("offset_m") is None:
            out.append("tool.offset_m 未定（Q2d/Q2e）")
        if self.slot_source != "verified":
            out.append(f"slot_source={self.slot_source!r}：暂定槽位可用于仿真，不能用于真机运动")
        return out

    def require_ready(self):
        gaps = self.gaps()
        if gaps:
            raise ProfileError("profile 未就绪，拒绝进入写指令阶段：\n  - " + "\n  - ".join(gaps))
        return self


def load(path=PROFILE_PATH):
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ProfileError(f"无法读取 profile: {path}: {exc}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("joints"), list):
        raise ProfileError("profile 必须是含 joints 数组的对象")
    joints = []
    seen_names, seen_slots = set(), set()
    for raw in data["joints"]:
        if not isinstance(raw, dict) or not raw.get("name"):
            raise ProfileError(f"关节项格式错误: {raw!r}")
        name = raw["name"]
        if name in seen_names:
            raise ProfileError(f"关节名重复: {name}")
        seen_names.add(name)
        slot = raw.get("slot")
        if slot is not None:
            if not isinstance(slot, int) or slot < 0:
                raise ProfileError(f"{name}: slot 必须是非负整数或 null")
            if slot in seen_slots:
                raise ProfileError(f"{name}: slot {slot} 与其他关节重复")
            seen_slots.add(slot)
        kp, kd = raw.get("kp"), raw.get("kd")
        for label, value in (("kp", kp), ("kd", kd)):
            if value is not None and (not isinstance(value, (int, float)) or value < 0):
                raise ProfileError(f"{name}: {label} 必须是非负数或 null")
        role = raw.get("role")
        if role not in ROLES:
            raise ProfileError(f"{name}: role 必须是 {ROLES} 之一，实际 {role!r}")
        joints.append(Joint(name, slot, kp, kd, role))
    if not joints:
        raise ProfileError("profile 里没有关节")
    tool = data.get("tool") or {}
    hold_joints = []
    hold_raw = (data.get("hold_on_robot") or {}).get("joints") or []
    for raw in hold_raw:
        if not isinstance(raw, dict) or not raw.get("name"):
            raise ProfileError(f"保持关节格式错误: {raw!r}")
        slot = raw.get("slot")
        if slot is not None and (not isinstance(slot, int) or slot < 0):
            raise ProfileError(f"{raw['name']}（保持）: slot 必须是非负整数或 null")
        kp, kd = raw.get("kp"), raw.get("kd")
        for label, value in (("kp", kp), ("kd", kd)):
            if value is not None and (not isinstance(value, (int, float)) or value < 0):
                raise ProfileError(f"{raw['name']}（保持）: {label} 必须是非负数或 null")
        hold_joints.append(HoldJoint(str(raw["name"]), slot, kp, kd))
    return Profile(robot=str(data.get("robot", "")), joints=tuple(joints), tool=tool,
                   verified=bool(data.get("verified", False)),
                   slot_source=str(data.get("slot_source", "provisional")),
                   hold_joints=tuple(hold_joints))


def status(profile):
    """给人和脚本看的一页状态。"""
    ready = not profile.gaps()
    lines = [f"机型: {profile.robot}",
             f"关节数: {len(profile.joints)}（腰 {len(profile.by_role('waist'))}，"
             f"左臂 {len(profile.by_role('left_arm'))}，右臂 {len(profile.by_role('right_arm'))}）",
             f"保持关节数: {len(profile.hold_joints)}（每拍写固定值让它们不动）",
             f"槽位来源: {profile.slot_source}",
             f"已确认槽位: {len(profile.slot_map())}/{len(profile.joints)}",
             f"可进入真机写指令阶段: {'是' if ready else '否'}"]
    lines.extend(f"  - {gap}" for gap in profile.gaps())
    return "\n".join(lines)


if __name__ == "__main__":
    print(status(load()))
