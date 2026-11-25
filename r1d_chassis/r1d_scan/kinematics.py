"""左臂 + 腰 yaw 的运动学封装：FK、TCP、5 约束 IK（位置 3 + 探头轴线 2）。

约定：
- TCP = wrist link 原点沿局部 +x 伸出 probe_len（与 build_scene.py 的探头一致）；
- 探头轴线 = wrist 局部 +x 在世界系的方向；
- 8 个关节对 5 个约束 → 2 个冗余自由度，用软限位惩罚 + 小正则项消解。
"""
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares

from profile import load as load_profile

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "models" / "r1_a7_scene.xml"

JOINTS = [joint.name for joint in load_profile().joints]


def default_seed():
    """一个"手臂前伸、肘部微弯"的初值；按关节名赋值，不依赖关节数量。"""
    seed = np.zeros(len(JOINTS))
    mapping = {"left_shoulder_pitch_joint": 0.9, "left_shoulder_roll_joint": 0.1,
               "left_elbow_joint": 1.2, "left_wrist_pitch_joint": -1.2,
               "waist_yaw_joint": 0.0}
    for index, name in enumerate(JOINTS):
        seed[index] = mapping.get(name, 0.0)
    return seed
TCP_SITE = "tcp"
WRIST_BODY = "left_wrist_yaw_link"


@dataclass
class Solution:
    q: np.ndarray
    position_error: float
    axis_error_deg: float
    min_margin_rad: float
    binding_joint: str


class ArmKinematics:
    def __init__(self, scene=SCENE):
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.jids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINTS]
        missing = [n for n, j in zip(JOINTS, self.jids) if j < 0]
        if missing:
            raise ValueError(f"模型缺少关节: {missing}")
        self.qadr = np.array([self.model.jnt_qposadr[j] for j in self.jids])
        self.dadr = np.array([self.model.jnt_dofadr[j] for j in self.jids])
        self.lo = np.array([self.model.jnt_range[j][0] for j in self.jids])
        self.hi = np.array([self.model.jnt_range[j][1] for j in self.jids])
        self.tcp_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
        self.wrist = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, WRIST_BODY)

    # ---------- FK ----------
    def forward(self, q):
        """返回 (TCP 世界位置, 探头轴线世界方向, 腕部旋转矩阵)。"""
        self.data.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.model, self.data)
        rotation = self.data.xmat[self.wrist].reshape(3, 3)
        return self.data.site_xpos[self.tcp_site].copy(), rotation[:, 0].copy(), rotation

    def tcp(self, q):
        return self.forward(q)[0]

    def position_jacobian(self, q):
        """TCP 位置对 8 个关节的雅可比，形状 (3, 8)。"""
        self.forward(q)
        jacp = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp, None, self.tcp_site)
        return jacp[:, self.dadr]

    def margin(self, q):
        gaps = np.minimum(q - self.lo, self.hi - q)
        i = int(np.argmin(gaps))
        return float(gaps[i]), JOINTS[i]

    # ---------- IK ----------
    def main_residual(self, q, target, axis_target, weight_axis=1.0):
        position, axis, _ = self.forward(q)
        return np.r_[position - target, weight_axis * (axis - axis_target)]

    def follow_line(self, points, axis_target, q, weight_axis=1.0, iters=6, active=None,
                    axis_scale=0.2, max_step=0.05, damping=1e-6, joint_weights=None):
        """沿给定点位连续推进（微分 IK）：阻尼最小二乘 + 量纲加权 + 关节刚度权重。

        位置残差单位是米、轴线残差无量纲，直接求伪逆会让步长失去意义，
        因此把轴线行乘一个长度尺度 axis_scale 再求解，并限制单步幅值。
        joint_weights 越大，该关节越"不愿意动"（例如腰 yaw 应尽量少转）。
        """
        active = np.ones(len(JOINTS), dtype=bool) if active is None else np.asarray(active, bool)
        idx = np.flatnonzero(active)
        axis_target = np.asarray(axis_target, float)
        axis_target = axis_target / np.linalg.norm(axis_target)
        weight = np.diag([1.0, 1.0, 1.0, axis_scale, axis_scale, axis_scale])
        stiffness = np.ones(len(JOINTS)) if joint_weights is None else np.asarray(joint_weights, float)
        reg = np.diag((0.05 * stiffness[idx]) ** 2) + damping * np.eye(idx.size)
        q = np.asarray(q, float).copy()
        path = [q.copy()]
        for target in list(points)[1:]:
            target = np.asarray(target, float)
            for _ in range(iters):
                position, axis, _ = self.forward(q)
                error = np.r_[target - position, weight_axis * (axis_target - axis)]
                if np.linalg.norm(error) < 1e-10:
                    break
                jac = self.jacobian(q, target, axis_target, weight_axis, active=active)
                weighted_jac, weighted_error = weight @ jac, weight @ error
                normal = weighted_jac.T @ weighted_jac + reg
                dq = np.linalg.solve(normal, weighted_jac.T @ weighted_error)
                norm = np.linalg.norm(dq)
                if norm > max_step:
                    dq = dq * (max_step / norm)
                q = q.copy()
                q[idx] = np.clip(q[idx] + dq, self.lo[idx] + 1e-9, self.hi[idx] - 1e-9)
            path.append(q.copy())
        return np.array(path)

    def solve_main(self, target, axis_target, seed, weight_axis=1.0, max_nfev=200, active=None):
        """只解主任务（位置 3 + 轴线 2），不加任何参考项，保证约束精确满足。

        active 为布尔掩码：只优化被选中的关节，其余保持 seed 的值。
        """
        target = np.asarray(target, float)
        axis_target = np.asarray(axis_target, float)
        axis_target = axis_target / np.linalg.norm(axis_target)
        seed = np.clip(np.asarray(seed, float), self.lo + 1e-9, self.hi - 1e-9)
        if active is None:
            active = np.ones(len(JOINTS), dtype=bool)
        active = np.asarray(active, bool)
        idx = np.flatnonzero(active)

        def expand(value):
            q = seed.copy()
            q[idx] = value
            return q

        result = least_squares(
            lambda value: self.main_residual(expand(value), target, axis_target, weight_axis),
            seed[idx], bounds=(self.lo[idx], self.hi[idx]), method="trf",
            xtol=1e-14, ftol=1e-14, gtol=1e-14, max_nfev=max_nfev)
        return expand(result.x)

    def jacobian(self, q, target, axis_target, weight_axis=1.0, eps=1e-7, active=None):
        """主任务残差的数值雅可比，形状 (5, 有效关节数)。"""
        active = np.ones(len(JOINTS), dtype=bool) if active is None else np.asarray(active, bool)
        idx = np.flatnonzero(active)
        base = self.main_residual(q, target, axis_target, weight_axis)
        jac = np.zeros((base.size, idx.size))
        for col, k in enumerate(idx):
            step = np.zeros_like(q)
            step[k] = eps
            jac[:, col] = (self.main_residual(q + step, target, axis_target, weight_axis)
                           - base) / eps
        return jac

    def limit_pull(self, q, margin=0.15):
        """把关节往限位内侧推的方向（越靠近限位，力越大）。"""
        pull = np.zeros_like(q)
        near_lo = self.lo + margin - q
        near_hi = q - (self.hi - margin)
        pull += np.maximum(0.0, near_lo)
        pull -= np.maximum(0.0, near_hi)
        return pull

    def null_space_refine(self, q, target, axis_target, margin=0.15, toward=None,
                          iters=6, step=0.05, weight_axis=1.0, active=None):
        """在零空间里做两件事：远离限位、以及在 toward（上一点的解）附近少漂移。

        探头是圆柱，绕自身轴的滚转对任务无影响，属于"自由方向"；不加 toward 约束时
        求解器会在这些方向漂移，白白增加关节行程、拖慢轨迹。
        """
        active = np.ones(len(JOINTS), dtype=bool) if active is None else np.asarray(active, bool)
        idx = np.flatnonzero(active)
        for _ in range(iters):
            pull = self.limit_pull(q, margin)
            if toward is not None:
                pull = pull + 0.5 * (np.asarray(toward, float) - q)
            if not np.any(pull[idx]):
                break
            jac = self.jacobian(q, target, axis_target, weight_axis, active=active)
            _, _, vt = np.linalg.svd(jac, full_matrices=True)
            null = vt[jac.shape[0]:].T                    # (有效关节数 - 5, 零空间维数)
            if null.size == 0:
                break
            delta = null @ (null.T @ pull[idx])
            norm = np.linalg.norm(delta)
            if norm < 1e-9:
                break
            candidate = q.copy()
            candidate[idx] = np.clip(q[idx] + step * delta / norm,
                                     self.lo[idx] + 1e-9, self.hi[idx] - 1e-9)
            q = self.solve_main(target, axis_target, candidate, weight_axis,
                                max_nfev=60, active=active)
        return q

    def solve(self, target, axis_target, seed, margin=0.15, weight_axis=1.0, refine=True,
              toward=None, active=None):
        """主约束精确求解；再可选地在零空间里改善余量、抑制漂移。参考项不参与主残差。"""
        target = np.asarray(target, float)
        axis_target = np.asarray(axis_target, float)
        axis_target = axis_target / np.linalg.norm(axis_target)
        q = self.solve_main(target, axis_target, seed, weight_axis, active=active)
        if refine:
            q = self.null_space_refine(q, target, axis_target, margin, toward=toward,
                                       weight_axis=weight_axis, active=active)
        position, axis, _ = self.forward(q)
        position_error = float(np.linalg.norm(position - target))
        axis_error = float(np.degrees(np.arccos(np.clip(axis @ axis_target, -1, 1))))
        margin_rad, binding = self.margin(q)
        return Solution(q, position_error, axis_error, margin_rad, binding)
