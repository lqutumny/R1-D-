"""按当前右臂姿态检查左臂的接近和收纳路径。"""
import math

import mujoco
import numpy as np

from transport import ControlFault


class Clearance:
    def __init__(self, kin, lo, hi, wall_x, held, wall_axis=0):
        self.kin = kin
        self.model, self.data = kin.model, kin.data
        self.lo, self.hi = lo, hi
        self.wall_pos, self.wall_axis = wall_x, wall_axis
        self.left = set()
        self.base = []
        self.meshes = {}
        for i, body in enumerate(self.model.geom_bodyid):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body) or ""
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if name in ("base_plate", "lift_mast") or (name or "").startswith(("lift_sleeve_", "lift_middle_")):
                self.base.append(i)
            if body_name.startswith("left_") or body_name == "radiate_probe":
                self.left.add(i)
                if self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
                    mesh = self.model.geom_dataid[i]
                    start = self.model.mesh_vertadr[mesh]
                    self.meshes[i] = self.model.mesh_vert[start:start + self.model.mesh_vertnum[mesh]]
        # 分离的包围球不可能接触，剩余配对仍按 MuJoCo 的精确距离判断。
        pairs = [(i, base) for i in self.left for base in self.base]
        self.pair_left, self.pair_base = np.asarray(pairs, dtype=np.intp).reshape(-1, 2).T
        radii = self.model.geom_rbound.copy()
        for i, vertices in self.meshes.items():
            radii[i] = max(radii[i], float(np.linalg.norm(vertices, axis=1).max()))
        self.pair_radius_squared = (radii[self.pair_left] + radii[self.pair_base] + 1e-6) ** 2
        for name, angle in held.items():
            joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint >= 0:
                self.data.qpos[self.model.jnt_qposadr[joint]] = angle

    def issue(self, q):
        if not np.isfinite(q).all() or np.any(q < self.lo) or np.any(q > self.hi):
            return "关节限位余量不足"
        self.data.qpos[self.kin.qadr] = q
        mujoco.mj_fwdPosition(self.model, self.data)
        if self.wall_pos - self.data.site_xpos[self.kin.tcp_site, self.wall_axis] < 0.10:
            return "探头到箱面的间距不足 100 mm"
        for contact in self.data.contact:
            if contact.dist < -0.002 and (contact.geom1 in self.left or contact.geom2 in self.left):
                return "左臂或探头与其他部件碰撞"
        for i in self.left:
            coordinate = self.data.geom_xpos[i, self.wall_axis]
            radius = self.model.geom_rbound[i]
            if coordinate + radius > self.wall_pos - 0.01:
                local_normal = self.data.geom_xmat[i].reshape(3, 3)[self.wall_axis]
                size = self.model.geom_size[i]
                if i in self.meshes:
                    extent = np.max(self.meshes[i] @ local_normal)
                elif self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_BOX:
                    extent = np.abs(local_normal) @ size
                elif self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                    extent = np.linalg.norm(local_normal[:2]) * size[0] + abs(local_normal[2]) * size[1]
                else:
                    extent = radius
                if coordinate + extent > self.wall_pos - 0.01:
                    return "左臂外形接近箱面"
        centers = self.data.geom_xpos
        offsets = centers[self.pair_left] - centers[self.pair_base]
        near = np.flatnonzero(np.einsum("ij,ij->i", offsets, offsets) <= self.pair_radius_squared)
        for index in near:
            i, base = self.pair_left[index], self.pair_base[index]
            gap = mujoco.mj_geomDistance(self.model, self.data, i, base, 0.01, None)
            if gap <= 0:
                arm = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
                if arm is None:
                    arm = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                           self.model.geom_bodyid[i])
                base_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, base)
                return f"左臂或探头与底座模型碰撞：{arm} / {base_name}，间距 {gap * 1000:.1f} mm"
        return None

    def segment_is_clear(self, start, end, resolution=0.0125):
        segments = max(1, math.ceil(np.max(np.abs(end - start)) / resolution))
        return all(self.issue(start + u * (end - start)) is None
                   for u in np.linspace(0.0, 1.0, segments + 1)[1:])


def _extend(tree, target, space):
    near = min(range(len(tree)), key=lambda i: np.linalg.norm(tree[i][0] - target))
    q = tree[near][0]
    distance = np.linalg.norm(target - q)
    if distance < 1e-9:
        return near, True
    reached = distance <= 0.25
    next_q = target if reached else q + (0.25 / distance) * (target - q)
    if not space.segment_is_clear(q, next_q):
        return None, False
    tree.append((next_q, near))
    return len(tree) - 1, reached


def _toward_root(tree, index):
    path = []
    while index is not None:
        q, index = tree[index]
        path.append(q)
    return path


def plan(space, start, end, label, via=()):
    """优先尝试直连和已知中间姿态，再做有预算的双向搜索。"""
    for name, q in (("起点", start), ("终点", end)):
        issue = space.issue(q)
        if issue:
            raise ControlFault(f"{label}{name}不可用：{issue}")
    if space.segment_is_clear(start, end):
        return np.array([start, end])
    for mid in via:
        if (space.issue(mid) is None and space.segment_is_clear(start, mid)
                and space.segment_is_clear(mid, end)):
            return np.array([start, mid, end])

    rng = np.random.default_rng(25)
    a, b = [(start, None)], [(end, None)]
    for attempt in range(1000):
        sample = b[0][0] if attempt % 6 == 0 else rng.uniform(space.lo, space.hi)
        ai, _ = _extend(a, sample, space)
        if ai is not None:
            while True:
                bi, reached = _extend(b, a[ai][0], space)
                if bi is None:
                    break
                if reached:
                    path = _toward_root(a, ai)[::-1] + _toward_root(b, bi)[1:]
                    if attempt % 2:
                        path.reverse()
                    # 合并能直接走通的连续段，再对最终路径加密复核。
                    short = [path[0]]
                    i = 0
                    while i < len(path) - 1:
                        j = len(path) - 1
                        while j > i and not space.segment_is_clear(path[i], path[j]):
                            j -= 1
                        if j == i:
                            break
                        short.append(path[j])
                        i = j
                    if i == len(path) - 1:
                        return np.array(short)
                    break
        a, b = b, a
    raise ControlFault(f"{label}找不到通过模型检查的路径")
