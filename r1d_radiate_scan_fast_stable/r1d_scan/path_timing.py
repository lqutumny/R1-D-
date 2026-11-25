"""沿固定关节路径的局部时间规划，不改变路径、不放宽关节限速。

变量 x=(ds/dt)^2；每格采用恒定路径加速度 u=(x[i+1]-x[i])/(2*ds)。
关节加速度 q_ss*x+q_s*u 对相邻 x 是线性的。通过稀疏 LP 选择局部速度，
再解析检查格内的关节速度/加速度极值。LP 最大化离散速度平方之和，
不宣称连续时间全局最优，也没有施加 jerk 约束。
"""
import numpy as np
from scipy.interpolate import BSpline, CubicSpline, make_lsq_spline
from scipy.optimize import linprog
from scipy.sparse import coo_matrix


class LocalPathMotion:
    def __init__(self, positions, q, speed, joint_speed, joint_acceleration, cycles=1,
                 endpoint_slopes=None):
        positions, q = np.asarray(positions, float), np.asarray(q, float)
        if (positions.ndim != 1 or len(positions) < 3 or q.ndim != 2
                or q.shape[0] != len(positions) or np.any(np.diff(positions) <= 0)
                or not np.isfinite(positions).all() or not np.isfinite(q).all()
                or not np.isfinite([speed, joint_speed, joint_acceleration]).all()
                or min(speed, joint_speed, joint_acceleration) <= 0
                or not isinstance(cycles, int) or cycles < 1):
            raise ValueError("局部时间规划的路径、速度或循环数不合法")
        bc = ("not-a-knot" if endpoint_slopes is None else
              ((1, endpoint_slopes[0]), (1, endpoint_slopes[1])))
        self.path = CubicSpline(positions, q, bc_type=bc)
        self.cycles = cycles
        # 每个路径样条节点都在规划网格内，保证后续格内极值分析使用单个三次多项式。
        self.grid = np.concatenate([np.linspace(a, b, max(2, int(np.ceil((b-a)/.001))+1))[:-1]
                                    for a, b in zip(positions, positions[1:])]
                                   + [positions[-1:]])
        ds = np.diff(self.grid)
        count, joints = len(self.grid), q.shape[1]
        qs = self.path(self.grid, 1)
        caps = np.minimum((speed*.98)**2,
                          np.min(np.divide((joint_speed*.98)**2, qs**2,
                                           out=np.full_like(qs, np.inf), where=np.abs(qs)>1e-12), axis=1))
        left, right = [], []
        for fraction in (0., .5, 1.):
            s = self.grid[:-1] + fraction*ds
            slope = self.path(s, 1)/(2*ds[:, None])
            curve = self.path(s, 2)
            left.append(curve*(1-fraction)-slope)
            right.append(curve*fraction+slope)
        left, right = np.concatenate(left, axis=1).ravel(), np.concatenate(right, axis=1).ravel()
        cells = np.repeat(np.arange(count-1), joints*3)
        rows = np.arange(len(cells))
        matrix = coo_matrix((np.r_[left, right, -left, -right],
                             (np.r_[rows, rows, rows+len(rows), rows+len(rows)],
                              np.r_[cells, cells+1, cells, cells+1])),
                            shape=(2*len(rows), count)).tocsr()
        bounds = np.column_stack((np.zeros(count), caps))
        bounds[[0, -1], 1] = 0.  # 两端停稳，往返与接近/返回都保持位置和速度连续。
        result = linprog(-np.ones(count), A_ub=matrix,
                         b_ub=np.full(matrix.shape[0], joint_acceleration*.98),
                         bounds=bounds, method="highs")
        if not result.success or result.x is None or not np.isfinite(result.x).all():
            raise ValueError(f"局部时间规划失败: {result.message}")
        x = np.maximum(result.x, 0.)
        self.path_velocity = np.sqrt(x)
        speed_sum = self.path_velocity[:-1] + self.path_velocity[1:]
        if np.any(speed_sum <= 1e-12):
            raise ValueError("路径局部无法通过，需要调整扫描范围或 IK 分支")
        self.path_acceleration = np.diff(x)/(2*ds)
        self.times = np.r_[0., np.cumsum(2*ds/speed_sum)]
        raw_v, raw_a = self._exact_extrema(x)
        # 修正离散 LP 在格内可能产生的小超限；大倍率表示应细化几何/网格而非静默变慢。
        self.certification_scale = max(1., raw_v/joint_speed,
                                       np.sqrt(raw_a/joint_acceleration)) * 1.000001
        if self.certification_scale > 1.05:
            raise ValueError("格内极值验证要求超过 5% 的修正，需要细化路径，未生成运动指令")
        self.one_way_duration = float(self.times[-1] * self.certification_scale)
        self.duration = self.one_way_duration * 2 * cycles
        self.peak_joint_speed = raw_v/self.certification_scale
        self.peak_joint_acceleration = raw_a/self.certification_scale**2
        self.peak_path_speed = float(self.path_velocity.max()/self.certification_scale)

    def _exact_extrema(self, x):
        """q_s 为二次式，x 为一次式；q_tt 为二次式，可求出全部速度/加速度极值。"""
        s, h = self.grid[:-1], np.diff(self.grid)[:, None]
        p, c, d = self.path(s, 1), self.path(s, 2), self.path(s, 3)
        u, x0 = self.path_acceleration[:, None], x[:-1, None]
        aa, bb, cc = 2.5*d*u, d*x0+3*c*u, c*x0+p*u
        max_v = max_a = 0.
        def evaluate(offset):
            nonlocal max_v, max_a
            offset = np.clip(np.broadcast_to(offset, p.shape), 0., h)
            velocity = (p+c*offset+.5*d*offset**2)*np.sqrt(np.maximum(0., x0+2*u*offset))
            acceleration = aa*offset**2+bb*offset+cc
            max_v = max(max_v, float(np.abs(velocity).max()))
            max_a = max(max_a, float(np.abs(acceleration).max()))
        evaluate(0.)
        evaluate(h)
        evaluate(np.divide(-bb, 2*aa, out=np.zeros_like(bb), where=np.abs(aa)>1e-14))
        discriminant = bb**2-4*aa*cc
        root = np.sqrt(np.maximum(0., discriminant))
        stable = -.5*(bb+np.where(bb >= 0., root, -root))
        for candidate in (np.divide(stable, aa, out=np.zeros_like(bb), where=np.abs(aa)>1e-14),
                          np.divide(cc, stable, out=np.zeros_like(bb), where=np.abs(stable)>1e-14)):
            candidate = np.where(discriminant >= 0., candidate, 0.)
            evaluate(candidate)
        evaluate(np.divide(-cc, bb, out=np.zeros_like(bb), where=np.abs(bb)>1e-14))
        return max_v, max_a

    def sample(self, t, derivative=0):
        if derivative not in (0, 1, 2):
            raise ValueError("只支持位置、速度和加速度")
        elapsed = np.clip(np.asarray(t, float), 0., self.duration)
        phase = np.remainder(elapsed, 2*self.one_way_duration)
        reverse = phase > self.one_way_duration
        local = np.where(reverse, 2*self.one_way_duration-phase, phase)/self.certification_scale
        cell = np.clip(np.searchsorted(self.times, local, side="right")-1, 0, len(self.times)-2)
        dt = local-self.times[cell]
        u, v = self.path_acceleration[cell], self.path_velocity[cell]
        s = np.clip(self.grid[cell]+v*dt+.5*u*dt**2, self.grid[0], self.grid[-1])
        if derivative == 0:
            return self.path(s)
        ds = (v+u*dt)/self.certification_scale
        if derivative == 1:
            return self.path(s, 1)*(ds*np.where(reverse, -1., 1.))[..., None]
        return (self.path(s, 2)*ds[..., None]**2
                + self.path(s, 1)*(u/self.certification_scale**2)[..., None])



class SmoothedTiming:
    """把"时间最优"轨迹的 bang-bang 速度曲线抹平，**空间路径完全不变**。

    为什么需要：线性规划给的是时间最优解，加速度在 ±a_max 之间硬切换。
    加速度阶跃 = jerk 无穷大，电机每次切换都"顿"一下，真机上就是可见/可听的卡顿。
    这里只重排时间轴：s(t) 变平滑，而 q = path(s) 仍落在同一条空间路径上，
    所以 TCP 直线度、离箱面距离这些几何指标一点都不会变。

    做法：按 1 ms 从原 motion 的表格还原 s(t) → 对 ds/dt 滑动平均 →
    缩放到总长度不变 → 重新积分成 s(t)。
    """

    def __init__(self, motion, window_s=0.15, accel_limit=None):
        self._m = motion
        self.duration = motion.duration
        self.one_way_duration = motion.one_way_duration

        dt = 0.001
        t = np.arange(0.0, self.one_way_duration + dt, dt)
        times = np.asarray(motion.times, float)
        grid = np.asarray(motion.grid, float)
        vel = np.asarray(motion.path_velocity, float)
        acc = np.asarray(motion.path_acceleration, float)
        cell = np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(times) - 2)
        h = t - times[cell]
        s = np.clip(grid[cell] + vel[cell] * h + 0.5 * acc[cell] * h ** 2, grid[0], grid[-1])

        v = np.gradient(s, dt)
        n = max(3, int(round(window_s / dt)) | 1)          # 奇数窗口
        # 用 Hann（升余弦）窗而不是方窗：方窗卷积在两端有折角，速度曲线仍会有拐点；
        # Hann 窗是 C∞ 的，抹出来的速度曲线没有折角，这才是"丝滑"的来源。
        kernel = np.hanning(n)
        kernel = kernel / kernel.sum()
        pad = n // 2
        v_pad = np.concatenate([np.full(pad, v[0]), v, np.full(pad, v[-1])])
        v_s = np.clip(np.convolve(v_pad, kernel, mode="valid"), 0.0, None)

        if v_s.sum() <= 0:
            raise ValueError("平滑后速度全为 0，窗口过大")
        s_new = np.concatenate([[0.0], np.cumsum(v_s[:-1]) * dt])
        s_new *= s[-1] / s_new[-1]                          # 保证走完全程

        self._t = t
        self._s = s_new
        self.peak_speed = float(v_s.max())

        # 平滑会把速度拉平 → 归一化后加速度反而可能超标。超了就按 sqrt 比例拉长时间轴
        # （加速度 ∝ 1/t²），用"稍慢一点"换"不顿挫"。
        if accel_limit:
            step = 0.001
            ts = np.arange(0.0, self.one_way_duration, step)
            qs_ = motion.path(np.interp(ts, self._t, self._s))
            ddq = np.diff(qs_, 2, axis=0) / step ** 2
            amax = float(np.abs(ddq).max())
            if amax > accel_limit:
                stretch = float(np.sqrt(amax / accel_limit))
                self._t = self._t * stretch
                self.one_way_duration *= stretch
                self.duration *= stretch
                self.stretch = stretch
                self.accel_before = amax
            else:
                self.stretch = 1.0
                self.accel_before = amax

    def sample(self, t, derivative=0):
        if derivative != 0:
            raise ValueError("平滑后的时间轴只支持位置采样")
        elapsed = np.clip(np.asarray(t, float), 0.0, self.duration)
        phase = np.remainder(elapsed, 2 * self.one_way_duration)
        local = np.where(phase > self.one_way_duration,
                         2 * self.one_way_duration - phase, phase)
        return self._m.path(np.interp(local, self._t, self._s))


class JerkLimitedTiming:
    """用连续的路径速度生成扫描时间轴，再按关节导数峰值留余量。

    速度用非负三次 B 样条表示，其积分给出路径位置。两端速度及其
    前两阶导数为零，所以往返、起停可直接拼接。限值用密集采样复核。
    """

    def __init__(self, positions, q, speed, joint_speed, joint_acceleration,
                 joint_jerk, fit_knots, window_s, cycles=1):
        if type(fit_knots) is not int or not 1 <= fit_knots <= len(q) - 6:
            raise ValueError("关节路径的五次样条节点数不合法")
        if not np.isfinite([joint_jerk, window_s]).all() or min(joint_jerk, window_s) <= 0:
            raise ValueError("jerk 限值和平滑窗口必须为正数")
        degree = 5
        knots = np.r_[np.repeat(positions[0], degree + 1),
                      np.linspace(positions[0], positions[-1], fit_knots + 2)[1:-1],
                      np.repeat(positions[-1], degree + 1)]
        self.path = make_lsq_spline(positions, q, knots, k=degree, axis=0)
        # LP 用三次路径近似产生速度初值。最终执行五次路径，
        # 下面用执行路径的导数重新计算时长。
        seed = LocalPathMotion(positions, self.path(positions), speed, joint_speed,
                               joint_acceleration, cycles)
        dt = 0.002
        seed_t = np.arange(0.0, seed.one_way_duration, dt)
        seed_v = np.interp(seed_t, seed.times * seed.certification_scale,
                           seed.path_velocity / seed.certification_scale)
        kernel = np.hanning(max(3, int(round(window_s / dt)) | 1))
        filtered = np.convolve(np.r_[seed_v, 0.0], kernel / kernel.sum())
        coeff = np.pad(filtered, (3, 3))
        time_knots = (np.arange(len(coeff) + 4) - 3) * dt
        self._speed = BSpline(time_knots, coeff, 3, extrapolate=False)
        self._integral = self._speed.antiderivative()
        self._accel = self._speed.derivative(1)
        self._jerk = self._speed.derivative(2)
        self._base_duration = float(time_knots[len(coeff)])
        self._offset = float(self._integral(0.0))
        self._begin, self._end = float(positions[0]), float(positions[-1])
        length = self._end - self._begin
        self._distance_scale = length / (self._integral(self._base_duration) - self._offset)

        times = np.linspace(0.0, self._base_duration,
                            int(np.ceil(self._base_duration / 0.0005)) + 1)
        peaks = [float(np.max(np.abs(self._joint_at(times, order)))) for order in (1, 2, 3)]
        path_speed = float(np.max(self._speed(times))) * self._distance_scale
        self.certification_scale = max(
            1.0, path_speed / speed, peaks[0] / joint_speed,
            (peaks[1] / joint_acceleration) ** 0.5,
            (peaks[2] / joint_jerk) ** (1 / 3)) * 1.02
        self.peak_path_speed = path_speed / self.certification_scale
        self.peak_joint_speed = peaks[0] / self.certification_scale
        self.peak_joint_acceleration = peaks[1] / self.certification_scale ** 2
        self.peak_joint_jerk = peaks[2] / self.certification_scale ** 3
        self.joint_speed_limit = joint_speed
        self.joint_acceleration_limit = joint_acceleration
        self.joint_jerk_limit = joint_jerk
        self.one_way_duration = self._base_duration * self.certification_scale
        self.duration = 2 * self.one_way_duration * cycles

    def _joint_at(self, t, derivative):
        s = self._begin + (self._integral(t) - self._offset) * self._distance_scale
        s = np.clip(s, self._begin, self._end)
        if derivative == 0:
            return self.path(s)
        v = (self._speed(t) * self._distance_scale)[..., None]
        q_s = self.path(s, 1)
        if derivative == 1:
            return q_s * v
        a = (self._accel(t) * self._distance_scale)[..., None]
        q_ss = self.path(s, 2)
        if derivative == 2:
            return q_ss * v ** 2 + q_s * a
        j = (self._jerk(t) * self._distance_scale)[..., None]
        return self.path(s, 3) * v ** 3 + 3 * q_ss * v * a + q_s * j

    def sample(self, t, derivative=0):
        if derivative not in (0, 1, 2, 3):
            raise ValueError("只支持位置、速度、加速度和 jerk")
        elapsed = np.clip(np.asarray(t, float), 0.0, self.duration)
        phase = np.remainder(elapsed, 2 * self.one_way_duration)
        reverse = phase > self.one_way_duration
        local = np.where(reverse, 2 * self.one_way_duration - phase, phase)
        local = np.clip(local / self.certification_scale, 0.0, self._base_duration)
        value = self._joint_at(local, derivative) / self.certification_scale ** derivative
        return value * np.where(reverse, -1.0, 1.0)[..., None] if derivative % 2 else value
