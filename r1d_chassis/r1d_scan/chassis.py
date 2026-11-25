"""底盘（AGV）与升降柱：官方 `agv` 服务的一层薄封装，加限幅和看门狗。

## 为什么底盘要单独一个模块

`r1_a7.urdf` 里没有底盘、没有轮子——整份模型只有 pelvis + 腰 + 双臂 + 头（17 个 revolute）。
真机上的底盘是另一块板子，**它的轮子不在 `rt/lowcmd` 的电机槽里**。

所以底盘和手臂走的是两条互不相干的通道：

| | 手臂 | 底盘 |
|---|---|---|
| 通道 | `rt/lowcmd` / `rt/lowstate`（低层电机指令） | RPC 服务 `agv`（请求/应答） |
| 谁能发 | 我们的 `scan.py`（独占） | 官方 AGV 固件 |
| 冲突 | 两个写者会抢通道 | 不占 `lowcmd`，可以和手臂并存 |

这也是当初 `sim.py` 把底盘锁死、不建模的原因：低层接管拿不到底盘。

## 三个接口事实（查 `g1_agv_api.hpp` / `g1_agv_client.hpp` 确认，不是猜的）

1. 服务名 `agv`，API 版本 `1.0.0.1`。
2. `1001 = Move(vx, vy, vyaw)`，参数 JSON `{"vx","vy","vyaw"}`，单位 m/s、m/s、rad/s。
3. `1002 = HeightAdjust(vz)`，参数 JSON `{"data": vz}`，而 vz 是**归一化** [-1, 1]，
   乘 76.5 mm/s 才是立柱速度。**不是米制速度**——这里写错会让立柱以 20 倍速冲出去。

## 关于"有响应才算成功"

服务调用只证明"请求被服务端收下了"，不证明底盘真的在走。所以这里额外记 `sent` 计数和
最后一次返回码；真机上的第一步永远是零速探针（见 `probe()`），确认服务存在再给速度。
"""
from dataclasses import dataclass
import json
import threading
import time
from pathlib import Path

AGV_SERVICE = "agv"
AGV_API_VERSION = "1.0.0.1"
ID_MOVE = 1001
ID_HEIGHT_ADJUST = 1002

ROOT = Path(__file__).resolve().parents[1]
LIMITS_PATH = ROOT / "models" / "chassis.json"


@dataclass
class Limits:
    max_vx: float = 0.20
    max_vy: float = 0.10
    max_w: float = 0.40
    watchdog: float = 0.5
    max_vz: float = 0.5
    height_enabled: bool = False
    height_range: tuple = (0.40, 0.90)      # 可用间隙（离底盘），已扣掉手臂磕碰段
    height_mechanical: tuple = (0.10, 0.90)  # 机械行程，仅记录

    @classmethod
    def load(cls, path=LIMITS_PATH):
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        height = raw.get("height", {})
        return cls(max_vx=float(raw["max_vx_m_s"]), max_vy=float(raw["max_vy_m_s"]),
                   max_w=float(raw["max_w_rad_s"]),
                   watchdog=float(raw.get("watchdog_s", 0.5)),
                   max_vz=float(height.get("max_vz_normalized", 0.5)),
                   height_enabled=bool(height.get("enabled", False)),
                   height_range=tuple(height.get("usable_gap_m", (0.40, 0.90))),
                   height_mechanical=tuple(height.get("mechanical_gap_m", (0.10, 0.90))))


class _AgvClient:
    """照 `g1_agv_client.hpp` 用 Python RPC 重写的一份：只有 Move 和 HeightAdjust。"""

    def __init__(self):
        from unitree_sdk2py.rpc.client import Client

        class _Client(Client):
            def __init__(self):
                super().__init__(AGV_SERVICE, False)

            def Init(self):
                self._SetApiVerson(AGV_API_VERSION)
                self._RegistApi(ID_MOVE, 0)
                self._RegistApi(ID_HEIGHT_ADJUST, 0)

            def Move(self, vx, vy, vyaw):
                code, _ = self._Call(ID_MOVE, json.dumps({"vx": vx, "vy": vy, "vyaw": vyaw}))
                return code

            def HeightAdjust(self, vz):
                code, _ = self._Call(ID_HEIGHT_ADJUST, json.dumps({"data": vz}))
                return code

        self._impl = _Client()
        self._impl.Init()

    def Move(self, vx, vy, vyaw):
        return self._impl.Move(vx, vy, vyaw)

    def HeightAdjust(self, vz):
        return self._impl.HeightAdjust(vz)

    def ServerApiVersion(self):
        """只读：向服务端要 API 版本。这个方法不发任何控制指令。"""
        return self._impl.GetServerApiVersion()


class Chassis:
    """底盘速度控制。所有命令都先限幅，再由看门狗兜底。

    用法（仿真和真机同一份代码）：

        Chassis(dry_run=True)          # 只算不发（预检）
        Chassis().drive(0.15, 0, 0, 2) # 前进 0.15 m/s 走 2 s，自动停
    """

    def __init__(self, limits=None, dry_run=False, auto_watchdog=True):
        self.limits = limits or Limits.load()
        self.dry_run = bool(dry_run)
        self.sent = 0
        self.last_code = None
        self.last_move_at = None
        self.clamped = 0
        self._lock = threading.Lock()
        self._client = None if self.dry_run else _AgvClient()
        self._stop_watchdog = threading.Event()
        self._watchdog = None
        if auto_watchdog and not self.dry_run and self.limits.watchdog > 0:
            self._watchdog = threading.Thread(target=self._watch, daemon=True)
            self._watchdog.start()

    # ---------- 基础命令 ----------
    def clamp(self, vx, vy, vyaw):
        lim = self.limits
        cx = max(-lim.max_vx, min(lim.max_vx, float(vx)))
        cy = max(-lim.max_vy, min(lim.max_vy, float(vy)))
        cw = max(-lim.max_w, min(lim.max_w, float(vyaw)))
        if (cx, cy, cw) != (float(vx), float(vy), float(vyaw)):
            self.clamped += 1
        return cx, cy, cw

    def move(self, vx, vy=0.0, vyaw=0.0):
        cx, cy, cw = self.clamp(vx, vy, vyaw)
        with self._lock:
            self.last_move_at = time.monotonic()
        if self.dry_run:
            return 0
        self.last_code = self._client.Move(cx, cy, cw)
        self.sent += 1
        return self.last_code

    def stop(self):
        with self._lock:
            self.last_move_at = None
        if self.dry_run:
            return 0
        self.last_code = self._client.Move(0.0, 0.0, 0.0)
        self.sent += 1
        return self.last_code

    def height(self, vz):
        """vz 是归一化 [-1, 1]，不是 m/s。"""
        if not self.limits.height_enabled:
            return -9  # 明确拒绝，而不是"发出去但没作用"
        c = max(-self.limits.max_vz, min(self.limits.max_vz, float(vz)))
        if self.dry_run:
            return 0
        self.last_code = self._client.HeightAdjust(c)
        self.sent += 1
        return self.last_code

    def lift(self, vz, seconds, hz=20.0):
        """持续发同一个归一化速度 seconds 秒，结束时发 0。返回指令条数。

        注意这是**速度**指令：真机不报立柱位置，所以"走多少米"只能按
        速度×时间开环算。到行程端头由固件自己顶住（仿真里会记 limit_hits）。
        """
        period = 1.0 / hz
        end = time.monotonic() + float(seconds)
        n = 0
        try:
            while time.monotonic() < end:
                self.height(vz)
                n += 1
                time.sleep(period)
        finally:
            self.height(0.0)
        return n

    def travel(self, meters, up=True, vz=None, hz=20.0):
        """开环走 meters 米（正=上升）。返回 (指令条数, 预计秒数)。"""
        vz = self.limits.max_vz if vz is None else abs(vz)
        speed = abs(vz) * 0.0765
        seconds = abs(meters) / speed
        n = self.lift(abs(vz) if up else -abs(vz), seconds, hz=hz)
        return n, seconds

    # ---------- 组合动作 ----------
    def drive(self, vx, vy, vyaw, seconds, hz=20.0):
        """持续发同一速度指令 seconds 秒，结束时一定停。返回实际发出的指令条数。"""
        period = 1.0 / hz
        end = time.monotonic() + float(seconds)
        n = 0
        try:
            while time.monotonic() < end:
                self.move(vx, vy, vyaw)
                n += 1
                time.sleep(period)
        finally:
            self.stop()
        return n

    def probe(self):
        """零速探针：确认 `agv` 服务在这台机器上存在。不会让底盘动。

        返回 (ok, code)。code=0 表示服务端收下了；非 0 多半是服务不存在或固件没进
        AGV 模式（超时返回 3001 之类）。这一步应该在给任何非零速度之前做。
        """
        code = self.stop()
        return code == 0, code

    def server_version(self):
        """只读探针：`agv` 服务在不在、版本是多少。

        比"发一条零速指令"干净：它只走 RPC 的内部版本查询，不动任何执行器。
        返回 (code, version)。服务不存在时一般是 send error（3102）或超时（3104）。
        """
        if self.dry_run or self._client is None:
            return 0, "dry-run"
        return self._client.ServerApiVersion()

    def probe_height(self):
        """升降柱零速探针：确认 1002 被服务端接受。"""
        code = self.height(0.0)
        return code == 0, code

    # ---------- 看门狗 ----------
    def _watch(self):
        # 客户端侧兜底：超过 watchdog 没再发 move，就补一条零速。
        # 注意这只是"客户端不再催"，真机上底盘固件自己也有超时停止，两层都要有。
        limit = self.limits.watchdog
        while not self._stop_watchdog.wait(0.05):
            with self._lock:
                last = self.last_move_at
            if last is None or self.dry_run:
                continue
            if time.monotonic() - last > limit:
                self.stop()

    def close(self):
        self._stop_watchdog.set()
        try:
            self.stop()
        except Exception:  # noqa: BLE001
            pass

    def summary(self):
        return {"dry_run": self.dry_run, "sent": self.sent, "clamped": self.clamped,
                "last_code": self.last_code,
                "limits": {"vx": self.limits.max_vx, "vy": self.limits.max_vy,
                           "w": self.limits.max_w, "watchdog_s": self.limits.watchdog},
                "height": {"enabled": self.limits.height_enabled,
                           "max_vz": self.limits.max_vz,
                           "usable_gap_m": list(self.limits.height_range),
                           "mechanical_gap_m": list(self.limits.height_mechanical),
                           "column_max_m_s": 0.0765}}