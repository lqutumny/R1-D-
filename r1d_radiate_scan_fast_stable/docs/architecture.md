# 代码结构与数据流

> 先读 [`README.md`](../README.md) 的"工作原理"三句话，再看这里的细节。

## 1. 两个进程，一条真实协议

```
        ┌──────────────────────────┐         rt/lowcmd (LowCmd_)         ┌────────────────────────┐
        │  scan.py                 │ ──────────────────────────────────▶ │  sim.py                │
        │  ├ controller.py         │     关节角 / KP / KD / 前馈力矩       │  ├ PhysicsBridge       │
        │  ├ runner.py  主循环      │                                     │  │  · CRC/mode 校验    │
        │  └ transport.py 编码+CRC  │ ◀────────────────────────────────── │  │  · PD → data.ctrl   │
        └──────────────────────────┘         rt/lowstate (LowState_)     │  │  · mj_step 1 ms     │
                    ▲                                                    │  └ 发布实测状态         │
                    │  真机时这个位置换成 R1-D 本体                          └────────────────────────┘
```

关键点：**`scan.py` 里没有任何"仿真专用"分支**。换真机只改 `--network`，
所以在仿真里验证过的保护项、状态机、时序，到真机上是同一份代码。

## 2. 模块职责

| 文件 | 职责 | 依赖 |
|---|---|---|
| `profile.py` | 读 `robot_profile.json`，校验槽位/KP-KD，`gaps()` 列出还不能上真机的原因 | 无 |
| `transport.py` | `LowCmd_`/`LowState_` 编解码、CRC、按槽位写信；`Snapshot` 是纯数据 | unitree_sdk2py |
| `controller.py` | 状态机 + 全部 Guard + 重力前馈；**不依赖 DDS**，可离线驱动 | MuJoCo（只为了算前馈） |
| `runner.py` | 控制主循环：取状态 → tick → 编码 → 发送。**不含 DDS**，publisher 可替换 | 上面三个 |
| `scan.py` | 入口：初始化 DDS、订阅状态、捕获初始姿态、跑 Runner、写报告 | 上面四个 |
| `sim.py` | 仿真器：MuJoCo 物理 + DDS 收发 | MuJoCo |
| `fake_robot.py` | 假机器人：只有"收到指令就到位"+ 坏状态注入，供测试 | unitree_sdk2py |
| `kinematics.py` | 7 轴 IK：主解（5 约束）+ 零空间精修 | numpy |
| `path_timing.py` | 把几何路径变成时间参数化轨迹（线性规划 + 极值复核） | numpy/scipy |
| `dds_network.py` | 网卡与 DDS 域初始化（`lo` 时写回环配置） | unitree_sdk2py |

依赖方向是单向的：`scan.py` → `runner` → `controller` → `transport`。
`controller` 与 `runner` 都不 import DDS，所以注入测试可以脱离网络跑。

## 3. 一轮扫描的时间线

`controller.target(now)` 用绝对时间决定阶段，`runner` 每 2 ms 调一次：

| 阶段 | 时长 | 目标 |
|---|---|---|
| `startup-hold` | 2.0 s | 保持捕获到的姿态（等链路稳定） |
| `approach` | 4–8 s（按限速/限加速度反算） | 五次曲线从初始姿态过渡到扫描起点 |
| `scan-up` | 15.73 s | 探头端面从 z=0.92 m 走到 1.42 m |
| `scan-down` | 15.73 s | 原路返回 |
| `return` | 同 approach | 五次曲线回到初始姿态 |
| `finished-hold` | 2.0 s（`--hold`） | 保持，然后退出 |

过渡段用**解析五次曲线**而不是查表——查表在采样点之间会造成速度尖峰（实测 0.48 rad/s）。

## 4. 控制律

```
τ = τ_ff(重力前馈) + Kp·(q_target − q_actual) − Kd·q̇
```

- **重力前馈**：用 MuJoCo 的 `mj_rne` 在目标姿态上算静态重力矩。kp=100 的 PD 撑不住 17 kg 的手臂，
  没有这一项，稳态误差会有好几度。
- **KP/KD**：来自 `robot_profile.json`，官方 A7 示例值（肩/肘 100，腕 50；Kd=2）。
- **模型阻尼**：`tools/tune_gains.py` 按目标阻尼比 ζ=0.8 反解出关节阻尼写回场景，
  这样"指令里的 Kd"保持官方值，仿真的动态特性由模型阻尼补齐。

## 5. 保护项在哪里（改代码时别破坏这个顺序）

```
runner.step(state, now)
  ├── controller.tick()          ← 全部 Guard 在这里，任何一个抛 ControlFault
  │     ├── 状态为空 / 超时 0.2 s / mode_pr≠0 / mode_machine 变
  │     ├── 故障字 / 温度 ≥70°C / 实测力矩 >110% effort
  │     ├── 目标超限位（余量 0.02 rad）/ 目标角速度 >0.44 rad/s
  │     ├── 预计力矩 >90% effort / 跟踪误差 >0.2 rad 持续 0.3 s
  │     └── 控制周期 ≤0 或 >70 ms
  ├── codec.encode()             ← 目标含 NaN 在这里被拦（最后一道防线）
  └── publisher.Write()          ← 只有前面都成功才发
```

**不变量**：任何 Guard 抛异常，这一拍就不发指令。机器人继续执行上一条指令（KP/KD 未清零），
也就是"停轨迹并保持最后目标"。退出码：0=走完整轮，2=保护触发，1=其它错误。

坏包（CRC/tick/NaN）不在这里处理——它们在 `Codec.decode` 里被拒，状态缓存停在上一帧有效值，
再由 0.2 s 超时保护兜底。所以链路级故障的停止延迟是 **200 ms**，直接故障是 **3–7 ms**。

## 6. 规划链

```
docs/hardware.md 站位卡（箱面 0.70 m、站距 0.20 m、可扫 z 0.92–1.48）
        │
        ▼  tools/scan_workspace.py   可达性扫描（关节余量、奇异、离箱面距离）
        ▼  tools/scan_plan.py        IK 逐点推进 → 101 点关节角 + 时间参数化
models/scan_profile.json（q_rad[101][7]、长度、峰值速度、限速限加速度）
        │
        ▼  controller.load_motion()  →  LocalPathMotion
scan.py 运行期采样
```

## 7. 想改东西，从哪下手

| 想改 | 改哪里 | 之后要做什么 |
|---|---|---|
| 行程 / 起止高度 | `tools/scan_plan.py` 的扫描线参数 | 重生成 `models/scan_profile.json` |
| 站位（箱面距离） | `tools/build_scene.py --wall-x` | 重建场景 + 重生成轨迹 + 重跑闭环 |
| 换探头（尺寸/质量） | `robot_profile.json` 的 `tool` | 重建场景 + 重生成轨迹 + 重跑闭环 |
| 换机型/槽位 | `robot_profile.json` 的 `joints`/`hold_on_robot` | `python3 r1d_scan/profile.py` 查缺口 |
| 保护阈值 | `controller.py` 的 `Limits` | 跑 `bash run_fault_tests.sh` |
| 右臂怎么保持 | `robot_profile.json` 的 `hold_on_robot` | `HOLD=servo bash run_demo.sh` 验证 |
| 控制频率 | `runner.py` 的 `period` | 注意虚拟机 `sleep` 粒度（见 docs/testing.md） |

## 8. 测试

| 测试 | 命令 | 覆盖 |
|---|---|---|
| 进程内 19 项 | `python3 tools/fault_injection.py --unit --codec` | 所有 Guard 的判据 + 解码层 |
| 进程级 8 项 | `python3 tools/fault_injection.py --dds` | 真 DDS 上"停了就真不发" |
| 仿真器对抗 2 项 | `python3 tools/fault_injection.py --sim` | 仿真器会拒坏包 |
| 全套 | `bash run_fault_tests.sh` | 上面全部，29 项 |

`tools/fault_injection.py` 的分层设计说明见 [`testing.md`](testing.md)。
