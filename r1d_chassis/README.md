# R1-D 集装箱箱面辐射扫描

> 本源码副本包含代码、必要说明和模型，现场数据与运行输出保存在原部署环境。

宇树 **R1-A7-D**（单臂 7 自由度）二次开发：用**左臂**带动辐射检测模块，
让探头端面在**一个竖直平面内上下扫动**（行程 50 cm），后期用辐射检测仪替换灵巧手。

> **状态**：手臂扫描真机已跑通（柔顺版 20.2 mm/s、直线度 5.5 mm）；保护项注入测试 30/30。
> **底盘**（阶段 6）仿真里已能按指令行走，真机未动，见 [`docs/chassis.md`](docs/chassis.md)。
> 真机阶段尚未开始——`robot_profile.json` 里的槽位与 KP/KD 仍是暂定值，**程序会拒绝在真机上启动**（见下）。


## 手臂与立柱联动仿真

在 Ubuntu 虚拟机桌面终端运行：

```bash
cd ~/r1d_chassis
MUJOCO_GL=glfw bash run_sync_sim.sh
```

从 SSH 启动图形窗口时加 `DISPLAY=:0`。只需记录数据时用
`NO_VIEWER=1 bash run_sync_sim.sh`。联动入口固定使用本机回环，
默认 DDS 域为 `217`。报告、手臂和立柱 CSV 保存在 `out/sync/`。

抬臂和收纳期间立柱停在低位。正式上扫约 16.22 s 内，左臂沿既有轨迹
上扫约 50 cm，立柱的 DDS 高度读数同步从 400 mm 升到 700 mm。换向后
两者同步返回，探头名义竖直覆盖约 80 cm。`models/sync_scan_profile.json` 来自
`r1d_radiate_scan_fast_stable` 的 16.22 s 方案。

联动仿真使用 `rt/cmd_vel` 的 `Twist_.linear.z` 发归一化升降速度，
用 `rt/height_state` 的 `Point32_.z` 毫米读数闭环，并读取
`rt/state_vel` 和 `rt/height_alarm`。仿真把机械低位 100 mm 上方的
5 mm 作为读数原点，台架刻度和真机偏移须另行核实。立柱是带小幅响应滞后的
运动学模型，不能用来判断真机的振动、停止距离或话题兼容性。

### 查看立柱高度反馈

联动仿真运行时，在另一个终端执行：

```bash
cd ~/r1d_chassis
python3 tools/read_lift_height.py --network lo --domain 217 --seconds 30
```

诊断工具只订阅，显示 `rt/height_state.z` 的毫米原始值，并统计速度和告警反馈。
仿真中 505–805 mm 的模型间隙对应 400–700 mm 的话题值。
真机可按现场 DDS 配置改用 `--network ens33 --domain 0`。
用 `--report out/lift_height.json` 可保留各话题的帧数、频率和测量范围。
手动遥控升降时可用 `--seconds 0` 持续查看，停稳后按 Ctrl+C 结束并保存报告。
真机机身到底盘板的间隙需量尺标定参考偏移。

### 真机联动入口

先用遥控器将立柱停在 **DDS 读数 400 ± 10 mm**。将底盘停稳，让手臂进入
原扫描项目使用的开发模式。启动前可运行只读预检：

```bash
cd ~/r1d_chassis
NETWORK=ens33 DOMAIN=0 bash run_sync_real.sh --dry-run
```

预检通过后，使用 `NETWORK=ens33 DOMAIN=0 bash run_sync_real.sh` 并确认启动。
程序根据当前左臂姿态规划到 50 cm 扫描线下端，停稳后和立柱同步上扫、换向和下扫。
立柱回到低位并停稳后，手臂将探头竖直收纳。报告位于 `out/sync_real_*/`。

400 和 700 mm 是真机 DDS 原始读数。组合几何预检暂以 105 mm 为模型偏移，
实际机身到底盘板的参考偏移仍需测量。测得后可以用 `MODEL_OFFSET_MM` 设置，
控制目标仍以 DDS 原始读数为准。真机的自动升降响应、制动和同步到位还需实测。

## 这个项目解决什么问题

被控对象不是"手腕"，而是"探头端面"：手腕和探头之间差一个沿工具轴的偏移，
而且探头朝向必须固定，否则"点走直线"会变成"端面画弧"。所以：

- 约束是 **5 个**（TCP 位置 3 + 探头轴线 2），而左臂有 **7 个**自由度 → 2 个冗余自由度用来躲限位、避奇异。
- 7 轴臂的逆解不唯一，逐点独立求解会让腕部在零空间里漂移（实测漂了 72°）。这里用**微分连续推进**
  （阻尼最小二乘 + 量纲加权）保证相邻点解连续。
- "10 cm 精度"拆成两个数：**直线度 20 mm**（代码的责任）+ **绝对位置 100 mm**（站位与标定的责任）。


## 目录结构

```
r1d_radiate_scan/
├── docs/
│   ├── install.md            # 环境搭建（Ubuntu 22.04 / MuJoCo / unitree_sdk2py）
│   ├── architecture.md       # 代码结构、数据流、控制回路、如何扩展
│   ├── hardware.md           # 机型/槽位/限位/探头参数 + 站位卡与轨迹指标
│   ├── testing.md            # 保护项清单与注入测试 + 真机核对作业单
│   └── results.md            # 需求对照（验收结论）+ 五次真机试验记录
├── models/
│   ├── r1_a7.urdf            # 官方 R1-A7 URDF（unitree_ros）
│   ├── meshes/               # 官方 STL（19 个）
│   ├── r1_a7_scene.xml       # 由 build_scene.py 生成：整机 + 探头 + 箱面
│   ├── chassis.json          # 底盘限幅与假设参数（体积/质量待实测）
│   ├── r1_a7_scene_mobile.xml # 移动底盘场景（build_scene.py --mobile 生成）
│   ├── scan_profile.json     # 101 点扫描轨迹（关节角序列）
│   └── simulation.json       # 场景清单：nq/nu、锁了哪些关节、执行器力矩上限
├── r1d_scan/                 # 运行时库
│   ├── robot_profile.json    # 关节名/槽位/KP-KD/保持关节（真机前必须核对）
│   ├── profile.py            # profile 读取与校验
│   ├── kinematics.py         # 7 轴 IK：微分连续推进 + 量纲加权
│   ├── path_timing.py        # 路径时间参数化（LP + 极值复核）
│   ├── controller.py         # 状态机 + 全部 Guard
│   ├── runner.py             # 控制主循环（不含 DDS，可单测）
│   ├── transport.py          # LowCmd/LowState 编解码 + CRC
│   ├── scan.py               # 控制程序（唯一接 DDS 的运动入口）
│   ├── sim.py                # MuJoCo 仿真器（订阅 lowcmd、发布 lowstate）
│   ├── fake_robot.py         # 假机器人：按需注入坏状态
│   ├── chassis.py            # 底盘：agv 服务封装 + 限幅 + 看门狗
│   ├── lowstate.py           # LowState → 字典（离线分析用）
│   └── dds_network.py        # 网卡与 DDS 域初始化
├── tools/
│   ├── build_scene.py        # URDF → MuJoCo 场景（含保持关节自检）
│   ├── scan_workspace.py     # 可达性/站位扫描
│   ├── scan_plan.py          # 生成扫描轨迹 → models/scan_profile.json
│   ├── tune_gains.py         # 按目标阻尼比标定模型阻尼
│   ├── measure_step.py       # 从 STEP 文件量探头外形（不需要 CAD）
│   ├── check_urdf.py         # URDF 自检
│   ├── inspect_device.py     # 真机只读采集（窗口内统计；不 import 任何 publisher）
│   ├── analyze_device.py     # 采集结果 → 六列对照表草稿
│   ├── watch_lowcmd.py       # 接管前检查通道上有没有别人在发指令
│   ├── nudge.py              # 分级接管：抓取 / 单关节微动
│   ├── check_run.py          # 运行后复盘：逐关节指令 vs 实测
│   ├── tcp_from_csv.py       # 用实测关节角反算 TCP 轨迹与直线度
│   ├── chassis_demo.py       # 底盘动作演示（走直线/转向）
│   ├── test_chassis.py       # 底盘通道测试（限幅/服务缺失/端到端）
│   ├── repeatability.py      # 多轮运行之间的散差
│   ├── fault_injection.py    # 保护项注入测试（三层）
│   └── fetch_urdf.sh         # 官方 URDF 下载脚本（记录模型来源）
├── run_demo.sh               # 仿真跑一轮
├── run_real.sh               # 真机一条龙（检查 → 抓取 → 扫描 → 汇总）
└── run_fault_tests.sh        # 全套保护项注入测试
```

`out/` 是运行产物目录（报告、CSV、截图），已在 `.gitignore` 里，不提交。

## 快速开始

环境见 [`docs/install.md`](docs/install.md)。装好后：

```bash
bash run_demo.sh                 # 仿真里跑完整一轮（约 70 s，弹窗口）
bash run_fault_tests.sh          # 保护项注入测试（约 110 s，20 项）
bash run_real.sh                 # 真机：现场检查 → 预检 → 抓取 → 扫描 → 汇总
PASSES=3 bash run_real.sh        # 真机连扫 3 趟

# 只预检不动：用机器人实测姿态把整轮时间轴离线走一遍，一条指令都不发
python3 r1d_scan/scan.py --network ens33 --domain 0 --dry-run --report out/pre.json
```

底盘（阶段 6，目前只在仿真）。**升降柱伸缩**（实际要做的）：

```bash
python3 tools/build_scene.py --lift                 # 一次性：生成伸缩场景
python3 r1d_scan/sim.py --network lo --domain 215 --lift   # 终端 1
python3 tools/lift_demo.py --network lo --domain 215 \
    --plan up:0.10,wait:1,down:0.05                  # 终端 2：升 10 cm 再降 5 cm
python3 tools/test_lift.py                          # 12 项，18 s
```

底盘平移（`Move`，顺带做的，暂不作为主线）：

```bash
python3 tools/build_scene.py --mobile              # 一次性：生成移动底盘场景
python3 r1d_scan/sim.py --network lo --mobile      # 终端 1：仿真器托管 agv 服务
python3 tools/chassis_demo.py --network lo --plan forward:0.4,left:90,forward:0.4   # 终端 2
python3 tools/test_chassis.py                      # 13 项通道测试，17 s
```

`run_demo.sh` 会自己拉起 MuJoCo 仿真器、等它就绪、跑控制器、收尾关窗。期望输出：

```
阶段： startup-hold(0.0s) → approach(2.0s) → scan-up(7.6s) → scan-down(43.9s)
       → return(80.2s) → finished-hold(85.7s)
指令 3.7 万条，最大跟踪误差 0.023 rad，complete=True
最终关节角(deg)： [10.0, 0.0, 0.0, 60.0, 0.0, 0.0, 0.0]
```

`complete=True` + 最终关节角回到 `[10,0,0,60,...]°` = 这一轮走完整、回到了初始下垂姿态。

## 工作原理（三段）

1. **规划**：`scan_plan.py` 用 `kinematics.py` 把"探头端面沿竖直线走 50 cm"翻译成 7 个关节的角度序列
   （101 个节点），再用 `path_timing.py` 给它打时间戳（单程 21.6 s，运行时约 24.6 s）。
2. **控制**：`scan.py` 按 2 ms 周期把关节角写进 `rt/lowcmd`（HG 协议，含 CRC），
   同时订阅 `rt/lowstate` 拿实测角做闭环。状态机：保持 → 接近 → 上行 → 下行 → 回位 → 保持。
3. **仿真**：`sim.py` 就是这个"机器人"——它订阅 `rt/lowcmd`，按 PD 力矩驱动 MuJoCo，发布 `rt/lowstate`。
   两个进程走真实的 DDS 协议，所以**换成真机只是把 `--network lo` 换成网卡名**。

`scan.py` 与真机之间没有任何"仿真专用"分支；保护项（超时/CRC/故障字/温度/限位/角速度/力矩/跟踪误差/
模式变化）在仿真里全部被真实触发过，见 [`docs/testing.md`](docs/testing.md)。

## 真机安全声明

- `robot_profile.json` 的 `slot_source` 现为 `verified`（槽位已在真机只读核对过）。
  `scan.py` 在**非 `lo` 网卡**上仍会检查这一项，未核对就直接退出，**一条指令都不会发**。
- `--dry-run` 是同一份代码的"只规划"模式：用机器人实测姿态把整轮时间轴离线走完，
  报告里 `published_commands` 恒为 0。`run_real.sh` 已把它放在抓取之前做闸门。
- 真机前必须完成只读核对（`tools/inspect_device.py` 只订阅 `rt/lowstate`，不含任何 publisher）：
  槽位归属、关节方向、限位、`mode_machine`、腰 yaw 的槽位。
- 保护触发时的行为是**停止发送、不清零 KP/KD**（机器人保持最后一条指令），不是急停。
  现场必须另有硬件急停，并由专人负责。
- 扫描时人员不得进入机器人工作范围。

## 已知限制

| 项 | 现状 | 说明 |
|---|---|---|
| 探头模型 | 真件 126×126×190、探测面 Ø47、1.5 kg | 质量与质心仍是估值，用户决定不再核对 |
| 指令频率 | 309–350 Hz | 虚拟机 `time.sleep` 粒度所限；原生 Linux 预期接近 500 Hz |
| 力矩预算 | URDF effort × 0.9 | 实测峰值：肩 pitch 9.5/60（16%）、腕 pitch 1.9/10（19%）→ 不是瓶颈 |
| 左腕 pitch 限位 | **±43°（实测硬限位）** | URDF 的 ±92.5° 作废；决定站位与速度 |
| 扫描速度 | **实测 0.0202 m/s**（24.6 s 单程） | 已进入 0.02–0.05 区间下沿；提速会放大轴线误差 |
| 精度 | 真机柔顺版 **5.5 mm 直线度 / 8.05° 轴线**（旧轨迹 4.8 / 5.77） | 轴线余量只剩 2°；绝对定位未测（用户判定无需） |
| 重复性 | 三轮散差 2.9 mm | 通过 |
| 底盘 | 仿真运动学模型（无轮地摩擦/惯性耦合） | 真机未动；轮径轮距质量未实测 |

## 文档索引

| 想了解 | 看 |
|---|---|
| 环境怎么搭、常见报错 | [`docs/install.md`](docs/install.md) |
| 代码怎么组织、想改哪里 | [`docs/architecture.md`](docs/architecture.md) |
| 机型参数、槽位、右臂怎么保持 | [`docs/hardware.md`](docs/hardware.md) |
| 站位、可达范围、轨迹指标 | [`docs/hardware.md`](docs/hardware.md) |
| 探头怎么装的、实测尺寸 | [`docs/hardware.md`](docs/hardware.md) §7 |
| 真机核对怎么一步步做 | [`docs/testing.md`](docs/testing.md) |
| 保护项怎么测的、停多快 | [`docs/testing.md`](docs/testing.md) |
| 底盘怎么动、接口是哪几个 | [`docs/chassis.md`](docs/chassis.md) |

## 数据来源

- 机器人模型：`unitreerobotics/unitree_ros` → `robots/r1_a7_description/r1_a7.urdf`（+ 对应 mesh）。
- 低层协议：`unitree_sdk2py`（HG 系列 `rt/lowcmd` / `rt/lowstate`，含 CRC 与 `mode_machine` 回显）。
- 仿真器：MuJoCo 3.17。
