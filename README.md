# R1-D 集装箱箱面辐射扫描

宇树 **R1-A7-D**（单臂 7 自由度）二次开发：用**左臂**带动辐射检测模块，
让探头端面在**一个竖直平面内上下扫动**（行程 50 cm），后期用辐射检测仪替换灵巧手。

> **状态**：仿真闭环已跑通（MuJoCo + DDS 真实协议回环），保护项注入测试 29/29 通过。
> 真机阶段尚未开始——`robot_profile.json` 里的槽位与 KP/KD 仍是暂定值，**程序会拒绝在真机上启动**（见下）。

## 这个项目解决什么问题

被控对象不是"手腕"，而是"探头端面"：手腕和探头之间差一个沿工具轴的偏移，
而且探头朝向必须固定，否则"点走直线"会变成"端面画弧"。所以：

- 约束是 **5 个**（TCP 位置 3 + 探头轴线 2），而左臂有 **7 个**自由度 → 2 个冗余自由度用来躲限位、避奇异。
- 7 轴臂的逆解不唯一，逐点独立求解会让腕部在零空间里漂移（实测漂了 72°）。这里用**微分连续推进**
  （阻尼最小二乘 + 量纲加权）保证相邻点解连续。
- "10 cm 精度"拆成两个数：**直线度 20 mm**（代码的责任）+ **绝对位置 100 mm**（站位与标定的责任）。

详细的工程判断写在 [`TASKS.md`](TASKS.md)（六条工作思路、五阶段、状态表）。

## 目录结构

```
r1d_radiate_scan/
├── TASKS.md                  # 工作思路、五阶段、状态表（唯一跟踪表）
├── docs/
│   ├── install.md            # 环境搭建（Ubuntu 22.04 / MuJoCo / unitree_sdk2py）
│   ├── architecture.md       # 代码结构、数据流、控制回路、如何扩展
│   ├── device.md             # 机型事实：关节、限位、effort、DDS 槽位、右臂保持做法
│   ├── workspace.md          # 站位卡、可扫 z 范围、轨迹指标
│   ├── joint_map.md          # 阶段 5a 现场作业单：六列表怎么填、三条硬标准
│   ├── faults.md             # 保护项清单 + 注入测试证据
│   └── real_trials.md        # 真机试验记录（仿真数据不混入）
├── models/
│   ├── r1_a7.urdf            # 官方 R1-A7 URDF（unitree_ros）
│   ├── meshes/               # 官方 STL（19 个）
│   ├── r1_a7_scene.xml       # 由 build_scene.py 生成：整机 + 探头假件 + 箱面
│   ├── scan_profile.json     # 101 点扫描轨迹（关节角序列）
│   ├── simulation.json       # 场景清单：nq/nu、锁了哪些关节、执行器力矩上限
│   └── urdf_check.json       # URDF 自检结果（限位/effort/臂长）
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
│   ├── lowstate.py           # LowState → 字典（离线分析用）
│   └── dds_network.py        # 网卡与 DDS 域初始化
├── tools/
│   ├── build_scene.py        # URDF → MuJoCo 场景（含保持关节自检）
│   ├── scan_workspace.py     # 可达性/站位扫描
│   ├── scan_plan.py          # 生成扫描轨迹 → models/scan_profile.json
│   ├── tune_gains.py         # 按目标阻尼比标定模型阻尼
│   ├── check_urdf.py         # URDF 自检
│   ├── measure_step.py       # 从 STEP 文件量探头外形（不需要 CAD）
│   ├── fault_injection.py    # 保护项注入测试（三层）
│   ├── inspect_device.py     # 真机只读采集（窗口内统计；不 import 任何 publisher）
│   ├── analyze_device.py     # 采集结果 → 六列对照表草稿
│   └── fetch_urdf.sh         # 官方 URDF 下载脚本（记录模型来源）
├── run_demo.sh               # 一条命令跑完整一轮（仿真）
└── run_fault_tests.sh        # 一条命令跑全套保护项注入测试
```

`out/` 是运行产物目录（报告、CSV、截图），已在 `.gitignore` 里，不提交。

## 快速开始

环境见 [`docs/install.md`](docs/install.md)。装好后：

```bash
bash run_demo.sh                 # 仿真里跑完整一轮（约 86 s，弹窗口）
bash run_fault_tests.sh          # 保护项注入测试（约 110 s，30 项）
bash run_real.sh                 # 真机：现场检查 → 抓取 → 扫描 → 汇总
PASSES=3 bash run_real.sh        # 真机连扫 3 趟
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
   （101 个节点），再用 `path_timing.py` 给它打时间戳（单程 15.73 s）。
2. **控制**：`scan.py` 按 2 ms 周期把关节角写进 `rt/lowcmd`（HG 协议，含 CRC），
   同时订阅 `rt/lowstate` 拿实测角做闭环。状态机：保持 → 接近 → 上行 → 下行 → 回位 → 保持。
3. **仿真**：`sim.py` 就是这个"机器人"——它订阅 `rt/lowcmd`，按 PD 力矩驱动 MuJoCo，发布 `rt/lowstate`。
   两个进程走真实的 DDS 协议，所以**换成真机只是把 `--network lo` 换成网卡名**。

`scan.py` 与真机之间没有任何"仿真专用"分支；保护项（超时/CRC/故障字/温度/限位/角速度/力矩/跟踪误差/
模式变化）在仿真里全部被真实触发过，见 [`docs/faults.md`](docs/faults.md)。

## 真机安全声明

- `robot_profile.json` 的 `slot_source` 目前是 `provisional`（暂定槽位）。
  `scan.py` 在**非 `lo` 网卡**上启动时会检查这一项，未核对就直接退出，**一条指令都不会发**。
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
| 力矩预算 | URDF effort × 0.9 | 真机 `tau_est` 实测近 0，只有"预计力矩"有效 |
| 左腕 pitch 限位 | **±43°（实测硬限位）** | URDF 的 ±92.5° 作废；决定站位与速度 |
| 扫描速度 | 0.014 m/s | 低于 0.02–0.05；腕限位收窄 + 为平滑降加速度所致 |
| 精度 | 真机 4.8 mm 直线度 / 5.8° 轴线 | 绝对定位未测（用户判定无需） |
| 重复性 | 三轮散差 2.9 mm | 通过 |

## 文档索引

| 想了解 | 看 |
|---|---|
| 为什么这么设计、还差什么 | [`TASKS.md`](TASKS.md) |
| 环境怎么搭、常见报错 | [`docs/install.md`](docs/install.md) |
| 代码怎么组织、想改哪里 | [`docs/architecture.md`](docs/architecture.md) |
| 机型参数、槽位、右臂怎么保持 | [`docs/device.md`](docs/device.md) |
| 站位、可达范围、轨迹指标 | [`docs/workspace.md`](docs/workspace.md) |
| 探头怎么装的、实测尺寸 | [`docs/device.md`](docs/device.md) §7 |
| 真机核对怎么一步步做 | [`docs/joint_map.md`](docs/joint_map.md) |
| 保护项怎么测的、停多快 | [`docs/faults.md`](docs/faults.md) |
| 真机跑成什么样、还有哪些没做 | [`docs/acceptance.md`](docs/acceptance.md) |

## 数据来源

- 机器人模型：`unitreerobotics/unitree_ros` → `robots/r1_a7_description/r1_a7.urdf`（+ 对应 mesh）。
- 低层协议：`unitree_sdk2py`（HG 系列 `rt/lowcmd` / `rt/lowstate`，含 CRC 与 `mode_machine` 回显）。
- 仿真器：MuJoCo 3.17。
