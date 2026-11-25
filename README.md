# R1-D 左臂扫描与立柱联动

| 目录 | 用途 |
| --- | --- |
| [r1d_radiate_scan_fast_stable](./r1d_radiate_scan_fast_stable/) | 左臂独立扫描，按当前姿态规划到 50 cm 扫描线，扫描后竖直收纳 |
| [r1d_chassis](./r1d_chassis/) | 左臂与升降柱联动扫描，立柱 DDS 读数目标 400 → 700 → 400 mm |

两套代码可独立使用，运行环境及依赖见各自项目的说明。
仓库包含运行需要的 MuJoCo 场景、关节配置和机器人网格。现场数据和运行输出留在部署机器上。

## 左臂独立仿真

```bash
cd r1d_radiate_scan_fast_stable
MUJOCO_GL=glfw bash run_demo.sh
```

## 左臂与立柱联动仿真

从本仓库根目录执行：

```bash
cd r1d_chassis
MUJOCO_GL=glfw bash run_sync_sim.sh
```

真机只读预检与启动命令见[联动项目说明](./r1d_chassis/README.md#真机联动入口)。
