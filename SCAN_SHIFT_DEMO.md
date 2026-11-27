# 第一列上扫、顶部换列、第二列下扫 Demo

更新时间：2026-11-27

## 当前状态

代码、针对性回环和首轮真机演示已经完成，**仿真通过、真机通过**。流程完整执行：自动归位到330 mm、第一列上扫到830 mm、顶部保持、正 `linear.x` 换列、连续全零Twist停稳0.5秒、第二列下扫到330 mm、收纳。负责人现场确认正 `linear.x` 会让机器人向前移动，正式上扫—换列—下扫流程已成功跑通一轮。

仿真顶部为831.540 mm，第二列底部为328.968 mm；两列模型反算覆盖分别为1090.24 mm和1091.97 mm。换列反馈峰值为0.05，停稳时前后、左右和旋转反馈均为0；`complete=true`、`fault=null`、无DDS过期。证据位于Ubuntu `~/r1d_chassis/out/scan_shift_scan_sim_20261127/`。

程序复用现有600 mm、约51 mm/s轨迹和330↔830 mm立柱联动。底盘与立柱只有一个 `rt/cmd_vel` 发布线程：扫描时仅设置 `linear.z`，换列时仅设置 `linear.x`，`linear.y`和全部角速度始终为0。换列期间 `rt/lowcmd` 持续保持左臂顶部姿态。

## 现场执行顺序

### 1. 只读预检

不会创建运动发布器，报告中的 `published_commands` 必须为0：

```bash
cd ~/r1d_chassis
NETWORK=ens33 DOMAIN=0 \
OUTPUT=out/scan_shift_scan_dryrun \
bash run_scan_shift_scan_real.sh --dry-run
```

查看结果：

```bash
cat ~/r1d_chassis/out/scan_shift_scan_dryrun/preflight.json
```

### 2. 空旷地带确认底盘正方向（已完成，可按需复测）

此步骤必须在左臂已经收纳、机器人不靠箱且正前方清空时运行。程序若检测到左臂未收纳会拒绝运动。先使用0.05和0.5秒：

```bash
cd ~/r1d_chassis
NETWORK=ens33 DOMAIN=0 \
OUTPUT=out/base_direction_test \
bash run_scan_shift_scan_real.sh \
  --base-direction-test \
  --move-command 0.05 \
  --move-seconds 0.5 \
  --move-rate 50
```

第一次提示时确认周围清空后按回车。底盘停止后，只有肉眼确认正 `linear.x` 沿机器人正前方移动，才输入：

```text
FORWARD
```

负责人已经用0.05、0.5秒确认机器人向正前方移动。正式Demo不再依赖方向确认文件；该入口仅保留为后续按需复测工具。

### 3. 正式上扫、换列、下扫

机器人左侧朝箱、身体正前方沿箱面。确认顶部换列路径没有箱体突出件、人员或障碍物后执行：

```bash
cd ~/r1d_chassis
NETWORK=ens33 DOMAIN=0 \
OUTPUT=out/scan_shift_scan_real_demo \
bash run_scan_shift_scan_real.sh \
  --move-command 0.05 \
  --move-seconds 2.0 \
  --move-rate 50
```

预检通过后按一次回车才会运动。首轮1.0秒换列已成功，但现场观察位移不明显，因此当前演示建议保持 `--move-command 0.05`、把 `--move-seconds` 改为2.0。

报告位于：

```bash
cat ~/r1d_chassis/out/scan_shift_scan_real_demo/scan_shift_report.json
```

## 安全行为

- 顶部只有手臂、830 mm立柱和底盘都停稳后才开始换列。
- 换列结束后连续发送完整零Twist至少0.5秒；`rt/state_vel`确认停稳后才开始第二列下扫。
- 任意 `Ctrl+C`、DDS反馈超过200 ms、告警、关节/立柱跟踪失败或程序异常，都会结束共享发布线程并连续发送三次完整零Twist；不会继续下扫或收纳。
- 正常流程下扫到330 mm后才执行现有收纳。
- 换列期间命令始终保持 `linear.y=0`、`angular.z=0`。现场为快速演示，将侧向/旋转反馈即时停止阈值由0.03放宽为0.10；换列结束后仍须回到0.03以内才允许下扫。
- 负责人确认实物当前没有碰撞。替代模型在330 mm对 `probe_body/lift_sleeve_*` 的保守包络误报已在本Demo中排除，其余碰撞对继续检查。

## 修改入口

- `r1d_scan/scan_shift_scan.py`：独立状态机、预检、方向测试和报告。
- `run_scan_shift_scan_real.sh`：真机入口。
- `r1d_scan/sync_scan.py`：共享Twist线程增加前进分量，默认值仍为0，不改变既有单轮与连续扫描行为。
- `r1d_scan/lift_dds.py`：统一构造完整Twist。
- `r1d_scan/sim.py`：回环接受互斥的前进或升降指令并反馈前进速度，仅用于状态机验证，不估算真实换列距离。
