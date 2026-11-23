# 关节核对表（阶段 5a 的现场作业单）

> 用途：接真机那天，按这张表逐项确认"槽位 ↔ 关节 ↔ 方向 ↔ 限位 ↔ 归属"。
> 只读阶段能自动填一半，另一半必须靠分级验证（会动一点）才能确定，**不要混着填**。
>
> 模板由 `python3 tools/analyze_device.py out/device.json --profile r1d_scan/robot_profile.json --urdf models/r1_a7.urdf`
> 生成草稿（`out/joint_map_draft.md`），本文件是"怎么填、填到什么算过关"的说明。

## 1. 六列分别是什么

| 列 | 含义 | 谁能填 | 判据 |
|---|---|---|---|
| 槽位 | `LowState.motor_state[]` 的下标 | 只读采集 | 与 `robot_profile.json` 一致 |
| 关节名 | URDF 里的关节名 | 只读采集（按 profile 对） | 与 `r1_a7.urdf` 一致 |
| 实测正方向 | 关节角增大时，机械往哪边转 | **分级验证**（`probe`：发 0.03 rad） | 与 URDF 轴向一致（±） |
| 实测限位 | 该关节真实能到的两端 | **分级验证**（`hold` / 慢速试探） | 不小于 URDF 限位，且留 2° 余量 |
| 控制范围 | 本程序敢用的区间 | 计算（URDF 限位 − 余量） | 见 `robot_profile.json` 的 lower/upper |
| 归属 | 由谁负责：本程序控制 / 本程序保持 / 原厂 / 不用 | 联调会议 | 与 `hold_on_robot` 一致 |

## 2. 只读阶段（5a）能确认什么、不能确认什么

**能确认**

- DDS 通不通：网卡、域、`rt/lowstate` 有没有包，包率多少，坏 CRC 多少。
- 哪些槽位活着：有 `mode`、有 q/dq、有 `tau_est` 的才算电机槽位。
- 当前姿态是否与"双臂自然下垂"一致（`initial` + `held` 的期望值）。
- 姿态在采集窗口内**稳不稳**：`q` 极差大 = 有人在动/在缓慢下垂，`q` 噪声大 = 抖。
- `mode_pr`、`mode_machine` 的值（本程序会把 `mode_machine` 回显进指令，必须对）。
- 腰 yaw 到底占不占独立槽位（13？）——看有没有信号。

**不能确认（必须动一点）**

- 正方向：不发指令就没法知道"q 增大 = 往左还是往右"。→ 5b 的 `probe`（0.03 rad）。
- 真实限位：不去顶就不知道软限位在哪。→ 5b 的 `hold` + 慢速试探。
- 力矩预算是否够：静止时 `tau_est` 只有重力分量，动态峰值要到 `dry-run` 才看得见。

## 3. 现场顺序（照这个来，别跳）

1. **上电、保持待机，不要按启动**（低层准入，Q5 已确认的话）。
2. 只读采集，确认上面"能确认"的六项：
   ```bash
   python3 tools/inspect_device.py --network eth0 --seconds 8
   python3 tools/analyze_device.py out/device.json \
       --profile r1d_scan/robot_profile.json --urdf models/r1_a7.urdf
   ```
   **注意 `out/device.json` 里 `scope` 字段必须是 READ ONLY**，`published_commands` 不存在。
3. 把 `out/joint_map_draft.md` 打印出来，现场对照机器人实物逐个念一遍关节名。
4. 有任何一项对不上 → **停下来**，不要进 5b。对不上的地方先查：
   - 槽位错位（最常见）：把 profile 的 slot 改对，再采一次。
   - 方向/sign 反：URDF 与实机轴向相反，需要在 profile 里加符号翻转（待实现）。
   - 限位不同：按实测收窄 `robot_profile.json` 的 lower/upper。
5. 全部对上 → 把 `robot_profile.json` 的 `slot_source` 改成 `verified`、`verified` 改成 `true`，
   然后跑 `python3 r1d_scan/profile.py` 确认"可进入真机写指令阶段: 是"。
   在那之前，`scan.py` 会因为准入锁拒绝在真机上启动——这是故意的。

## 4. 这张表要交什么

- [ ] `out/device.json`（只读采集原始数据）
- [ ] `out/joint_map_draft.md`（自动草稿）
- [ ] 填完的六列表（纸质签字或改成本文件）
- [ ] `robot_profile.json` 的 `slot_source: verified` 那一行
- [ ] 现场人员签字 + 日期

## 5. 判"通过"的三条硬标准

1. **14 个槽位全部活着**（控制 7 + 保持 7），没有一个是"活着=否"。
2. **采集窗口内 q 极差 < 0.5°**（机器人静止时，说明没有人在偷偷发指令、也没有掉使能）。
3. **姿态偏差 < 5°**（实测 vs `initial`/`held` 期望；大了说明零位或方向不对）。

三条都满足，才允许进入 5b 分级验证。任何一条不满足，先查清楚再说——这一关省下的时间，
后面会用整机撞墙来还。
