# 真机运行数据（2026-11-23）

原始 CSV（scan.py 每 20 ms 记一行：指令角、实测角、力矩）压缩存在这里。
`out/` 目录不入库，所以这些是**唯一留存的真机证据**。

## 文件

| 文件 | 内容 |
|---|---|
| `rep1/2/3.csv.gz` + `.json` | **重复性三轮**（同一段程序连跑三次），用于 P8 |
| `scan_real2.csv.gz` + `.json` | 第一轮完整跑通（旧轨迹，33 s/单程，用于"卡顿"前后对比） |
| `scan_real4.csv.gz` + `.json` | 换平滑后重跑（25 s/单程） |

## 怎么用

```bash
gunzip -k data/rep1.csv.gz          # 解压
python3 tools/check_run.py data/rep1.csv                    # 逐关节跟踪误差
python3 tools/repeatability.py data/rep1.csv data/rep2.csv data/rep3.csv   # 三轮散差
python3 tools/tcp_from_csv.py data/rep1.csv                 # 反算 TCP 轨迹与直线度
```

## 三轮重复性的结论（复算结果）

| 轮次 | 直线度（最大 / RMS） |
|---|---|
| rep1 | 4.2 mm / 1.7 mm |
| rep2 | 4.2 mm / 1.7 mm |
| rep3 | 4.2 mm / 1.7 mm |

三轮之间最大散差 **2.9 mm**（判据 ≤30 mm）。三轮直线度完全一致，说明运动是确定性的。
详见 `docs/results.md` 第五次记录。

## CSV 列

`t, phase, q_cmd_<关节>×7, q_act_<关节>×7, tau_<关节>×7`

- `q_cmd_*`：我们发出去的目标角（rad）
- `q_act_*`：机器人回报的实测角（rad）
- `tau_*`：实测力矩（真机上基本为 0，见 `docs/results.md`）
- `phase`：startup-hold / approach / scan-up / scan-down / return / finished-hold
