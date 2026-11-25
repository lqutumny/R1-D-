#!/usr/bin/env bash
# 真机一条龙：现场确认 → 通道与轨迹预检 → 扫描 → 汇总
#
# 用法：
#   bash run_real.sh                 # 1 趟（上下各一次）
#   PASSES=3 bash run_real.sh        # 3 趟
#   NETWORK=eth0 bash run_real.sh    # 换网卡
#
# 前置：机器人上电、已按遥控器进入低层（Q5），人已就位、急停在手。
set -u
cd "$(dirname "$0")" || exit 1

NETWORK="${NETWORK:-ens33}"
DOMAIN="${DOMAIN:-0}"
PASSES="${PASSES:-1}"
STAMP=$(date +%H%M%S)
REPORT="out/real_${STAMP}.json"
LOG="out/real_${STAMP}.csv"

echo "现场确认：机器人已进入低层模式，运动范围内无人、无遮挡，急停在手边。"
printf "确认后按回车，Ctrl-C 中止…… "
read -r _ || exit 1

echo "检查指令通道与轨迹，通过后启动 $PASSES 趟扫描。"
python3 r1d_scan/scan.py --network "$NETWORK" --domain "$DOMAIN" \
    --preflight-report "out/preflight_${STAMP}.json" \
    --passes "$PASSES" --report "$REPORT" --log "$LOG"
STATUS=$?

echo "=== 运行汇总 ==="
if [ -f "$REPORT" ]; then
    python3 tools/check_run.py "$LOG" 2>/dev/null | head -14
    python3 - <<EOF
import json
r = json.load(open("$REPORT", encoding="utf-8"))
print()
print("阶段：", " → ".join(f"{p['phase']}({p['t']:.1f}s)" for p in r["phases"]))
print(f"指令 {r['commands_sent']} 条，{r['command_rate_hz']} Hz，"
      f"跟踪最大 {r['max_tracking_error_rad']:.4f} rad，complete={r['complete']}")
print("结束关节角(deg)：", r["final_q_deg"])
if r["fault"]:
    print("!! 保护触发：", r["fault"])
EOF
fi
echo "报告 $REPORT    CSV $LOG"
if [ "$STATUS" -eq 2 ]; then
    if [ -f "$REPORT" ]; then
        echo "运行保护触发，已停止发送新指令。详情见运行报告。"
    else
        echo "轨迹预检未通过，未发送运动指令。详情见预检报告。"
    fi
fi
exit "$STATUS"
