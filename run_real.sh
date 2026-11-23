#!/usr/bin/env bash
# 真机一条龙：现场检查 → 抓取确认 → 扫描 → 汇总
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

echo "=== 0/4 现场检查 ==="
echo "  · 机器人周围 1 m 内无人、无遮挡"
echo "  · 急停在手边（现场负责人）"
echo "  · 机器人已进入低层模式（遥控器）"
printf "  按回车继续，Ctrl-C 中止…… "
read -r _

echo "=== 1/4 检查指令通道（只读）==="
python3 tools/watch_lowcmd.py --network "$NETWORK" --domain "$DOMAIN" --seconds 3 || exit 1
if ! python3 -c "
import json,sys
d=json.load(open('out/lowcmd_watch.json'))
sys.exit(0 if d['packets']==0 else 1)
"; then
    echo "通道上已经有人在发指令 → 现在接管会两个写者抢同一个通道，退出。"
    echo "请先关掉原厂运控/其它程序再试。"
    exit 2
fi

echo "=== 2/4 抓取确认（14 槽位按住实测姿态 7 s，不动作）==="
python3 tools/nudge.py --network "$NETWORK" --domain "$DOMAIN" \
    --joint left_elbow_joint --amplitude-deg 0 --log out/grab_${STAMP}.csv || exit 1

echo "=== 3/4 扫描 $PASSES 趟（每趟约 86 s）==="
python3 r1d_scan/scan.py --network "$NETWORK" --domain "$DOMAIN" \
    --passes "$PASSES" --report "$REPORT" --log "$LOG"
STATUS=$?

echo "=== 4/4 汇总 ==="
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
    echo "控制器是「保护触发」退出的（返回码 2）——机器人已停止接收新指令，保持最后姿态。"
fi
exit "$STATUS"
