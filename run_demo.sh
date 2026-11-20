#!/usr/bin/env bash
# 一条命令跑完整一轮：启动 MuJoCo 仿真器（带窗口）→ 等待就绪 → 运行控制器 → 收尾
set -u
cd "$(dirname "$0")" || exit 1
NETWORK="${NETWORK:-lo}"
DOMAIN="${DOMAIN:-0}"
HOLD_MODE="${HOLD:-constraint}"
VIEWER_FLAG=""
[ "${NO_VIEWER:-0}" = "1" ] && VIEWER_FLAG="--no-viewer"

# 清掉上一次残留的仿真器（用 [s] 防止匹配到自己这条命令）
pkill -f "[r]1d_scan/sim.py" 2>/dev/null
sleep 1

echo "[1/3] 启动 MuJoCo 仿真器（窗口即将弹出）..."
mkdir -p out
python3 r1d_scan/sim.py --network "$NETWORK" --domain "$DOMAIN" $VIEWER_FLAG \
    --hold "$HOLD_MODE" --duration 240 > out/sim.log 2>&1 &
SIM_PID=$!

for i in $(seq 1 100); do
    grep -q SIM_READY out/sim.log 2>/dev/null && break
    sleep 0.1
done
if ! grep -q SIM_READY out/sim.log 2>/dev/null; then
    echo "仿真器没有就绪，日志："
    tail -5 out/sim.log
    kill "$SIM_PID" 2>/dev/null
    exit 1
fi
echo "[2/3] 仿真器就绪，开始运行控制器（约 60 秒）..."

python3 r1d_scan/scan.py --network "$NETWORK" --domain "$DOMAIN" \
    --report out/scan_run.json --log out/scan_run.csv
STATUS=$?

echo "[3/3] 结束，关闭仿真器..."
kill "$SIM_PID" 2>/dev/null
for _ in $(seq 1 20); do
    kill -0 "$SIM_PID" 2>/dev/null || break
    sleep 0.2
done
kill -9 "$SIM_PID" 2>/dev/null
wait "$SIM_PID" 2>/dev/null
if [ "$STATUS" -ne 0 ]; then
    if [ "$STATUS" -eq 2 ]; then
        echo "保护触发（返回码 2）：控制器已停止发送指令，机器人保持最后一条指令；原因见上面的报错。"
    else
        echo "控制器异常退出（返回码 $STATUS），上面的报错就是原因。"
    fi
    echo "仿真日志：out/sim.log"
    exit "$STATUS"
fi
if [ -f out/scan_run.json ]; then
    python3 - <<'EOF'
import json
r = json.load(open("out/scan_run.json", encoding="utf-8"))
print("阶段：", " → ".join(f"{p['phase']}({p['t']:.1f}s)" for p in r["phases"]))
print(f"指令 {r['commands_sent']} 条，最大跟踪误差 {r['max_tracking_error_rad']:.4f} rad，"
      f"complete={r['complete']}")
print("最终关节角(deg)：", r["final_q_deg"])
EOF
fi
echo "CSV: out/scan_run.csv   报告: out/scan_run.json   仿真日志: out/sim.log"
exit $STATUS
