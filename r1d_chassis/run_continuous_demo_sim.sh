#!/usr/bin/env bash
# 连续演示回环入口：仿真器与控制器共用独立 DDS 域，只清理本脚本启动的进程。
set -eu
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1

domain="${DOMAIN:-218}"
output="${OUTPUT:-out/continuous_demo_sim_$(date +%Y%m%d_%H%M%S)}"
scene="models/r1_a7_scene_side_sync.xml"
initial_height="${INITIAL_HEIGHT_MM:-650}"
mkdir -p "$output"

viewer=()
if [ "${NO_VIEWER:-0}" = 1 ]; then
    viewer=(--no-viewer)
fi

python3 r1d_scan/sim.py --network lo --domain "$domain" --sync-lift \
    --scene "$scene" --initial-lift-height-mm "$initial_height" "${viewer[@]}" \
    --report "$output/sim.json" > "$output/sim.log" 2>&1 &
sim_pid=$!
controller_pid=""

cleanup() {
    if [ -n "$controller_pid" ]; then
        kill -TERM "$controller_pid" 2>/dev/null || true
        wait "$controller_pid" 2>/dev/null || true
    fi
    kill -TERM "$sim_pid" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
}
forward_interrupt() {
    if [ -n "$controller_pid" ]; then
        kill -INT "$controller_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap forward_interrupt INT
trap 'exit 143' TERM

for _ in $(seq 1 100); do
    if grep -q '^SIM_READY ' "$output/sim.log"; then
        echo "本次仿真报告目录：$output"
        python3 r1d_scan/continuous_demo.py --network lo --domain "$domain" \
            --output "$output" "$@" &
        controller_pid=$!
        set +e
        while kill -0 "$controller_pid" 2>/dev/null; do
            wait "$controller_pid"
            status=$?
        done
        wait "$controller_pid" 2>/dev/null
        status=${status:-$?}
        set -e
        controller_pid=""
        exit "$status"
    fi
    if ! kill -0 "$sim_pid" 2>/dev/null; then
        break
    fi
    sleep 0.1
done

tail -25 "$output/sim.log"
exit 1
