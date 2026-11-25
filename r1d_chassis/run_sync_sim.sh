#!/usr/bin/env bash
# 独立回环 DDS 域。只收尾本脚本启动的仿真进程。
set -eu
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1
domain="${DOMAIN:-217}"
output="${OUTPUT:-out/sync}"
mkdir -p "$output"
viewer=()
if [ "${NO_VIEWER:-0}" = 1 ]; then
    viewer=(--no-viewer)
fi
python3 r1d_scan/sim.py --network lo --domain "$domain" --sync-lift "${viewer[@]}" \
    --report "$output/sim.json" > "$output/sim.log" 2>&1 &
sim_pid=$!
cleanup() {
    kill "$sim_pid" 2>/dev/null || true
    wait "$sim_pid" || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
for _ in $(seq 1 100); do
    if grep -q '^SIM_READY ' "$output/sim.log"; then
        python3 r1d_scan/sync_scan.py --domain "$domain" --output "$output"
        exit
    fi
    if ! kill -0 "$sim_pid" 2>/dev/null; then
        break
    fi
    sleep 0.1
done
tail -25 "$output/sim.log"
exit 1
