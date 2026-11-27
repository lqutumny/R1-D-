#!/usr/bin/env bash
# 独立回环 DDS 域。只收尾本脚本启动的仿真进程。
set -eu
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1
domain="${DOMAIN:-217}"
output="${OUTPUT:-out/sync}"
scene="models/r1_a7_scene_side_sync.xml"
mkdir -p "$output"
viewer=()
if [ "${NO_VIEWER:-0}" = 1 ]; then
    viewer=(--no-viewer)
fi
initial_height=()
if [ -n "${INITIAL_HEIGHT_MM:-}" ]; then
    initial_height=(--initial-lift-height-mm "$INITIAL_HEIGHT_MM")
fi
feedback_drop=()
if [ -n "${DROP_TOPICS:-}" ]; then
    if [ -z "${DROP_PHASE:-}" ] || [ -z "${DROP_DURATION_MS:-}" ]; then
        echo "DROP_TOPICS 需要同时设置 DROP_PHASE 和 DROP_DURATION_MS" >&2
        exit 2
    fi
    read -r -a drop_topics <<< "$DROP_TOPICS"
    feedback_drop=(--drop-feedback-topics "${drop_topics[@]}"
                   --drop-feedback-phase "$DROP_PHASE"
                   --drop-feedback-duration-ms "$DROP_DURATION_MS")
fi
python3 r1d_scan/sim.py --network lo --domain "$domain" --sync-lift --scene "$scene" "${viewer[@]}" \
    "${initial_height[@]}" "${feedback_drop[@]}" \
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
