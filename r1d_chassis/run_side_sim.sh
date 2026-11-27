#!/usr/bin/env bash
# 第 1 阶段：固定立柱，只在回环 DDS 域运行。
set -eu
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1
domain="${DOMAIN:-227}"
output="${OUTPUT:-out/side_scan}"
profile="${SIDE_PROFILE:-models/side_scan_profile.json}"
scene="${SIDE_SCENE:-models/r1_a7_scene_side.xml}"
mkdir -p "$output"
viewer=()
if [[ "${NO_VIEWER:-0}" == 1 ]]; then
    viewer=(--no-viewer)
fi
python3 r1d_scan/sim.py --network lo --domain "$domain" --scene "$scene" "${viewer[@]}" \
    --report "$output/sim.json" > "$output/sim.log" 2>&1 &
sim=$!
cleanup() {
    kill "$sim" 2>/dev/null || true
    wait "$sim" || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
for _ in $(seq 1 120); do
    if grep -q '^SIM_READY ' "$output/sim.log"; then
        status=0
        python3 r1d_scan/scan.py --network lo --domain "$domain" --side-profile "$profile" \
            --side-scene "$scene" --report "$output/scan_report.json" --log "$output/arm.csv" || status=$?
        if [[ "$output/scan_report.json" -nt "$output/sim.log" ]]; then
            python3 tools/report_side_scan.py --scene "$scene" --profile "$profile" --output "$output" || status=$?
        fi
        exit "$status"
    fi
    if ! kill -0 "$sim" 2>/dev/null; then break; fi
    sleep .1
done
tail -25 "$output/sim.log"
exit 1
