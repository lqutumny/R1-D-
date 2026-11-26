#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1
network="${NETWORK:-ens33}"
domain="${DOMAIN:-0}"
model_offset="${MODEL_OFFSET_MM:-105}"
output="${OUTPUT:-out/sync_real_$(date +%Y%m%d_%H%M%S)}"
echo "本次报告目录：$output"
if [[ $# == 1 && "${1:-}" == "--dry-run" ]]; then
    exec python3 r1d_scan/sync_scan.py --network "$network" --domain "$domain" \
        --output "$output" --model-offset-mm "$model_offset" --dry-run
fi
if [[ $# != 0 ]]; then
    echo "用法：bash run_sync_real.sh [--dry-run]" >&2
    exit 2
fi
echo "侧向 600 mm 手臂 + 400→800→400 mm 立柱联动。"
echo "程序先完成实时姿态和组合几何预检；回车后自动把立柱调到 400 mm，再开始联动。"
exec python3 r1d_scan/sync_scan.py --network "$network" --domain "$domain" \
    --output "$output" --model-offset-mm "$model_offset"
