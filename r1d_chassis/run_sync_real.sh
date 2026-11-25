#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1
network="${NETWORK:-ens33}"
domain="${DOMAIN:-0}"
model_offset="${MODEL_OFFSET_MM:-105}"
output="${OUTPUT:-out/sync_real_$(date +%Y%m%d_%H%M%S)}"
if [[ $# == 1 && "${1:-}" == "--dry-run" ]]; then
    exec python3 r1d_scan/sync_scan.py --network "$network" --domain "$domain" \
        --output "$output" --model-offset-mm "$model_offset" --dry-run
fi
if [[ $# != 0 ]]; then
    echo "用法：bash run_sync_real.sh [--dry-run]" >&2
    exit 2
fi
echo "确认底盘停稳、立柱读数在 400 ± 10 mm 并停稳，手臂运动范围已清空，急停可用。"
printf '确认后按回车启动，按 Ctrl+C 取消。'
read -r _
exec python3 r1d_scan/sync_scan.py --network "$network" --domain "$domain" \
    --output "$output" --model-offset-mm "$model_offset"
