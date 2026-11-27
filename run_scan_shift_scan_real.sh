#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1

network="${NETWORK:-ens33}"
domain="${DOMAIN:-0}"
model_offset="${MODEL_OFFSET_MM:-105}"
output="${OUTPUT:-out/scan_shift_scan_real_$(date +%Y%m%d_%H%M%S)}"

echo "本次报告目录：$output"
echo "流程：第一列上扫 → 顶部正前方换列 → 第二列下扫 → 收纳。"
echo "底盘与立柱由同一个 rt/cmd_vel 发布器发送完整 Twist。"
exec python3 r1d_scan/scan_shift_scan.py --network "$network" --domain "$domain" \
    --model-offset-mm "$model_offset" --output "$output" "$@"
