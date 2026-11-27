#!/usr/bin/env bash
set -eu
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1

network="${NETWORK:-ens33}"
domain="${DOMAIN:-0}"
model_offset="${MODEL_OFFSET_MM:-105}"
output="${OUTPUT:-out/continuous_demo_real_$(date +%Y%m%d_%H%M%S)}"

echo "本次真机报告目录：$output"
echo "连续演示：一次自检和 330 mm 自动归位后，600 mm 左臂与 330↔830 mm 立柱连续联动。"
echo "第一次 Ctrl+C 完成本轮并收纳；第二次 Ctrl+C 立即触发安全停止。"
exec python3 r1d_scan/continuous_demo.py --network "$network" --domain "$domain" \
    --output "$output" --model-offset-mm "$model_offset" "$@"
