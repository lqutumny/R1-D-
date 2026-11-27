#!/usr/bin/env bash
# 固定立柱、左臂 600 mm 平顺提速组的一键真机入口。
set -euo pipefail
cd "$(dirname "$0")"

stamp="$(date +%Y%m%d_%H%M%S)"
output="${OUTPUT:-out/scan60_real_${stamp}}"
mkdir -p "$output"

echo "600 mm 固定立柱试扫；报告目录：$output"
echo "程序将读取当前静止姿态，自适应移动到扫描下端点。"
bash run_side_real_trial.sh --trial-length-cm 60 --trial-speed-mm-s 51 \
  --report "$output/scan_report.json" --log "$output/arm.csv" \
  2>&1 | tee "$output/terminal.log"
