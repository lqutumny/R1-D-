#!/usr/bin/env bash
# 现场固定底盘与立柱的慢速试扫，默认 15 cm。
set -eu
cd "$(dirname "$0")"
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
output="${OUTPUT:-out/side_trial_real}"
exec python3 r1d_scan/scan.py --network "${NETWORK:-ens33}" --domain "${DOMAIN:-0}" \
    --side-real-trial --report "$output/scan_report.json" --log "$output/arm.csv" "$@"
