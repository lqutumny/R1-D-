#!/usr/bin/env python3
"""读一次运行留下的 CSV（scan.py / nudge.py 都写同样的列），看"指令 vs 实际"到底差多少。

用在真机复盘：哪一关节跟不上、什么时候开始掉、误差是慢慢积累还是突然跳。

用法：
  python3 tools/check_run.py out/scan_real.csv             # 汇总
  python3 tools/check_run.py out/scan_real.csv --every 25  # 每 25 行打一行明细
"""
import argparse
import csv
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from profile import load as load_profile  # noqa: E402

DEG = 180.0 / np.pi


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--every", type=int, default=0, help="每 N 行打一次明细，0=不打")
    args = parser.parse_args()

    names = [j.name for j in load_profile().joints]
    rows = list(csv.DictReader(args.csv_path.open(encoding="utf-8")))
    if not rows:
        raise SystemExit("CSV 是空的")
    t = np.array([float(r["t"]) for r in rows])
    phases = [r["phase"] for r in rows]

    print(f"{len(rows)} 行，{t[0]:.1f}–{t[-1]:.1f} s，阶段："
          + " → ".join(dict.fromkeys(phases)))
    print()
    print(f"{'关节':<30}{'最大误差':>10}{'出现时刻':>10}{'平均误差':>10}{'相位':>10}")
    worst = None
    for name in names:
        cmd = np.degrees(np.array([float(r[f"q_cmd_{name}"]) for r in rows]))
        act = np.degrees(np.array([float(r[f"q_act_{name}"]) for r in rows]))
        err = cmd - act
        k = int(np.argmax(np.abs(err)))
        phase = phases[k]
        print(f"{name:<30}{err[k]:>+9.1f}°{t[k]:>9.1f}s{np.abs(err).mean():>9.1f}°{phase:>10}")
        if worst is None or abs(err[k]) > abs(worst[1]):
            worst = (name, err[k], t[k], phase)

    print()
    print(f"最差：{worst[0]} 在 {worst[2]:.1f} s（{worst[3]}）偏 {worst[1]:+.1f}°")

    if args.every:
        print()
        head = "  时间 阶段       " + " ".join(f"{n.split('_')[1][:6]:>13}" for n in names)
        print(head + "   (指令/实际)")
        for i in range(0, len(rows), args.every):
            cells = []
            for name in names:
                c = np.degrees(float(rows[i][f"q_cmd_{name}"]))
                a = np.degrees(float(rows[i][f"q_act_{name}"]))
                cells.append(f"{c:+6.0f}/{a:+6.0f}")
            print(f"{t[i]:6.1f} {phases[i]:<9} " + " ".join(f"{c:>13}" for c in cells))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
