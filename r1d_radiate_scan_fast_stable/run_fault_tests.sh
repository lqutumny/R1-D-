#!/usr/bin/env bash
# 阶段 4.6 保护项注入测试：一条命令跑完三层注入，并汇总通过数
set -u
cd "$(dirname "$0")" || exit 1

# 清掉可能占着 DDS 的旧进程（用括号防止 pkill 匹配到自己的命令行）
pkill -f "[f]ake_robot" 2>/dev/null
pkill -f "[r]1d_scan/sim[.]py" 2>/dev/null
pkill -f "[r]1d_scan/s[.]can" 2>/dev/null
sleep 1

echo "=== 保护项注入测试（约 2 分钟，期间不要跑别的仿真） ==="
python3 tools/fault_injection.py --all "$@"
STATUS=$?

python3 - <<'EOF'
import json
from pathlib import Path

path = Path("out/fault_injection.json")
if not path.exists():
    raise SystemExit
report = json.loads(path.read_text(encoding="utf-8"))
print(f"\n通过 {report['passed']}/{report['total']}，用时 {report['seconds']} s")
for section, label in (("unit", "A  进程内"), ("codec", "A' 解码层"),
                       ("dds", "B  进程级"), ("sim", "C  仿真器对抗")):
    items = report.get(section) or []
    if not items:
        continue
    ok = sum(1 for item in items if item.get("status") == "通过")
    print(f"  {label}: {ok}/{len(items)}")
    for item in items:
        if item.get("status") != "通过":
            print(f"    失败：{item['case']} — {item.get('detail')}")
EOF
echo "报告: out/fault_injection.json"
exit $STATUS
