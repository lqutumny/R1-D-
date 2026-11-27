#!/usr/bin/env python3
"""看这台机器上有哪些 RPC 服务活着（只读，不发任何控制指令）。

为什么需要：单看一个服务返回 3102，分不清是"这个服务没有"还是"整台机器服务栈没起来"。
把所有候选服务都问一遍，才能判断属于哪种情况：

- 只有 agv 不在、别的在   → 这台机型没有 agv 服务，得换接口
- 全都不在                 → 机器人不在服务模式（多半是低层/阻尼/急停状态）
- 都在                     → 通道正常

用法：
    python3 tools/probe_services.py --network ens33 --domain 0
    python3 tools/probe_services.py --network ens33 --services agv,loco,audio
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from dds_network import initialize  # noqa: E402

DEFAULT_SERVICES = ["agv", "loco", "audio", "arm", "arm_action", "sport",
                    "lowstate", "video", "obstacles_avoid"]

CODE_HINT = {
    0: "在",
    2: "在（拒绝了这次请求）",
    3102: "不在（发不出去：这个域上没有该服务）",
    3103: "客户端接口未注册",
    3104: "超时（服务不在这个域）",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--network", required=True)
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--services", default=",".join(DEFAULT_SERVICES))
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "probe_services.json")
    args = parser.parse_args()

    initialize(args.network, args.domain)
    from unitree_sdk2py.rpc.client import Client

    table = {}
    print(f"目标：{args.network} / domain {args.domain}\n")
    print(f"{'服务':<16}{'返回码':>8}  {'版本':<12} 判读")
    print("-" * 62)
    for name in [s.strip() for s in args.services.split(",") if s.strip()]:
        client = Client(name, False)
        t0 = time.monotonic()
        code, version = client.GetServerApiVersion()
        dt = time.monotonic() - t0
        table[name] = {"code": code, "version": version, "seconds": round(dt, 2)}
        print(f"{name:<16}{code:>8}  {str(version):<12} {CODE_HINT.get(code, '未知')}")

    alive = [n for n, v in table.items() if v["code"] == 0]
    print()
    if not alive:
        print("判定：**一个服务都没问到** → 机器人多半不在服务模式"
              "（低层/阻尼/急停，或网卡/域不对）。")
        print("      这不能说明 agv 服务不存在，先让机器人回到正常运行状态再探。")
    elif "agv" not in alive:
        print(f"判定：服务栈是活的（{', '.join(alive)} 都在），但 **没有 agv**。")
        print("      → 这台机器的升降柱不走 agv 服务，需要找别的接口"
              "（例如 loco 的某组 API，或官方 App 用的那个服务）。")
    else:
        print(f"判定：agv 在（{', '.join(alive)}）。可以继续做零速探针与方向试验。")

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps({"network": args.network, "domain": args.domain,
                                       "services": table, "alive": alive},
                                      indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"\n报告 {args.report}")
    return 0 if "agv" in alive else 2


if __name__ == "__main__":
    raise SystemExit(main())