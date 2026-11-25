#!/usr/bin/env python3
"""`agv` 服务只读探针：确认底盘/升降柱这条通道在不在，不动任何执行器。

分两段，第二段要显式加 --zero-height 才会跑：

1. **只读**：向服务端要 API 版本（RPC 内部查询）。不创建任何 publisher 之外的东西。
2. **零速**（可选）：发一条 `HeightAdjust(0)`，确认 1002 被接受。零速度 = 立柱不动。

返回码怎么读：

| 返回码 | 含义 | 下一步 |
|---|---|---|
| 0 + 版本号 | 服务在，接口通 | 可以继续（量高度、试方向） |
| 2 | 服务在，但服务端明确拒绝（例如仿真器没开 --lift） | 查固件/模式 |
| 3102 | 发不出去：这条域上没有这个服务 | 换域、查网卡、查是否进了低层模式 |
| 3104 | 请求超时：服务可能不在同一域 | 同上 |
| 其他 | 打印原始码，别猜 | 查 SDK 错误表 |
"""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "r1d_scan"))

from dds_network import initialize  # noqa: E402


def explain(code):
    table = {
        0: "服务在，接口通",
        2: "服务在，但服务端明确拒绝（仿真器没开 --lift？或固件模式不对）",
        3102: "发不出去：这条 DDS 域上没有 agv 服务（网卡/域/是否进低层模式）",
        3103: "客户端的接口没注册（代码问题）",
        3104: "请求超时：服务不在这个域，或服务端没起来",
    }
    return table.get(code, f"未知返回码 {code}（别猜，去查 SDK 错误表）")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--network", required=True, help="网卡名，例如 ens33")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--zero-height", action="store_true",
                        help="额外发一条 HeightAdjust(0)。零速度，立柱不动")
    parser.add_argument("--report", type=Path, default=ROOT / "out" / "probe_agv.json")
    args = parser.parse_args()

    initialize(args.network, args.domain)
    from chassis import Chassis, Limits

    limits = Limits.load()
    # auto_watchdog=False：看门狗会发 Move(0,0,0)，那是底盘指令，这次不需要
    c = Chassis(limits=limits, dry_run=False, auto_watchdog=False)
    result = {"network": args.network, "domain": args.domain,
              "server_version": None, "height_probe": None,
              "limits": c.summary()["limits"], "height_limits": c.summary()["height"]}

    print(f"目标：{args.network} / domain {args.domain}")
    t0 = time.monotonic()
    code, version = c.server_version()
    dt = time.monotonic() - t0
    result["server_version"] = {"code": code, "version": version, "seconds": round(dt, 2)}
    print(f"\n[1] 只读：问服务端 API 版本")
    print(f"    返回码 {code}（{explain(code)}），耗时 {dt:.1f} s")
    if code == 0:
        print(f"    服务端版本 = {version!r}（官方 agv 是 '1.0.0.1'）")
        if str(version) != "1.0.0.1":
            print("    注意：版本和官方头文件不一致，接口语义可能不同，先别发指令")

    if args.zero_height and code == 0:
        t0 = time.monotonic()
        ok, hcode = c.probe_height()
        dt = time.monotonic() - t0
        result["height_probe"] = {"code": hcode, "ok": ok, "seconds": round(dt, 2)}
        print(f"\n[2] 零速：HeightAdjust(0)（立柱不动）")
        print(f"    返回码 {hcode}（{explain(hcode)}），耗时 {dt:.1f} s")
    elif args.zero_height:
        print("\n[2] 跳过：服务都没问到，发零速也没意义")
    else:
        print("\n[2] 未执行（要跑就加 --zero-height）")

    print(f"\n结论：{'通道可用' if code == 0 else '通道不可用 —— ' + explain(code)}")
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n",
                           encoding="utf-8")
    print(f"报告 {args.report}")
    return 0 if code == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
