"""网卡与 DDS 域初始化。本地仿真用 lo，真机用物理网卡。"""
from pathlib import Path
import os
import re


LOOPBACK_CONFIG = """<CycloneDDS><Domain Id="any">
  <General><Interfaces><NetworkInterface name="lo" multicast="false"/></Interfaces>
<AllowMulticast>false</AllowMulticast>
<MaxMessageSize>1200B</MaxMessageSize><FragmentSize>1024B</FragmentSize></General>
  <Discovery><ParticipantIndex>auto</ParticipantIndex>
    <Peers><Peer Address="127.0.0.1"/></Peers></Discovery>
</Domain></CycloneDDS>"""


def validate_network(network, domain):
    if not re.fullmatch(r"[A-Za-z0-9_.:-]+", network or ""):
        raise ValueError("需要显式指定 --network 网卡名（先用 ip -br link 查）")
    interface = Path("/sys/class/net") / network
    if not interface.exists():
        raise ValueError(f"网卡不存在: {network}；用 ip -br link 查看可用网卡")
    if network != "lo" and not int((interface / "flags").read_text().strip(), 16) & 0x1000:
        raise ValueError("物理网卡需要支持多播；本地仿真可使用 lo")
    if not 0 <= domain <= 232:
        raise ValueError("DDS domain 必须在 0–232 内")


def initialize(network, domain=0):
    validate_network(network, domain)
    from unitree_sdk2py.core import channel
    if network == "lo":
        # 本机回环：关闭多播、指定 127.0.0.1 单播发现，只改内存模板，不动 SDK 文件。
        previous = channel.ChannelConfigHasInterface
        try:
            config = LOOPBACK_CONFIG
            if os.environ.get("R1D_DDS_TRACE") == "1":
                config = config.replace("</Domain>",
                    "<Tracing><Verbosity>finest</Verbosity><OutputFile>stderr</OutputFile></Tracing></Domain>")
            channel.ChannelConfigHasInterface = config
            channel.ChannelFactoryInitialize(domain, network)
        finally:
            channel.ChannelConfigHasInterface = previous
    else:
        channel.ChannelFactoryInitialize(domain, network)
