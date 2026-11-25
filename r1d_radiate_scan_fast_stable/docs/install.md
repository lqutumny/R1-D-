# 环境搭建

> 实测环境：Ubuntu 22.04.5 LTS（VMware 虚拟机）+ Python 3.10.12
> MuJoCo 3.17.0 / numpy 1.26.4 / scipy 1.8.0 / unitree-sdk2py 1.0.1

## 1. 系统与 Python

```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-venv git
python3 --version        # 需要 3.10+
```

Ubuntu 22.04 自带 3.10.12，够用。**不建议用 conda**：Cyclone DDS 的本地库更容易和系统 Python 对不上。

## 2. Python 依赖

```bash
pip3 install --user "mujoco>=3.2" numpy scipy
pip3 install --user unitree_sdk2py
```

`unitree_sdk2py` 会带进 `cyclonedds`；装完确认一下：

```bash
python3 -c "import mujoco, numpy, scipy, cyclonedds, unitree_sdk2py; print('deps ok')"
```

> 会有 `scipy 1.8.0 requires numpy<1.25` 的警告。**可以忽略**——本项目只用到
> `scipy.optimize` 里的线性规划，实际跑下来没有问题。真要在意就把 scipy 升到 1.9+。

## 3. 图形界面（MuJoCo 窗口）

仿真器默认开窗口。在虚拟机里需要 GLFW + 桌面环境：

```bash
sudo apt install -y libglfw3 libgl1-mesa-dri
```

如果通过 ssh 跑，要显式给显示变量（否则报 `X11: The DISPLAY environment variable is missing`）：

```bash
DISPLAY=:0 MUJOCO_GL=glfw bash run_demo.sh
```

**不需要窗口**时加 `NO_VIEWER=1`，此时不需要任何显示服务：

```bash
NO_VIEWER=1 bash run_demo.sh
```

> 本机实测：EGL 与 OSMesa 后端都不可用，只能用 `MUJOCO_GL=glfw`。

## 4. 机器人模型

`models/r1_a7.urdf` 与 `models/meshes/` 来自官方仓库，仓库里已经带好，正常不用重新下。
若要重新获取：

```bash
bash tools/fetch_urdf.sh        # 下载官方 URDF 到 /tmp
```

URDF 拷到 `models/`、mesh 拷到 `models/meshes/` 后，重建场景：

```bash
python3 tools/build_scene.py --base-z 1.0 --wall-x 0.70 \
        --probe-len 0.19 --probe-width 0.126 --probe-face-r 0.0235 --probe-mass 1.5
python3 tools/tune_gains.py --write      # 按目标阻尼比写回关节阻尼
```

`build_scene.py` 最后会自动做一次"保持关节自检"（模拟 0.5 s 物理，确认右臂/腰没被拉走），
输出 `保持关节自检：通过` 才算成功。

## 5. DDS 网络

仿真全部走本机回环 `lo`，`dds_network.py` 会自动写一份关闭多播、单播 127.0.0.1 的
Cyclone DDS 配置（只改内存模板，不动 SDK 文件），所以：

```bash
bash run_demo.sh                 # 内部即 --network lo --domain 0
```

接真机时把 `--network` 换成物理网卡名（`ip -br link` 查看），例如：

```bash
ip -br link
python3 r1d_scan/scan.py --network eth0 --domain 0 --report out/real.json --log out/real.csv
```

**同一个 DDS 域里只能有一个程序往 `rt/lowcmd` 发指令**。跑之前请结束官方示例、
上一次没关掉的窗口、以及任何别的控制器。

## 6. 常见报错

| 现象 | 原因 | 处理 |
|---|---|---|
| `X11: The DISPLAY environment variable is missing` | ssh 里没有 DISPLAY | 加 `DISPLAY=:0`，或 `NO_VIEWER=1` |
| `ERROR: could not initialize GLFW` | 缺 GLFW / 没有桌面 | `sudo apt install libglfw3`，或关窗口 |
| `没有有效 LowState（good=0 ...）` | 仿真器没起来 / 启动慢 | 看 `out/sim.log`；首次加载模型要 1–2 s |
| `仿真器没有就绪` | 上一个仿真器还占着 DDS | `pkill -f "[s]im[.]py"` 后重试 |
| 右臂/腰突然反折 | 场景里等式约束符号写反（历史 bug，已修） | 重建场景，看"保持关节自检" |
| 跑到一半 `保护触发（返回码 2）` | 虚拟机负载抖动导致控制循环停顿 >70 ms | 关掉其它重任务后重跑；这不是崩溃 |

## 7. 验证安装是否成功

```bash
python3 r1d_scan/profile.py          # 打印 profile 状态（应显示"可进入真机写指令阶段: 否"）
python3 tools/fault_injection.py --unit --codec   # 19 项进程内测试，2 秒，不占 DDS
bash run_demo.sh                     # 完整一轮，约 50 秒
```
