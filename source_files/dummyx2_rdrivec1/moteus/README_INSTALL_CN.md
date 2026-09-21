# RdriveC1 moteus 安装、编译和刷写说明

本文档记录安装 DummyX2 RdriveC1 moteus 工具链、编译固件，以及通过 CAN-FD 或 SWD 刷写固件的常用流程。

## 1. 环境要求

推荐环境：

- 操作系统：x86-64 Ubuntu 20.04、22.04 或 24.04
- Python：Python 3.13.9
- 构建系统：使用仓库自带的 `tools/bazel`，不要直接调用系统 `bazel`
- 目标硬件：moteus 控制器，MCU 为 STM32G4

说明：

- `tools/bazel` 会按仓库中 `WORKSPACE` 指定的 Bazel 版本下载并运行对应 Bazel，目前该仓库指定版本为 `7.4.1`。
- 首次编译需要网络下载 Bazel、mbed、编译工具链和第三方依赖。
- 本文后续命令固定使用 `python3.13` 创建虚拟环境。Ubuntu 的 `python3-*` apt 包通常绑定系统 Python，不等同于 Python 3.13.9 环境内的包。

## 2. 安装系统依赖

先安装编译 Python 3.13.9、CAN、测试和构建辅助依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  curl \
  git \
  ca-certificates \
  libssl-dev \
  zlib1g-dev \
  libbz2-dev \
  libreadline-dev \
  libsqlite3-dev \
  libffi-dev \
  liblzma-dev \
  tk-dev \
  xz-utils \
  can-utils \
  mypy \
  nodejs
```

如果需要从 SWD 口刷写固件，还需要：

```bash
sudo apt install -y openocd binutils-arm-none-eabi
```

如果需要用 GDB 调试：

```bash
sudo apt install -y gdb-multiarch
```

## 3. 安装 moteus Python 工具

日常使用和通过 CAN-FD 刷写固件时，需要安装 moteus Python 包。这里用 `pyenv` 安装并固定 Python 3.13.9：

```bash
mkdir -p ~/moteus
tar zxvf dummyx2_rdrivec1.tar.gz
cd ~/moteus

curl https://pyenv.run | bash
export PYENV_ROOT="$HOME/.pyenv"
export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init -)"

pyenv install 3.13.9
pyenv local 3.13.9
python3.13 --version
```

确认输出为：

```text
Python 3.13.9
```

然后创建虚拟环境并安装工具：

```bash
cd  /moteus
python3.13 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install moteus moteus-gui
```

主要工具：

- `moteus_tool`：命令行配置、诊断、校准和 CAN-FD 刷写工具
- `tview`：图形化遥测和配置工具

安装后可以检查：

```bash
python -m moteus.moteus_tool --help
tview --help
```

后续所有 `python -m ...` 命令都默认在这个虚拟环境中执行。重新打开终端后，需要先进入仓库并激活虚拟环境：

```bash
cd  /moteus
source .venv/bin/activate
```

## 4. CAN-FD 适配器和权限

常见连接方式包括：

- mjbots `fdcanusb` 或 `mjcanfd-usb-1x`
- Linux SocketCAN 接口，例如 `can0`

如果使用 `fdcanusb` 或 `mjcanfd-usb-1x`，普通用户可能没有 USB 设备权限，需要安装 udev 规则。可参考 fdcanusb 项目的 `70-fdcanusb.rules`，安装后重新插拔设备，或重新登录。

如果使用 SocketCAN，典型启动方式如下，具体 bitrate/dbitrate 要和硬件及固件设置一致：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

检查设备是否能被发现：

```bash
python -m moteus.moteus_tool --target 1 --info
```

如果只有一个控制器，也可以先尝试自动发现：

```bash
python -m moteus.moteus_tool --info
```

## 5. 编译 moteus 固件

进入仓库根目录：

```bash
cd  /moteus
```

编译目标固件：

```bash
tools/bazel build --config=target //:target
```

该目标会构建：

- `//fw:moteus`：主固件
- `//fw:can_bootloader`：CAN bootloader
- `//fw:bin`：用于 SWD 烧录的分段 bin 文件

常用输出文件：

```text
bazel-out/stm32g4-opt/bin/fw/moteus.elf
bazel-out/stm32g4-opt/bin/fw/can_bootloader.elf
bazel-out/stm32g4-opt/bin/fw/moteus.08000000.bin
bazel-out/stm32g4-opt/bin/fw/moteus.0800c000.bin
bazel-out/stm32g4-opt/bin/fw/moteus.08010000.bin
```

如需运行主机端测试：

```bash
tools/bazel test --config=host //:host
```

如果 Bazel 下载依赖失败，可以先预下载依赖缓存：

```bash
bash utils/download_bazel_deps.sh
```

然后再重新执行 build/test 命令。

## 6. 通过 CAN-FD 刷写固件

适用于控制器当前还能通过 CAN-FD 通信的情况。

先确认控制器在线：

```bash
python -m moteus.moteus_tool --target 1 --info
```

刷写本地编译出的固件：

```bash
python -m moteus.moteus_tool \
  --target 1 \
  --flash bazel-out/stm32g4-opt/bin/fw/moteus.elf
```

也可以刷写官方 release 下载的固件。应选择类似 `YYYYMMDD-moteus-HASH.elf` 的文件，不要选择 `bootloader` 文件：

```bash
python -m moteus.moteus_tool --target 1 --flash path/to/moteus.elf
```

注意事项：

- `--target 1` 是控制器 CAN ID，根据实际 ID 修改。
- 刷写时保持电源稳定，不要断开 CAN 或电源。
- 默认会校验 flash 内容，并尽量保留/恢复原配置。

## 7. 通过 SWD 刷写固件

适用于 CAN-FD 不可用、bootloader 损坏、或者需要完整重刷 bootloader 和主固件的情况。

硬件准备：

- 一个 ST-Link 或兼容 STM32 SWD 调试器
- 调试器连接到 moteus 的 SWD 接口
- moteus 正常供电

安装依赖：

```bash
sudo apt install -y openocd binutils-arm-none-eabi
```

先编译固件：

```bash
tools/bazel build --config=target //:target
```

使用脚本刷写默认产物：

```bash
./fw/flash.py
```

默认会刷写：

```text
bazel-out/stm32g4-opt/bin/fw/moteus.elf
bazel-out/stm32g4-opt/bin/fw/can_bootloader.elf
```

如果要指定固件和 bootloader：

```bash
./fw/flash.py path/to/moteus.elf path/to/can_bootloader.elf
```

如果需要整片擦除后再刷写：

```bash
./fw/flash.py --erase
```

也可以使用 Bazel 的 flash 目标直接刷写：

```bash
tools/bazel build --config=target //fw:flash
```

## 8. SWD 调试

安装 GDB：

```bash
sudo apt install -y gdb-multiarch
```

一个终端启动 OpenOCD：

```bash
cd  /moteus/fw
./run_openocd_noreset.sh
```

另一个终端启动 GDB：

```bash
cd  /moteus
gdb-multiarch -x moteus-debug.gdb bazel-out/stm32g4-opt/bin/fw/moteus.elf
```

## 9. fdcanusb 固件编译和刷写

仓库中包含一个独立的 `fdcanusb` 子仓库。它有自己的 `WORKSPACE`、`.bazelrc` 和 `tools/bazel`，所以编译 fdcanusb 固件时要进入 `fdcanusb` 目录，不要在 moteus 仓库根目录直接构建。

进入 fdcanusb 子仓库：

```bash
cd  /moteus/fdcanusb
```

安装 fdcanusb 编译和 SWD 刷写依赖：

```bash
sudo apt update
sudo apt install -y curl openocd binutils-arm-none-eabi
```

编译 fdcanusb 固件：

```bash
./tools/bazel build //fw:fdcanusb.bin
```

常用输出文件：

```text
bazel-out/stm32g4-opt/bin/fw/fdcanusb.elf
bazel-out/stm32g4-opt/bin/fw/fdcanusb.bin
```

通过 SWD 刷写默认编译产物：

```bash
./flash.py
```

`flash.py` 默认会把：

```text
bazel-out/stm32g4-opt/bin/fw/fdcanusb.elf
```

转换为二进制并用 OpenOCD 写入 `0x08000000`。

如果要指定 elf 文件：

```bash
./flash.py path/to/fdcanusb.elf
```

如果需要整片擦除后再刷写：

```bash
./flash.py --erase
```

也可以用 Bazel 的 flash 目标直接刷写：

```bash
./tools/bazel build //fw:flash
```

硬件连接要求：

- 使用 ST-Link 或兼容 STM32 SWD 调试器
- 连接 SWDIO、SWCLK、GND 和目标电源参考
- fdcanusb 板正常供电
- 刷写过程中不要断开 USB/SWD 或电源

## 10. 常见问题

### Bazel 版本不对

不要直接运行 `bazel build ...`。本仓库使用：

```bash
tools/bazel build --config=target //:target
```

`tools/bazel` 会自动使用仓库要求的 Bazel 版本。

### 找不到 Python 包

确认当前 shell 已激活虚拟环境：

```bash
cd  /moteus
source .venv/bin/activate
python --version
python -m moteus.moteus_tool --help
```

`python --version` 应显示 `Python 3.13.9`。

### CAN-FD 找不到设备

检查：

- 控制器是否供电
- CAN-H/CAN-L 是否接反
- CAN 总线是否有正确终端电阻
- CAN ID 是否正确
- `fdcanusb`/SocketCAN 接口是否正常
- 用户是否有 USB/CAN 设备权限

### SWD 刷写失败

检查：

- ST-Link 是否被系统识别
- SWDIO、SWCLK、GND、目标电源参考是否连接正确
- moteus 是否已供电
- 是否安装了 `openocd` 和 `binutils-arm-none-eabi`
- 是否有权限访问 USB 调试器

## 11. 参考文件

https://github.com/mjbots/moteus
