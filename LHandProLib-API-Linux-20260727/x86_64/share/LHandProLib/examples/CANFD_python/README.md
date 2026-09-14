# CANFD Python 示例

## 概述

基于 Python SDK 的 CANFD 通信示例，支持 **电机运动测试** 和 **传感器数据读取** 两种模式。

## 环境依赖

```bash
pip install -r requirements.txt
```

Linux 使用 socketcan 后端时还需安装 `python-can`；使用 libcanbus 后端时需部署厂商 `.so`（见下文）。

## 运行

```bash
cd LHandProLib_CANFD_Test_python/
python main.py
```

## 操作说明

1. 程序自动扫描并列出 CANFD 设备，选择目标设备
2. 输入 CANFD 节点 ID（Node ID，默认 1）
3. 选择测试模式：
   - `0` — 电机运动测试：使能 → 回零 → 逐轴循环运动
   - `1` — 传感器数据测试：直接读取传感器数据（不使能电机，硬件无需运动）
4. 按 `Ctrl+C` 退出程序

## 测试模式详情

### 电机运动测试

程序会按照以下位置序列循环运行，每次运动一个轴：

```
轴 1 → 10000
轴 2 → 10000
全部 → 0
轴 3 → 10000
轴 4 → 10000
轴 5 → 10000
轴 6 → 10000
```

每步会打印当前位置和返回值，返回值 `0` 表示成功。

### 传感器数据测试

实时读取所有传感器（5 个手指 + 掌心）并刷新显示：

- `pressure[...]` — 各通道压力值
- `NF[...]` — 法向力
- `TF[...]` — 切向力
- `DIR[...]` — 受力方向

当手型为 LAC_DOF_6_S 时，仅显示 pressure 字段。

## CANFD 驱动选择（`USE_LIBCANBUS`）

Linux 底层驱动由 `canfd_lib.py` 文件顶部的 **`USE_LIBCANBUS` 宏** 控制。优先级（高 → 低）：

1. **`USE_LIBCANBUS` 宏**（`canfd_lib.py` 顶部）
2. **`driver` 参数**（仅当宏为 `False` 时生效）

```python
# canfd_lib.py 顶部
# 设为 True 时使用 Linux 厂商 libcanbus.so；False 时使用 socketcan（python-can）
USE_LIBCANBUS = False
```

**启用 libcanbus**

```python
USE_LIBCANBUS = True
```

保存后，`CANFD()` 与 `LHandProController` 在 Linux 上会自动走 libcanbus 后端；此时 `canfd_driver` / `driver` 参数会被忽略。

**使用 socketcan（默认）**

```python
USE_LIBCANBUS = False
```

若需临时指定 libcanbus 而不改宏，可保持 `USE_LIBCANBUS = False` 并传入 `driver`：

```python
from canfd_lib import CANFD

canfd = CANFD(driver="libcanbus")
```

通过控制器指定（同样仅在 `USE_LIBCANBUS = False` 时有效）：

```python
controller = LHandProController(communication_mode="CANFD",
                                canfd_driver="libcanbus")
# 或：
controller.connect(canfd_driver="libcanbus")
```

| 平台 | 后端 | 控制方式 |
|---|---|---|
| Windows | HCanbus.dll（固定） | `USE_LIBCANBUS` 无效 |
| Linux | socketcan（默认） | `USE_LIBCANBUS = False` |
| Linux | libcanbus.so | `USE_LIBCANBUS = True` |

### Windows HCanbus.dll 部署

`canfd_lib.py` 按以下顺序搜索 HCanbus.dll：

1. `./lib/HCanbus.dll`（推荐，见下方 `lib/` 目录说明）
2. `./HCanbus.dll`（当前目录）
3. 系统 PATH

### Linux libcanbus.so 部署

详见 `lib/libcanbus使用说明.txt`。

```bash
# 将对应平台的 tar 解压到 /usr/local/lib/
tar -xvf lib/libcanbus\(ubuntu22\).tar -C /usr/local/lib/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

然后在 `canfd_lib.py` 顶部设置：

```python
USE_LIBCANBUS = True
```

### Linux socketcan 初始化

详见 `lib/socketcan初始化测试指令.txt`。

```bash
sudo modprobe gs_usb
echo "a8fa 8598" | sudo tee /sys/bus/usb/drivers/gs_usb/new_id
sudo ip link set can0 up type can bitrate 1000000 sample-point 0.75 \
    dbitrate 5000000 dsample-point 0.75 fd on
```

使用 socketcan 时保持 `USE_LIBCANBUS = False`（默认）。

### 非 root 用户权限

将 `lib/hcanbus.rules` 复制到 `/etc/udev/rules.d/` 后重启系统，即可非 root 使用 USB CAN 设备。

## lib/ 目录说明

| 文件 | 说明 |
|---|---|
| `HCanbus.dll` | Windows CANFD 驱动（程序自动搜索 `./lib/HCanbus.dll` → PATH） |
| `hcanbus.rules` | Linux udev 规则，非 root 用户使用 USB CAN 设备 |
| `libcanbus(ubuntu20).tar` | libcanbus.so — Ubuntu 20.04（gcc 9.4） |
| `libcanbus(ubuntu22).tar` | libcanbus.so — Ubuntu 22.04（gcc 11.3） |
| `libcanbus_arm.tar` | libcanbus.so — ARM32（arm-linux-gnueabihf-gcc） |
| `libcanbus_arm64.tar` | libcanbus.so — ARM64（aarch64-linux-gnu-gcc） |
| `libcanbus使用说明.txt` | libcanbus 编译/部署/权限详细说明 |
| `socketcan初始化测试指令.txt` | Linux socketcan 初始化及收发测试指令 |

## 文件说明

| 文件 | 作用 |
|---|---|
| `main.py` | 示例主程序入口 |
| `canfd_lib.py` | CANFD 通信库封装（Windows HCanbus / Linux socketcan / libcanbus） |
| `requirements.txt` | Python 依赖（socketcan 后端需 python-can） |
| `lib/` | CANFD 驱动和文档（详见上方 `lib/` 目录说明） |
| `lhandprolib_python_sdk/` | Python SDK 封装库（详见该目录下 README） |
