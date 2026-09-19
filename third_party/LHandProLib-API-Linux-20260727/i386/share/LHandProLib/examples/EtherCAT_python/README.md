# EtherCAT Python 示例

## 概述

基于 Python SDK 的 EtherCAT 通信示例，支持 **电机运动测试** 和 **传感器数据读取** 两种模式。

底层使用 SOEM（Simple Open EtherCAT Master）协议栈。

## 环境依赖

```bash
cd LHandProLib_EtherCAT_Test_python/
pip install -r requirements.txt
```

**Windows 额外依赖**：[Npcap](https://npcap.com/) 或 [WinPcap](https://www.winpcap.org/)
安装 Npcap 时务必勾选 **"WinPcap API‑Compatible Mode"**。

> **DH116 ESI 映射说明：** C++ SOEM 封装可以直接补齐 context 内缺失的
> SM2/SM3 字段，PySOEM 1.1.12 不公开这些字段。Python 封装会核验
> 64O/192I、expected WKC 以及 SM2/SM3 寄存器；发现映射不完整时会拒绝
> 启动。此时应修复设备 ESI/EEPROM，或使用增加了 SM context 配置接口的
> PySOEM 构建。

## 运行

```bash
python main.py
```

连接时会先选择网口，再枚举从站。默认自动选择从站 `1`；可以显式指定
目标从站：

```python
controller = LHandProController(communication_mode="ECAT")
controller.connect(slave_id=2)
```

设置 `auto_select=False` 时，如果存在多个网口或多个从站，都会进入交互选择。

### 直接使用 EtherCAT 封装

```python
from ethercat_master import EthercatMaster

master = EthercatMaster()
interfaces = master.scanNetworkInterfaces()
interface_index = 0

probed_slaves = []
master.probeSlaves(interface_index, probed_slaves)

if not master.init(interface_index) or not master.start():
    raise RuntimeError(master.getLastError())

slave_id = 1
master.run()

master.setSlaveOutputs(slave_id, bytes(64), 64)
input_size = master.getSlaveInputSize(slave_id)
inputs = master.getSlaveInputs(slave_id, input_size)

master.stop()
```

## 操作说明

1. 选择 EtherCAT 网口
2. 选择目标 EtherCAT 从站（只有一个从站时自动选择）
3. 选择传感器格式：
   - `0` — 默认格式（6 传感器）
   - `1` — Channel 格式（7 传感器，手指指尖 + 指腹）
4. 程序自动使能 → 回零 → 进入循环运动
5. 按 `Ctrl+C` 退出程序

## 文件说明

| 文件 | 作用 |
|---|---|
| `main.py` | 示例主程序入口 |
| `ethercat_master.py` | EtherCAT 主站封装（基于 SOEM） |
| `requirements.txt` | Python 依赖 |
| `lhandprolib_python_sdk/` | Python SDK 封装库（详见该目录下 README） |
