# EtherCAT C++ 示例

## 概述

基于 C++ SDK 的 EtherCAT 通信示例，底层使用 SOEM 协议栈。

提供多个独立入口程序，覆盖不同使用场景：

| 程序 | 说明 |
|---|---|
| `LHandProLib_EtherCAT_Test.exe` | 单手运动测试（C++ API） |
| `LHandProLib_EtherCAT_Test_C.exe` | 单手运动测试（C API） |
| `LHandProLib_EtherCAT_Test_Multi.exe` | 多机械手控制 |
| `LHandProLib_EtherCAT_Test_Sensor.exe` | 传感器数据测试（含 format 0/1 选择） |
| `LHandProLib_EtherCAT_Test_GLOVE.exe` | 数据手套跟随控制 |

## 编译

```bash
cd LHandProLib_EtherCAT_Test_cpp/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

编译产物在 `build/bin/` 下。

## 运行

```bash
LHandProLib_EtherCAT_Test_Sensor.exe   # 传感器测试为例
```

## Windows 依赖

需要 [Npcap](https://npcap.com/) 或 [WinPcap](https://www.winpcap.org/) 运行时。
安装 Npcap 时务必勾选 **"WinPcap API‑Compatible Mode"**。

运行时需要 `Packet.dll` 和 `wpcap.dll`，位于 `x64/` 目录下。
cmake install 时会自动部署到 `bin/` 目录。

## 文件说明

| 文件 | 作用 |
|---|---|
| `EthercatMaster.h / .cpp` | EtherCAT 主站封装 |
| `main.cpp` / `main_c.cpp` / `main_multi.cpp` / `main_sensor_test.cpp` / `main_glove.cpp` | 各程序入口 |
| `GloveSDK.h / .cpp` | 数据手套 SDK 封装 |
| `json.hpp` | JSON 解析（单头文件库） |
| `SOEM/` | SOEM 协议栈源码 |
| `x64/` | Windows 64位依赖（Packet.dll, wpcap.dll） |
