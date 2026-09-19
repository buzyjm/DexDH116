# LabVIEW 测试示例

## 概述

LabVIEW 环境下的控制测试程序，基于 `LHandProLib_LV` 封装库调用 SDK，支持 CANFD、EtherCAT、RS485 三种通信方式。

## 编译

```bash
cd LHandProLib_LabView_Test/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

> 需要先编译 `LHandProLib_LabView` 子项目（CMake 选项 `-DBUILD_LABVIEW=ON`）。

## 运行

```bash
# CANFD 测试
LabView_Test_CANFD.exe

# EtherCAT 测试
LabView_Test_EtherCAT.exe

# RS485 测试
LabView_Test_RS485.exe
```

## 文件说明

| 文件 | 作用 |
|---|---|
| `main_canfd.cpp` | CANFD 通信测试示例 |
| `main_ethercat.cpp` | EtherCAT 通信测试示例 |
| `main_rs485.cpp` | RS485 通信测试示例 |
