# RS485 C++ 示例

## 概述

基于 C++ SDK 的 RS485 串口通信示例：

- `LHandProLib_RS485_Test`：单节点电机运动和传感器数据测试。
- `LHandProLib_RS485_DualNode_Test`：在一个物理串口总线上初始化
  NodeID 1、2 两个 SDK，通过共享总线调度分别控制两台设备。

## 编译

```bash
cd LHandProLib_RS485_Test_cpp/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

编译产物在 `build/bin/` 下。

## 运行

```bash
LHandProLib_RS485_Test.exe
LHandProLib_RS485_DualNode_Test.exe
```

## 操作说明

1. 扫描并选择串口
2. 选择测试模式：
   - `0` — 电机运动测试：使能 → 回零 → 逐轴循环运动
   - `1` — 传感器数据测试：直接读取传感器数据（不使能电机）
3. 按 `Esc` 退出程序

双节点程序：

1. 只打开一个串口，依次初始化 NodeID 1 和 NodeID 2。
2. 两个 SDK 通过 `set_rs485_shared_bus(&bus)` 绑定到 SDK 提供的同一个
   `RS485SharedBus`，然后仍分别调用 `initial(LCN_RS485, 1/2)`。
3. 示例只透明写串口并把接收数据送入一次 `RS485SharedBus`；轮询命令、
   优先级、30 ms事务间隔、CRC和NodeID分发全部由SDK内部处理。
4. 通过菜单分别使能、回零、运动或查看两个节点。

## 驱动说明

RS485 使用标准串口通信，无需额外驱动或 DLL。

## 文件说明

| 文件 | 作用 |
|---|---|
| `main.cpp` | 示例主程序入口 |
| `main_dual_node.cpp` | 单串口、双 SDK、NodeID 1/2 控制示例 |
| `SerialPort.h / .cpp` | 串口通信封装 |
| `CMakeLists.txt` | CMake 构建配置 |
