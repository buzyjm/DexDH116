# CANFD C++ 示例

## 概述

基于 C++ SDK 的 CANFD 通信示例，支持 **电机运动测试** 和 **传感器数据读取** 两种模式。

提供 C++ API（`main.cpp`）和 C API（`main_c.cpp`）两个版本。

## 编译

在 SDK 示例目录下执行：

```bash
cd CANFD_cpp/
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

编译产物在 `build/bin/` 下：

- `{DistName}_CANFD_Test` — C++ API 版本
- `{DistName}_CANFD_Test_C` — C API 版本

## 运行

```bash
{DistName}_CANFD_Test       # C++ API 版本
# 或
{DistName}_CANFD_Test_C     # C API 版本
```

## 操作说明

1. 选择 CANFD 通道
2. 输入 CANFD 节点 ID（Node ID，默认 1）
3. 选择测试模式：
   - `0` — 电机运动测试：使能 → 回零 → 逐轴循环运动
   - `1` — 传感器数据测试：直接读取传感器数据（不使能电机）
4. 按 `Esc` 退出程序（Windows 控制台）

## Linux 后端选择（`USE_LIBCANBUS`）

`CANFDMaster.h` 顶部通过 **`USE_LIBCANBUS` 宏** 控制 Linux 底层驱动。优先级（高 → 低）：

1. **头文件宏** — 取消注释 `#define USE_LIBCANBUS`
2. **CMake 选项** — `-DLHANDPRO_USE_LIBCANBUS=ON`（仅当头文件宏未启用时生效）
3. **默认** — SocketCAN

```cpp
// CANFDMaster.h 顶部
// Linux 后端选择（优先级：高 → 低）:
// 1. 取消下行注释以启用 libcanbus
// 2. CMake 选项 -DLHANDPRO_USE_LIBCANBUS=ON
//
// 默认: SocketCAN
// #define USE_LIBCANBUS
```

**启用 libcanbus（推荐：直接改头文件）**

```cpp
#define USE_LIBCANBUS
```

**或通过 CMake（不改头文件时）**

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DLHANDPRO_USE_LIBCANBUS=ON
cmake --build .
```

> 若头文件中已 `#define USE_LIBCANBUS`，CMake 选项会被忽略。

| 后端 | 适用场景 | 依赖 |
|---|---|---|
| SocketCAN（默认） | Linux 内核 CAN 接口（`can0` 等） | `ip link` 配置、`socket()` |
| libcanbus | 厂商 USB CANFD 适配器（Linux `.so`） | `/usr/local/lib/libcanbus.so`、`libusb-1.0.so` |

### Linux libcanbus.so 部署

1. 将对应平台 tar 解压到 `/usr/local/lib/`（详见 CANFD Python 示例 `lib/libcanbus使用说明.txt`）
2. 配置 `LD_LIBRARY_PATH`（如需要）
3. 在 `CANFDMaster.h` 中启用 `USE_LIBCANBUS`，或使用 `-DLHANDPRO_USE_LIBCANBUS=ON` 编译

### Linux socketcan 初始化

示例会在连接时通过 `ip link` 自动配置接口。手动初始化可参考 CANFD Python 示例目录下的 `lib/socketcan初始化测试指令.txt`。

## Windows DLL 部署

将 `x64/bin/HCanbus.dll`（运行）或 `x64/lib/HCanbus.lib`（链接）放置到：

1. 与 `{DistName}_CANFD_Test` 相同的目录
2. 或添加到系统 PATH

`x64/` 目录结构：

| 路径 | 说明 |
|---|---|
| `x64/bin/HCanbus.dll` | CANFD 运行时 DLL |
| `x64/lib/HCanbus.lib` | CANFD 静态链接库 |
| `x64/include/HCanbus.h` | CANFD C 语言头文件 |

Windows 始终使用 HCanbus SDK，不受 `USE_LIBCANBUS` 影响。

## 文件说明

| 文件 | 作用 |
|---|---|
| `main.cpp` | C++ API 示例入口 |
| `main_c.cpp` | C API 示例入口 |
| `CANFDMaster.h / .cpp` | CANFD 通信封装（Windows HCanbus / Linux SocketCAN 或 libcanbus） |
| `CMakeLists.txt` | CMake 构建配置 |
| `x64/lib/` | Windows 64 位依赖库 |
| `x64/include/` | Windows 64 位头文件 |
| `x86/lib/` | Windows 32 位依赖库 |
| `x86/include/` | Windows 32 位头文件 |
