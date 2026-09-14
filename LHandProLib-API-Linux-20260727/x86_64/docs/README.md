# LHandProLib

灵巧手 SDK — C++17 跨平台库，支持 EtherCAT / CANFD / RS485 / 标准 CAN 四种通信协议。

## 部署目录结构

```
install/
├── bin/                          ← 可执行文件 + 运行时 DLL
│   ├── LHandProLib.dll           SDK 核心库
│   ├── LHandProLib_CANFD_Test    CANFD C++ 示例
│   ├── LHandProLib_RS485_Test    RS485 C++ 示例
│   ├── LHandProLib_RS485_DualNode_Test 单串口双节点 RS485 示例
│   ├── LHandProLib_EtherCAT_Test EtherCAT C++ 示例 (含 Sensor/Multi/GLOVE)
│   ├── LHandProLib_CAN_Test      标准 CAN C++ 示例
│   ├── LHandProLib_LV.dll        LabVIEW 封装库 (CANFD/EtherCAT/RS485)
│   ├── LHandProLib_LabView_Test  LabVIEW 封装测试 (CANFD/EtherCAT/RS485)
│   ├── HCanbus.dll               CANFD 驱动 (Windows)
│   ├── PCANBasic.dll             标准 CAN 驱动 (Windows)
│   ├── Packet.dll / wpcap.dll    EtherCAT 网络驱动 (Npcap/WinPcap)
│   └── msvcp140.dll / vcruntime140.dll  VC++ 运行时
├── include/
│   └── LHandProLib/              C++ / C 头文件
│       ├── LHandProLib.hpp       C++ 公开 API
│       └── LHandProLib.h         C API 封装
├── lib/
│   ├── LHandProLib.lib           链接库
│   └── LHandProLib_LV.lib        LabVIEW 链接库
├── docs/
│   ├── README.md                 SDK 说明
│   ├── LHandProLib_SDK_Manual.md     SDK 开发手册
│   └── LHandProLib_SDK_功能手册.md   SDK 功能手册
└── share/
    └── LHandProLib/
        └── examples/             各通信协议的示例源码
            ├── CANFD_cpp/        CANFD C++ 示例 + 驱动
            ├── CANFD_python/     CANFD Python 示例 + 驱动
            ├── CAN_cpp/          标准 CAN C++ 示例 + 驱动
            ├── EtherCAT_cpp/     EtherCAT C++ 示例 + SOEM 协议栈
            ├── EtherCAT_python/  EtherCAT Python 示例
            ├── RS485_cpp/        RS485 C++ 示例
            ├── RS485_python/     RS485 Python 示例
            └── LabView_CANFD_Test/ LabVIEW 封装测试
```

## 编译与安装

```bash
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_ETHERCAT_TEST=ON \
    -DBUILD_CANFD_TEST=ON \
    -DBUILD_RS485_TEST=ON \
    -DBUILD_CAN_TEST=ON \
    -DBUILD_LABVIEW=ON

cmake --build build --config Release
cmake --install build --prefix install      # ← 上方 deploy/ 目录结构
```

各选项说明：

| CMake 选项 | 作用 |
|---|---|
| `-DBUILD_ETHERCAT_TEST=ON` | 编译 EtherCAT C++/Python 示例 |
| `-DBUILD_CANFD_TEST=ON` | 编译 CANFD C++/Python 示例 |
| `-DBUILD_RS485_TEST=ON` | 编译 RS485 C++/Python 示例 |
| `-DBUILD_CAN_TEST=ON` | 编译标准 CAN C++ 示例 |
| `-DBUILD_LABVIEW=ON` | 编译 LabVIEW 封装 |
| `-DBUILD_INTERNAL=ON` | 编译内部测试程序 |

## 示例使用

每个示例目录下有独立 README。

同一物理 RS485 串口连接多个设备时，使用 `RS485SharedBus` 将多个
`LHandProLib` 实例绑定到同一个 SDK 总线调度器。完整的 NodeID 1、2 示例见
`RS485_cpp/main_dual_node.cpp`，初始化接口仍为
`initial(LCN_RS485, node_id)`。

## 通信协议支持

| 协议 | 平台 | 驱动 |
|---|---|---|
| **EtherCAT** | Windows / Linux | SOEM + Npcap(Win) / socket(Linux) |
| **CANFD** | Windows | HCanbus.dll |
| CANFD | Linux | socketcan（默认）或 libcanbus.so |
| **RS485** | 跨平台 | 标准串口（无需驱动） |
| **标准 CAN** | Windows | PCANBasic.dll |
