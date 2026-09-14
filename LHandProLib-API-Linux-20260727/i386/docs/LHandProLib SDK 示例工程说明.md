# SDK 示例工程说明

本文档说明如何使用 SDK 压缩包中的示例工程。示例涵盖 EtherCAT、CANFD、RS485、标准 CAN 四种通讯协议，以及 Python 示例和 LabVIEW 封装。

---

## Windows 端

### 部署目录结构

```
install/
├── bin/                          ← 可执行文件 + 运行时 DLL
│   ├── LHandProLib.dll / LHandProLibd.dll
│   ├── LHandProLib_EtherCAT_Test
│   ├── LHandProLib_CANFD_Test
│   ├── LHandProLib_RS485_Test
│   ├── LHandProLib_RS485_DualNode_Test
│   ├── LHandProLib_CAN_Test
│   ├── HCanbus.dll               CANFD 驱动
│   ├── PCANBasic.dll             标准 CAN 驱动
│   ├── Packet.dll / wpcap.dll    EtherCAT 网络驱动 (Npcap)
│   └── ...
├── include/LHandProLib/          C++ / C 头文件
├── lib/                          链接库 (.lib)
├── docs/                         SDK 文档
└── share/LHandProLib/examples/   各协议示例源码
    ├── EtherCAT_cpp/
    ├── EtherCAT_python/
    ├── CANFD_cpp/
    ├── CANFD_python/
    ├── CAN_cpp/
    ├── RS485_cpp/
    ├── RS485_python/
    └── LabView_Test/
```

### 编译环境

- Visual Studio 2022（或更高版本）
- CMake 3.5+
- Python 3.10+（Python 示例需要）

### 编译示例工程

以 EtherCAT C++ 示例为例，其他协议类似：

```bash
cd install/share/LHandProLib/examples/EtherCAT_cpp
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --config Release
cmake --install . --config Release
```

编译完成后，可执行文件在 `install/bin/` 目录下。

### 各协议前提条件

| 协议 | 前提条件 |
|---|---|
| **EtherCAT** | 安装 [Npcap](https://npcap.com/)（安装时勾选 WinPcap 兼容模式） |
| **CANFD** | 安装 CANFD 驱动（HCanbus.dll 已随 SDK 提供） |
| **RS485** | 无需额外驱动，使用标准串口 |
| **标准 CAN** | 安装 [PEAK PCAN 驱动](https://www.peak-system.com/support/downloads/drivers/)（PCANBasic.dll 已随 SDK 提供） |

### 各协议示例编译

#### EtherCAT C++

```bash
cd install/share/LHandProLib/examples/EtherCAT_cpp
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --config Release
cmake --install . --config Release
# 运行
cd ../install/bin
.\LHandProLib_EtherCAT_Test.exe
```

#### CANFD C++

```bash
cd install/share/LHandProLib/examples/CANFD_cpp
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --config Release
cmake --install . --config Release
# 运行
cd ../install/bin
.\LHandProLib_CANFD_Test.exe
```

#### RS485 C++

```bash
cd install/share/LHandProLib/examples/RS485_cpp
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --config Release
cmake --install . --config Release
# 运行
cd ../install/bin
.\LHandProLib_RS485_Test.exe
# 单串口双设备（NodeID 1 和 2）
.\LHandProLib_RS485_DualNode_Test.exe
```

`LHandProLib_RS485_DualNode_Test` 只打开一个串口，创建一个
`RS485SharedBus`，并将两个 `LHandProLib` 实例分别以 NodeID 1、2 绑定到该
总线。SDK 内部统一处理发送优先级、30 ms 事务间隔、后台轮询以及接收分发；
示例代码只透明写串口，并将每段串口接收数据送入共享总线一次。

#### 标准 CAN C++

```bash
cd install/share/LHandProLib/examples/CAN_cpp
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --config Release
cmake --install . --config Release
# 运行
cd ../install/bin
.\LHandProLib_CAN_Test.exe
```

### Python 示例

Python 示例无需编译，每个示例目录下都有 `requirements.txt`，安装依赖后直接运行：

```bash
# EtherCAT Python 示例
cd install/share/LHandProLib/examples/EtherCAT_python
pip install -r requirements.txt
python main.py

# CANFD Python 示例
cd install/share/LHandProLib/examples/CANFD_python
pip install -r requirements.txt
python main.py

# RS485 Python 示例
cd install/share/LHandProLib/examples/RS485_python
pip install -r requirements.txt
python main.py
```

> Python 示例依赖 `lhandprolib_python_sdk/` 目录下的 SDK loader，已随示例一起安装。

---

## Linux 端

### 部署目录结构

```
install/
├── x86_64/                       ← x86_64 平台
│   ├── bin/                      可执行文件
│   ├── include/LHandProLib/      C++ / C 头文件
│   ├── lib/                      共享库 (.so)
│   ├── docs/                     SDK 文档
│   └── share/LHandProLib/examples/  各协议示例源码
├── i386/                         ← i386 平台（结构同上）
└── aarch64/                      ← aarch64 平台（结构同上）
```

### 编译环境

```bash
sudo apt update
sudo apt install build-essential cmake python3-pip python3-dev git
```

### 编译示例工程

以 x86_64 平台 EtherCAT C++ 示例为例：

```bash
# 解压 SDK 压缩包
tar -xzf LHandProLib-API-Linux-xxxxxxxx.tar.gz
cd x86_64

# 安装 SDK 到系统目录
sudo cp lib/libLHandProLib.so* /usr/local/lib/
sudo cp -r include/LHandProLib /usr/local/include/
sudo ldconfig

# 编译 EtherCAT 示例
cd share/LHandProLib/examples/EtherCAT_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc)
make install

# 赋予网络权限并运行
cd ../install/bin
sudo setcap cap_net_raw,cap_net_admin+ep ./LHandProLib_EtherCAT_Test
./LHandProLib_EtherCAT_Test
```

### 各协议编译与运行

#### EtherCAT C++（需要 root 或网络权限）

```bash
cd share/LHandProLib/examples/EtherCAT_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc) && make install
cd ../install/bin
sudo setcap cap_net_raw,cap_net_admin+ep ./LHandProLib_EtherCAT_Test
./LHandProLib_EtherCAT_Test
```

#### CANFD C++（需要 SocketCAN 接口）

```bash
# 确保 can0 接口已配置
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

cd share/LHandProLib/examples/CANFD_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc) && make install
cd ../install/bin
./LHandProLib_CANFD_Test
```

#### RS485 C++（需要串口权限）

```bash
# 将当前用户加入 dialout 组（需重新登录生效）
sudo usermod -aG dialout $USER

cd share/LHandProLib/examples/RS485_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc) && make install
cd ../install/bin
./LHandProLib_RS485_Test
# 单串口双设备（NodeID 1 和 2）
./LHandProLib_RS485_DualNode_Test
```

#### 标准 CAN（需要 PCAN 驱动）

```bash
# 安装 PCAN Linux 驱动（libpcanbasic.so）
# 下载：https://www.peak-system.com/support/downloads/drivers/

cd share/LHandProLib/examples/CAN_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc) && make install
cd ../install/bin
./LHandProLib_CAN_Test
```

### Python 示例

```bash
# RS485 Python 示例
cd share/LHandProLib/examples/RS485_python
pip3 install -r requirements.txt
python3 main.py

# EtherCAT Python 示例（需要权限）
cd share/LHandProLib/examples/EtherCAT_python
pip3 install -r requirements.txt
sudo python3 main.py

# CANFD Python 示例
cd share/LHandProLib/examples/CANFD_python
pip3 install -r requirements.txt
python3 main.py
```

---

## 常见问题

### Windows

- **找不到 DLL**：确保 `bin/` 目录下的 DLL 与可执行文件在同一目录，或将 `bin/` 加入 PATH
- **EtherCAT 无法通讯**：确认已安装 Npcap，且选择了 WinPcap 兼容模式
- **CANFD 打开设备失败**：确认 HCanbus.dll 在可执行文件同目录下

### Linux

- **权限不足**：EtherCAT 需要 `cap_net_raw` + `cap_net_admin` 权限；RS485 需要 `dialout` 组权限
- **找不到 libLHandProLib.so**：确认已执行 `sudo cp lib/libLHandProLib.so* /usr/local/lib/ && sudo ldconfig`
- **CANFD 接口不存在**：确认 SocketCAN 接口已配置（`ip link set can0 type can bitrate 1000000`）
- **标准 CAN 打开失败**：确认已安装 PCAN Linux 驱动（`libpcanbasic.so`）
