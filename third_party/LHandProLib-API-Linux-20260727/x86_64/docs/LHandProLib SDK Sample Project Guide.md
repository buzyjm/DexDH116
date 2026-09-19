# LHandProLib SDK Sample Project Guide

This document explains how to use the sample projects included in the SDK package. Samples cover EtherCAT, CANFD, RS485, and standard CAN communication protocols, along with Python examples and LabVIEW wrappers.

---

## Windows Platform

### Deployment Directory Structure

```
install/
├── bin/                          ← Executables + runtime DLLs
│   ├── LHandProLib.dll / LHandProLibd.dll
│   ├── LHandProLib_EtherCAT_Test
│   ├── LHandProLib_CANFD_Test
│   ├── LHandProLib_RS485_Test
│   ├── LHandProLib_RS485_DualNode_Test
│   ├── LHandProLib_CAN_Test
│   ├── HCanbus.dll               CANFD driver
│   ├── PCANBasic.dll             Standard CAN driver
│   ├── Packet.dll / wpcap.dll    EtherCAT network driver (Npcap)
│   └── ...
├── include/LHandProLib/          C++ / C header files
├── lib/                          Link libraries (.lib)
├── docs/                         SDK documentation
└── share/LHandProLib/examples/   Sample source code
    ├── EtherCAT_cpp/
    ├── EtherCAT_python/
    ├── CANFD_cpp/
    ├── CANFD_python/
    ├── CAN_cpp/
    ├── RS485_cpp/
    ├── RS485_python/
    └── LabView_Test/
```

### Build Requirements

- Visual Studio 2022 (or later)
- CMake 3.5+
- Python 3.10+ (for Python samples)

### Building Sample Projects

The build steps are the same for all protocols. Example with EtherCAT C++:

```bash
cd install/share/LHandProLib/examples/EtherCAT_cpp
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --config Release
cmake --install . --config Release
```

After building, executables are in the `install/bin/` directory.

### Protocol Prerequisites

| Protocol | Prerequisites |
|---|---|
| **EtherCAT** | Install [Npcap](https://npcap.com/) (enable WinPcap compatibility mode during install) |
| **CANFD** | Install CANFD driver (HCanbus.dll is included with SDK) |
| **RS485** | No additional driver needed, uses standard serial port |
| **Standard CAN** | Install [PEAK PCAN driver](https://www.peak-system.com/support/downloads/drivers/) (PCANBasic.dll is included with SDK) |

### Building Each Protocol

#### EtherCAT C++

```bash
cd install/share/LHandProLib/examples/EtherCAT_cpp
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --config Release
cmake --install . --config Release
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
cd ../install/bin
.\LHandProLib_RS485_Test.exe
# One serial port with NodeID 1 and 2
.\LHandProLib_RS485_DualNode_Test.exe
```

`LHandProLib_RS485_DualNode_Test` opens one serial port, creates one
`RS485SharedBus`, and binds two `LHandProLib` instances as NodeID 1 and 2. The
SDK owns send priority, the global 30 ms transaction interval, background
polling, and receive routing. The sample only writes opaque bytes and feeds each
serial receive chunk to the shared bus once.

#### Standard CAN C++

```bash
cd install/share/LHandProLib/examples/CAN_cpp
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --config Release
cmake --install . --config Release
cd ../install/bin
.\LHandProLib_CAN_Test.exe
```

### Python Samples

Python samples require no compilation. Each sample directory includes a `requirements.txt` for dependencies:

```bash
# EtherCAT Python sample
cd install/share/LHandProLib/examples/EtherCAT_python
pip install -r requirements.txt
python main.py

# CANFD Python sample
cd install/share/LHandProLib/examples/CANFD_python
pip install -r requirements.txt
python main.py

# RS485 Python sample
cd install/share/LHandProLib/examples/RS485_python
pip install -r requirements.txt
python main.py
```

> Python samples depend on the `lhandprolib_python_sdk/` SDK loader, which is installed alongside the samples.

---

## Linux Platform

### Deployment Directory Structure

```
install/
├── x86_64/                       ← x86_64 platform
│   ├── bin/                      Executables
│   ├── include/LHandProLib/      C++ / C header files
│   ├── lib/                      Shared libraries (.so)
│   ├── docs/                     SDK documentation
│   └── share/LHandProLib/examples/  Sample source code
├── i386/                         ← i386 platform (same structure)
└── aarch64/                      ← aarch64 platform (same structure)
```

### Build Requirements

```bash
sudo apt update
sudo apt install build-essential cmake python3-pip python3-dev git
```

### Building Sample Projects

Example with x86_64 EtherCAT C++ sample:

```bash
# Extract SDK package
tar -xzf LHandProLib-API-Linux-xxxxxxxx.tar.gz
cd x86_64

# Install SDK to system directories
sudo cp lib/libLHandProLib.so* /usr/local/lib/
sudo cp -r include/LHandProLib /usr/local/include/
sudo ldconfig

# Build EtherCAT sample
cd share/LHandProLib/examples/EtherCAT_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc)
make install

# Grant network permissions and run
cd ../install/bin
sudo setcap cap_net_raw,cap_net_admin+ep ./LHandProLib_EtherCAT_Test
./LHandProLib_EtherCAT_Test
```

### Building and Running Each Protocol

#### EtherCAT C++ (requires root or network capabilities)

```bash
cd share/LHandProLib/examples/EtherCAT_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc) && make install
cd ../install/bin
sudo setcap cap_net_raw,cap_net_admin+ep ./LHandProLib_EtherCAT_Test
./LHandProLib_EtherCAT_Test
```

#### CANFD C++ (requires SocketCAN interface)

```bash
# Ensure can0 interface is configured
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

cd share/LHandProLib/examples/CANFD_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc) && make install
cd ../install/bin
./LHandProLib_CANFD_Test
```

#### RS485 C++ (requires serial port permissions)

```bash
# Add current user to dialout group (requires re-login to take effect)
sudo usermod -aG dialout $USER

cd share/LHandProLib/examples/RS485_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc) && make install
cd ../install/bin
./LHandProLib_RS485_Test
# One serial port with NodeID 1 and 2
./LHandProLib_RS485_DualNode_Test
```

#### Standard CAN (requires PCAN driver)

```bash
# Install PCAN Linux driver (libpcanbasic.so)
# Download: https://www.peak-system.com/support/downloads/drivers/

cd share/LHandProLib/examples/CAN_cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install
make -j$(nproc) && make install
cd ../install/bin
./LHandProLib_CAN_Test
```

### Python Samples

```bash
# RS485 Python sample
cd share/LHandProLib/examples/RS485_python
pip3 install -r requirements.txt
python3 main.py

# EtherCAT Python sample (requires permissions)
cd share/LHandProLib/examples/EtherCAT_python
pip3 install -r requirements.txt
sudo python3 main.py

# CANFD Python sample
cd share/LHandProLib/examples/CANFD_python
pip3 install -r requirements.txt
python3 main.py
```

---

## Troubleshooting

### Windows

- **DLL not found**: Ensure DLLs in `bin/` are in the same directory as the executable, or add `bin/` to PATH
- **EtherCAT communication fails**: Verify Npcap is installed with WinPcap compatibility mode enabled
- **CANFD device open failed**: Confirm HCanbus.dll is in the same directory as the executable

### Linux

- **Permission denied**: EtherCAT requires `cap_net_raw` + `cap_net_admin` capabilities; RS485 requires `dialout` group membership
- **libLHandProLib.so not found**: Run `sudo cp lib/libLHandProLib.so* /usr/local/lib/ && sudo ldconfig`
- **CANFD interface not found**: Configure the SocketCAN interface first (`ip link set can0 type can bitrate 1000000`)
- **Standard CAN open failed**: Ensure PCAN Linux driver (`libpcanbasic.so`) is installed
