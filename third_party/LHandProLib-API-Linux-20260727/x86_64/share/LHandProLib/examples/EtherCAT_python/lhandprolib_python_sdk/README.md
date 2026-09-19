# LHandProLib Python SDK

灵巧手的 Python 封装库，通过 ctypes 调用原生 C API（`LHandProLib.dll` / `libLHandProLib.so`）。

## 文件结构

| 文件 | 作用 |
|---|---|
| **`lhandprolib_loader.py`** | ctypes DLL/SO 加载器, 错误码和常量定义（`LAC_DOF_6`, `LCN_CANFD` 等） |
| **`lhandprolib_wrapper.py`** | `PyLHandProLib` 类：逐函数封装 C API，提供类型安全和错误检查 |
| **`controller.py`** | 工厂类 `LHandProController`：根据通信模式返回对应的控制器实例 |
| **`controller_base.py`** | `BaseLHandProController` 基类：共享的连接/运动/断开逻辑 |
| **`controller_canfd.py`** | `CANFDController`：CANFD 传输层实现 |
| **`controller_ecat.py`** | `EtherCATController`：EtherCAT 传输层实现 |
| **`controller_rs485.py`** | `RS485Controller`：RS485 传输层实现 |
| **`controller_canfd_entry.py`** | CANFD 控制器的独立入口封装 |
| **`controller_ecat_entry.py`** | EtherCAT 控制器的独立入口封装 |
| **`controller_rs485_entry.py`** | RS485 控制器的独立入口封装 |
| **`defaults.py`** | 默认参数常量 |
| **`__init__.py`** | 空文件，标记为包 |

## 调用层级

```
lhandprolib_loader.py       ← ctypes 加载 DLL/SO，定义常量和错误码
        ↓
lhandprolib_wrapper.py      ← PyLHandProLib 类，封装每个 C 函数
        ↓
controller_base.py          ← BaseLHandProController 基类：连接/使能/回零/运动/断开
   ┌──────┼──────┐
   │      │      │
 CANFD   ECAT   RS485       ← 各通信模式下的传输层实现
```

## 使用方式

### 方式一：高级控制器（推荐）

```python
from lhandprolib_python_sdk.controller import LHandProController

with LHandProController(communication_mode="CANFD") as controller:
    controller.connect(enable_motors=True, home_motors=True)
    dof_total, dof_active = controller.get_dof()
    controller.move_to_positions([10000, 0, 0, 0, 0, 0])
```

### 方式二：直接使用 PyLHandProLib

```python
from lhandprolib_python_sdk.lhandprolib_wrapper import PyLHandProLib

lib = PyLHandProLib()
lib.initial_ex(LCN_CANFD, 1)
# ... 自行处理发送/接收回调
```

## 常量

所有常量和错误码定义在 `lhandprolib_loader.py` 中，可从 `lhandprolib_wrapper` 导入：

```python
from lhandprolib_python_sdk.lhandprolib_wrapper import (
    LAC_DOF_6, LAC_DOF_6_S, LAC_DOF_16,   # 手型
    LCN_ECAT, LCN_CANFD, LCN_RS485,       # 通信模式
    LSS_FINGER_1_1, LSS_HAND_PALM,         # 传感器 ID
    LER_NONE, LER_PARAMETER,               # 错误码
)
```
