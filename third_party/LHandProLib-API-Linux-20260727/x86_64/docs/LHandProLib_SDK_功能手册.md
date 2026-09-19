# **LHandProLib SDK 功能手册**

> **版本：1.7**
>
> **目标平台：Windows/Linux，支持 EtherCAT/CANFD/RS485 通讯协议**

---

## **简介**

`LHandProLib` 是一个用于控制灵巧手（Dexterous Hand）的 C++ SDK，支持多种通讯方式（EtherCAT、CANFD、RS485），提供对灵巧手的自由度控制、传感器数据读取、回零、状态监控等功能。本手册详细说明 SDK 中所有接口、枚举类型及使用示例。

---

## **命名空间**

所有功能均定义在 `lhplib` 命名空间中

```c_cpp
namespace lhplib { ... }
```

---

## **错误码（LER）**

|错误码|宏定义|含义|
|--|--|--|
|`0`|`LER_NONE`|执行成功|
|`1`|`LER_PARAMETER`|参数错误|
|`2`|`LER_KEY_FUNC_UNINIT`|关键函数未初始化（如未设置回调）|
|`3`|`LER_GET_CONFIGURATION`|读取配置失败|
|`4`|`LER_DATA_ANOMALY`|数据异常|
|`5`|`LER_COMM_CONNECT`|通讯连接错误|
|`6`|`LER_COMM_SEND`|通讯发送错误|
|`7`|`LER_COMM_RECV`|通讯接收错误|
|`8`|`LER_COMM_DATA_FORMAT`|通讯数据格式错误|
|`9`|`LER_INVALID_PATH`|无效的文件路径|
|`10`|`LER_LOG_SAVE_FAIL`|日志文件保存失败|
|`11`|`LER_NOT_HOME`|没回零错误|
|`999`|`LER_UNKNOWN`|未知错误|

---

## **灵巧手类型（LAC）枚举**

|枚举值|含义|
|--|--|
|`LAC_DOF_6`|6自由度（默认）|
|`LAC_DOF_6_S`|6自由度(S版本)|
|`LAC_DOF_16`|16自由度（5指×3DOF + 1独立轴）|

---

## **通讯类型（LCN）枚举**

|枚举值|含义|
|--|--|
|`LCN_ECAT`|EtherCAT 通讯|
|`LCN_CANFD`|CANFD 通讯|
|`LCN_RS485`|RS485 通讯|
|`LCN_CAN`|标准CAN通讯|

---

## **6自由度灵巧手关节枚举（LAC_DOF_6，LAC_DOF_6_S）**

|枚举值|含义|
|--|--|
|`LMI6_ALL_JOINTS`|全部关节（广播）|
|`LMI6_THUMB_ABDUCTION`|大拇指侧摆|
|`LMI6_THUMB_FLEXION`|大拇指弯曲|
|`LMI6_INDEX_FLEXION`|食指弯曲|
|`LMI6_MIDDLE_FLEXION`|中指弯曲|
|`LMI6_RING_FLEXION`|无名指弯曲|
|`LMI6_PINKY_FLEXION`|小拇指弯曲|

---

## **16自由度灵巧手关节枚举（LAC_DOF_16）**

|枚举值|含义|
|--|--|
|`LMI16_ALL_JOINTS`|全部关节（广播）|
|`LMI16_THUMB_LATERAL_EX`|大拇指加强侧摆（独立轴）|
|`LMI16_THUMB_LATERAL`|大拇指侧摆|
|`LMI16_THUMB_PROXIMAL`|大拇指近节|
|`LMI16_THUMB_DISTAL`|大拇指远节|
|`LMI16_INDEX_LATERAL`|食指侧摆|
|`LMI16_INDEX_PROXIMAL`|食指近节|
|`LMI16_INDEX_DISTAL`|食指远节|
|`LMI16_MIDDLE_LATERAL`|中指侧摆|
|`LMI16_MIDDLE_PROXIMAL`|中指近节|
|`LMI16_MIDDLE_DISTAL`|中指远节|
|`LMI16_RING_LATERAL`|无名指侧摆|
|`LMI16_RING_PROXIMAL`|无名指近节|
|`LMI16_RING_DISTAL`|无名指远节|
|`LMI16_PINKY_LATERAL`|小拇指侧摆|
|`LMI16_PINKY_PROXIMAL`|小拇指近节|
|`LMI16_PINKY_DISTAL`|小拇指远节|

---

## **报警类型（LAM）枚举**

|枚举值|含义|
|--|--|
|`LAM_NULL`|无报警|
|`LAM_POS_ERR`|位置超差|
|`LAM_OVER_SPD`|超速|
|`LAM_OVER_CUR`|过流|
|`LAM_OVER_LOAD`|过载|
|`LAM_OVER_VOL`|过压|
|`LAM_UNDER_VOL`|欠压|
|`LAM_ENC_ERR`|编码器错误|
|`LAM_STALL`|堵转|
|`LAM_OTHER`|其他报警|

---

## **控制模式（LCM）枚举**

|枚举值|含义|
|--|--|
|`LCM_POSITION`|位置控制|
|`LCM_VELOCITY`|速度控制|
|`LCM_TORQUE`|力矩控制|
|`LCM_VEL_TOR`|速度+力矩混合控制|
|`LCM_POS_TOR`|位置+力矩混合控制|
|`LCM_HOME`|回零模式|

---

## **运行状态（LST）枚举**

|枚举值|含义|
|--|--|
|`LST_STOPPED`|正常停止状态|
|`LST_RUNNING`|正常运行状态|
|`LST_ALARM`|报警停止状态|
|`LST_POS_LIMIT`|正限位状态|
|`LST_NEG_LIMIT`|负限位状态|
|`LST_BOTH_LIMIT`|正负限位同时触发|
|`LST_EMG_STOP`|急停状态|
|`LST_HOMING`|回零运行状态|

---

## **传感器ID（LSS）枚举**

|枚举值|含义|
|--|--|
|`LSS_FINGER_1_1`|大拇指指尖|
|`LSS_FINGER_1_2`|大拇指指腹|
|`LSS_FINGER_2_1`|食指指尖|
|`LSS_FINGER_2_2`|食指指腹|
|`LSS_FINGER_3_1`|中指指尖|
|`LSS_FINGER_3_2`|中指指腹|
|`LSS_FINGER_4_1`|无名指指尖|
|`LSS_FINGER_4_2`|无名指指腹|
|`LSS_FINGER_5_1`|小拇指指尖|
|`LSS_FINGER_5_2`|小拇指指腹|
|`LSS_HAND_PALM`|手掌传感器|
|`LSS_MAX_COUNT`|最大传感器数量（用于数组分配）|

---

## **左右手方向（LDR）枚举**

|枚举值|含义|
|--|--|
|`LDR_HAND_RIGHT`|右手（默认）|
|`LDR_HAND_LEFT`|左手|

---

## **函数指针类型**

|类型|描述|
|--|--|
|`LogAddCallback`|日志回调函数：`void(*)(const char*)`|
|`ECSendDataCallback`|EtherCAT RPDO 发送回调：`bool(*)(const unsigned char*, unsigned int)`|
|`CANFDSendDataCallback`|CANFD 发送回调：`bool(*)(const unsigned char*, unsigned int)`|
|`RS485SendDataCallback`|RS485 发送回调：`bool(*)(const unsigned char*, unsigned int)`|
|`CANSendDataCallback`|标准 CAN 发送回调：`bool(*)(const unsigned char*, unsigned int)`|

---

## **RS485SharedBus 类接口说明**

`RS485SharedBus` 表示一条由多个 `LHandProLib` 实例共享的物理 RS485
总线。它不负责打开或关闭串口；应用程序仍负责串口生命周期以及同步写入。
SDK 在该对象内部统一完成：

- 用户控制命令、SDO、OTA 与后台状态轮询的优先级调度
- 整条物理总线唯一的 30 ms 事务间隔
- 多节点后台轮询的 NodeID 轮转
- RS485 地址头、CRC、分包/粘包处理及按 NodeID 分发反馈

一条物理串口应只创建一个 `RS485SharedBus`。该对象的生命周期必须长于所有
绑定到它的 `LHandProLib` 实例。

#### `RS485SharedBus()`

创建共享总线调度对象并启动内部总线工作线程。

#### `~RS485SharedBus()`

停止内部工作线程并释放资源。

#### `void set_send_callback(RS485SendDataCallback callback)`

设置物理串口发送回调。回调收到的是 SDK 已完成地址和 CRC 封装的字节帧，
应用程序只需同步写入串口。

#### `void set_send_callback_ex(void* callback_impl)`

设置 `std::function<bool(const unsigned char*, unsigned int)>` 形式的物理串口
发送回调。

#### `int set_receive_data(const unsigned char* data_ptr, int data_size)`

输入物理串口收到的原始字节片段。允许任意分包或粘包边界。每段物理接收数据
只应输入共享总线一次，不能再分别送入每个 SDK 实例。

#### `void close()`

停止共享总线工作线程，并取消尚未发送的请求。调用后该对象不能重新启动。

---

## **LHandProLib 类接口说明**

---

### **构造与析构函数**

#### `LHandProLib()`

构造函数，初始化内部资源和私有对象（Pimpl 模式）。

#### `~LHandProLib()`

析构函数，释放资源，自动调用 `close()`。

---

### **初始化与关闭**

#### `int initial(int mode)`

初始化灵巧手驱动程序。

- **参数**：
  - `mode`：通讯类型，取值为 `LCN_ECAT` / `LCN_CANFD` / `LCN_RS485`
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **前置条件**：
  1. 底层通讯设备已连接
  2. 已设置发送回调（`set_send_rpdo_callback`）
  3. 已设置接收数据处理（`set_tpdo_data_decode`）

---

#### `int initial(int mode, unsigned int node_id)`

初始化灵巧手驱动程序（指定节点ID）。

- **参数**：
  - `mode`：通讯类型，取值为 `LCN_CANFD` / `LCN_RS485`
  - `node_id`：CANFD或RS485的节点ID
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **前置条件**：
  1. 底层通讯设备已连接
  2. 已设置发送回调
  3. 已设置接收数据处理

---

#### `void close()`

关闭驱动程序，释放资源，断开所有连接。

---

#### `void start_monitor()`

启动电机/传感器状态监控，初始化时会自动启动。共享 RS485 模式不会为每个
SDK 实例创建独立轮询线程，而是将该节点注册到 `RS485SharedBus` 的唯一调度
线程中。可在调用 `stop_monitor()` 后手动恢复。

---

#### `void stop_monitor()`

停止后台监控线程。可在 OTA 固件升级等需要独占总线的场景下调用，避免轮询干扰。重新调用 `initial()` 或 `start_monitor()` 可恢复轮询。

---

#### `void set_dry_run_mode(bool enable)`

设置 dry-run 模式。开启后，命令数据正常生成但不通过回调发送，可用于无硬件环境下的协议调试和单元测试。

- **参数**：
  - `enable`：`true` 开启 dry-run，`false` 关闭
- **说明**：
  - dry-run 模式下 `initial()` 会跳过 SDO 读取，不会因无硬件而超时失败
  - dry-run 模式下 `move_motors` 跳过回零状态检查
  - 四种通信模式（EtherCAT/CANFD/RS485/标准CAN）均支持

---

#### `bool get_dry_run_mode()`

获取当前 dry-run 模式状态。

- **返回值**：`true` 已开启，`false` 已关闭

---

#### `void set_send_rpdo_callback(ECSendDataCallback callback)`

设置 EtherCAT RPDO 发送回调（用于输出数据）。

- **参数**：
  - `callback`：函数指针，原型为 `bool(const unsigned char* data, int size)`
    - `data`：待发送的 RPDO 数据
    - `size`：数据总长度
    - 返回值：`true` 成功，`false` 失败
- **说明**：该回调将被 `get_pre_send_rpdo_data` 触发调用（若已设置）

---

#### `void set_send_rpdo_callback_ex(void* callback_impl)`

设置 `std::function` 包装的 EtherCAT 发送回调。

- **参数**：
  - `callback_impl`：指向 `std::function<bool(const unsigned char*, unsigned int)>` 的指针
- **注意**：生命周期管理由用户负责

---

#### `int get_pre_send_rpdo_data(unsigned char* data_ptr, int* io_size)`

获取待发送的 RPDO 数据（用于 EtherCAT 输出区）。

- **参数**：
  - `data_ptr`：输出缓冲区指针
    - 若为 `nullptr`，仅输出所需缓冲区大小
  - `io_size`：输入时表示缓冲区容量，输出时返回实际需要或已写入的字节数
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **说明**：若设置了 `set_send_rpdo_callback`，此函数会自动触发发送

---

#### `int set_tpdo_data_decode(const unsigned char* data_ptr, int data_size)`

解码从 EtherCAT 接收到的 TPDO 数据（输入数据）。

- **参数**：
  - `data_ptr`：TPDO 数据指针
  - `data_size`：数据总长度（字节）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **说明**：解码后可通过 `get_now_angle` 等接口获取最新状态

---

### **发送与回调（CANFD）**

#### `void set_send_canfd_callback(CANFDSendDataCallback callback)`

设置 CANFD 发送回调。

#### `void set_send_canfd_callback_ex(void* callback_impl)`

设置 `std::function<bool(const unsigned char*, unsigned int)>` 包装的 CANFD 发送回调。

#### `int get_pre_send_canfd_data(unsigned char* data_ptr, int* io_size)`

获取待发送的 CANFD 数据（输出区）。参数语义同 `get_pre_send_rpdo_data`。

#### `int set_canfd_data_decode(const unsigned char* data_ptr, int data_size)`

解码从 CANFD 接收到的数据（输入区）。

---

#### `void set_send_rs485_callback(RS485SendDataCallback callback)`

设置 RS485 发送回调（用于输出数据）。

- **参数**：
  - `callback`：函数指针，原型为 `bool(const unsigned char* data, unsigned int size)`
    - `data`：待发送的 RS485 数据
    - `size`：数据总长度
    - 返回值：`true` 成功，`false` 失败
- **说明**：
  - 本接口保留用于单设备隐式总线模式。
  - 多设备共享一条物理总线时，应在 `RS485SharedBus` 上设置发送回调。
  - 配置后调用 `get_pre_send_rs485_data` 会自动触发发送。

---

#### `void set_send_rs485_callback_ex(void* callback_impl)`

设置 RS485 发送回调（使用 `std::function` 包装）。

- **参数**：
  - `callback_impl`：`std::function<bool(const unsigned char*, unsigned int)>` 的指针
- **注意**：用户需自行管理生命周期，确保该对象生命周期长于 LHandProLib 实例。

---

#### `void set_rs485_shared_bus(RS485SharedBus* bus)`

将当前 SDK 实例绑定到共享 RS485 总线。必须在
`initial(LCN_RS485, node_id)` 前调用。

- **参数**：
  - `bus`：共享总线对象指针；传入 `nullptr` 恢复原有单设备隐式总线模式
- **限制**：
  - 同一个共享总线中 NodeID 必须唯一
  - 共享总线对象必须在所有绑定实例销毁之后再销毁
  - 物理串口接收数据只调用一次 `RS485SharedBus::set_receive_data`

双节点示例：

```cpp
lhplib::RS485SharedBus shared_bus;
lhplib::LHandProLib hand1;
lhplib::LHandProLib hand2;

std::function<bool(const unsigned char*, unsigned int)> send_callback =
    [&serial_port](const unsigned char* data, unsigned int size) {
      return serial_port.write(data, size);
    };
shared_bus.set_send_callback_ex(&send_callback);

hand1.set_rs485_shared_bus(&shared_bus);
hand2.set_rs485_shared_bus(&shared_bus);

serial_port.setReadCallback(
    [&shared_bus](const uint8_t* data, size_t size) {
      shared_bus.set_receive_data(data, static_cast<int>(size));
    });

hand1.initial(lhplib::LCN_RS485, 1);
hand2.initial(lhplib::LCN_RS485, 2);
```

两个 `initial` 的接口形式保持不变。用户命令优先于后台轮询，后台轮询在
NodeID 1、2 之间轮转，所有发送共同遵守一条总线的 30 ms 事务间隔。

---

#### `int get_pre_send_rs485_data(unsigned char* data_ptr, int* io_size)`

获取待发送的 RS485 完整帧数据（含地址头 + CRC）。参数语义同 `get_pre_send_rpdo_data`。

- **返回数据格式**：`[地址头 2B] + [payload 64B] + [CRC 2B]` = 68 字节

---

#### `int set_rs485_data_decode(const unsigned char* data_ptr, int data_size)`

解码从 RS485 接收到的数据（输入区）。

- **说明**：
  - 单设备隐式总线模式继续使用该接口。
  - 共享总线模式应改为调用 `RS485SharedBus::set_receive_data`，并且每段
    物理串口数据只输入一次。

---

#### `void set_send_can_callback(CANSendDataCallback callback)`

设置标准 CAN 发送回调（用于 LCN_CAN 模式）。

- **参数**：
  - `callback`：函数指针，原型为 `bool(const unsigned char* data, unsigned int size)`
    - `data`：待发送的 CAN 数据
    - `size`：数据总长度
    - 返回值：`true` 成功，`false` 失败

---

#### `void set_send_can_callback_ex(void* callback_impl)`

设置标准 CAN 发送回调（使用 `std::function` 包装）。

- **参数**：
  - `callback_impl`：`std::function<bool(const unsigned char*, unsigned int)>` 的指针
- **注意**：用户需自行管理生命周期。

---

#### `int set_can_data_decode(unsigned int id, const unsigned char* data_ptr, int data_size)`

解码标准 CAN 数据（在 LCN_CAN 模式下由用户调用）。

- **参数**：
  - `id`：CAN 帧 ID
  - `data_ptr`：CAN 数据指针
  - `data_size`：数据长度（字节）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_pre_send_can_data(unsigned char* data_ptr, int* io_size)`

获取预发送区的标准 CAN 数据（dry-run 捕获）。每帧 11 字节：`[CAN_ID 小端 4B] + [data 7B]`，多帧拼接。参数语义同 `get_pre_send_rpdo_data`。

---

### **手型与方向设置**

#### `int set_hand_type(int hand_type)`

设置灵巧手类型。

- **参数**：
  - `hand_type`：取值为 `LAC_DOF_6` `LAC_DOF_6_S` `LAC_DOF_16`
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **默认值**：`LAC_DOF_6`

---

#### `int get_hand_type(int* hand_type)`

获取当前灵巧手类型。

- **参数**：
  - `hand_type`：输出参数，返回灵巧手类型
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_dof(int* total, int* active)`

获取当前灵巧手的自由度信息。

- **参数**：
  - `total`：输出参数，返回总关节数（包括被动关节）
  - `active`：输出参数，返回主动电机数量
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_hand_direction(int dir)`

设置手的方向（影响运动映射逻辑）。

- **参数**：
  - `dir`：方向，取值为 `LDR_HAND_RIGHT` 或 `LDR_HAND_LEFT`
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_hand_direction(int* dir)`

获取当前手的方向。

- **参数**：
  - `dir`：输出参数，接收方向值（`LDR_HAND_RIGHT` 或 `LDR_HAND_LEFT`）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

### **电机控制接口**

#### `int set_control_mode(int id, int mode = LCM_POSITION)`

设置电机控制模式。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播模式，设置所有电机
    - `id ≥ 1`：设置指定电机
  - `mode`：控制模式，取值参考 `LCM_*` 枚举
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_control_mode(int id, int* mode)`

获取指定电机的控制模式。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `mode`：输出参数，接收当前控制模式
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_safe_current_enable(int enable)`

设置安全电流使能状态。

- **参数**：
  - `enable`：安全电流使能状态，`1` 为使能，`0` 为去使能
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_safe_current_enable(int* enable)`

获取安全电流使能状态。

- **参数**：
  - `enable`：输出参数，接收当前安全电流使能状态，`1` 为使能，`0` 为去使能
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_home_current(int current)`

设置回零电流。

- **参数**：
  - `current`：回零电流值，单位 `%`（百分比），例如 `100` 表示 100%
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_home_current(int* current)`

获取回零电流。

- **参数**：
  - `current`：输出参数，接收当前回零电流值，单位 `%`（百分比）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_can_node_id(int node_id)`

设置CAN节点号（通过SDO写入驱动参数）。

- **参数**：
  - `node_id`：节点号
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  sdk_handle->set_can_node_id(1);
  ```

---

#### `int get_can_node_id(int* node_id)`

获取CAN节点号。

- **参数**：
  - `node_id`：输出参数，接收节点号
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  int node_id = 0;
  sdk_handle->get_can_node_id(&node_id);
  ```

---

#### `int set_canfd_arb_baudrate(int baudrate)`

设置CANFD仲裁段波特率（通过SDO写入驱动参数）。

- **参数**：
  - `baudrate`：波特率值（如 1000000）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  sdk_handle->set_canfd_arb_baudrate(1000000);
  ```

---

#### `int get_canfd_arb_baudrate(int* baudrate)`

获取CANFD仲裁段波特率。

- **参数**：
  - `baudrate`：输出参数，接收波特率值
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  int baudrate = 0;
  sdk_handle->get_canfd_arb_baudrate(&baudrate);
  ```

---

#### `int set_canfd_data_baudrate(int baudrate)`

设置CANFD数据段波特率（通过SDO写入驱动参数）。

- **参数**：
  - `baudrate`：波特率值（如 5000000）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  sdk_handle->set_canfd_data_baudrate(5000000);
  ```

---

#### `int get_canfd_data_baudrate(int* baudrate)`

获取CANFD数据段波特率。

- **参数**：
  - `baudrate`：输出参数，接收波特率值
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  int baudrate = 0;
  sdk_handle->get_canfd_data_baudrate(&baudrate);
  ```

---

#### `int set_rs485_node_id(int node_id)`

设置RS485节点号（通过SDO写入驱动参数）。

- **参数**：
  - `node_id`：节点号
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  sdk_handle->set_rs485_node_id(1);
  ```

---

#### `int get_rs485_node_id(int* node_id)`

获取RS485节点号。

- **参数**：
  - `node_id`：输出参数，接收节点号
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  int node_id = 0;
  sdk_handle->get_rs485_node_id(&node_id);
  ```

---

#### `int set_rs485_baudrate(int baudrate)`

设置RS485波特率（通过SDO写入驱动参数）。

- **参数**：
  - `baudrate`：波特率值（如 500000）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  sdk_handle->set_rs485_baudrate(500000);
  ```

---

#### `int get_rs485_baudrate(int* baudrate)`

获取RS485波特率。

- **参数**：
  - `baudrate`：输出参数，接收波特率值
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  int baudrate = 0;
  sdk_handle->get_rs485_baudrate(&baudrate);
  ```

---

#### `int set_enable(int id, int enable)`

使能或禁用电机驱动。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播设置所有电机
    - `id ≥ 1`：设置指定电机
  - `enable`：使能标志，`1` 表示使能，`0` 表示禁用
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_enable(int id, int* enable)`

获取电机当前使能状态。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `enable`：输出参数，`1` 表示已使能，`0` 表示已禁用
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_position_reached(int id, int* reached)`

获取电机是否已到达目标位置。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `reached`：输出参数，`1` 表示已到位，`0` 表示仍在运动
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_torque_reached(int id, int* reached)`

获取力矩到位是否已到位。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `reached`：输出参数，`1` 表示已到位，`0` 表示没运动
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_clear_alarm(int id = 0)`

清除电机报警状态。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播清除所有电机报警
    - `id ≥ 1`：清除指定电机报警
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_now_alarm(int id, int* alarm)`

获取电机当前报警状态码。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `alarm`：输出参数，接收报警码（具体含义参考硬件手册）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int home_motors(int id)`

启动电机回零操作。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：所有电机回零
    - `id ≥ 1`：指定电机回零
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_move_no_home(int enable = 0)`

配置是否允许未回零情况下运动。

- **参数**：
  - `enable`：使能标志
    - `0`：不允许未回零运动（默认）
    - `1`：允许未回零运动
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int play_gesture(int gesture_id, int velocity, int current)`

执行预定义手势。

- **参数**：
  - `gesture_id`：要执行的手势 ID
  - `velocity`：目标速度（单位：当量/秒）
  - `current`：最大电流（单位：‰，千分之一）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  // 执行手势1，目标速度 20000 当量/秒，最大电流 1000‰
  sdk_handle->play_gesture(1, 20000, 1000);
  ```

---

#### `int get_limit_target_angle(int id, float* min_angle, float* max_angle)`

获取电机目标角度的上下限。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `min_angle`：输出参数，最小允许角度（单位：°）
  - `max_angle`：输出参数，最大允许角度（单位：°）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_target_angle(int id, float angle)`

设置电机目标角度。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播设置所有电机
    - `id ≥ 1`：设置指定电机
  - `angle`：目标角度，单位为 °（角度制）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **注意**：实际可达范围由机械结构决定

---

#### `int get_target_angle(int id, float* angle)`

获取最近一次设置的目标角度。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `angle`：输出参数，接收目标角度值（单位：°）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_target_position(int id, int position)`

设置电机目标位置（行程当量）。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播设置所有电机
    - `id ≥ 1`：设置指定电机
  - `position`：目标位置，单位为“当量”，取值范围 `[0, 10000]`
    - `0` 表示起始位
    - `10000` 表示满行程
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **注意**：实际行程由机械结构和校准决定

---

#### `int get_target_position(int id, int* position)`

获取最近一次设置的目标位置（行程当量）。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `position`：输出参数，接收目标位置值（单位：当量，范围 `[0, 10000]`）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_velocity(int id, float velocity)`

设置电机目标速度（默认使用角度速度单位）。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播设置所有电机
    - `id ≥ 1`：设置指定电机
  - `velocity`：目标速度，单位为 °/s
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **注意**：实际速度受硬件限制；推荐使用 `set_angular_velocity` 以明确语义

---

#### `int get_velocity(int id, float* velocity)`

获取最近一次设置的目标速度（默认返回角度速度）。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `velocity`：输出参数，接收目标速度值（单位：°/s）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **注意**：推荐使用 `get_angular_velocity` 以明确语义

---

#### `int set_angular_velocity(int id, float velocity)`

设置电机目标速度（单位：°/s）。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播设置所有电机
    - `id ≥ 1`：设置指定电机
  - `velocity`：目标角速度，单位为 °/s
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **说明**：此接口与 `set_velocity` 功能一致，但命名更清晰，建议优先使用

---

#### `int get_angular_velocity(int id, float* velocity)`

获取最近一次设置的目标速度（单位：°/s）。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `velocity`：输出参数，接收目标角速度值（单位：°/s）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_position_velocity(int id, int velocity)`

设置电机目标速度（行程当量速度）。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播设置所有电机
    - `id ≥ 1`：设置指定电机
  - `velocity`：目标速度，单位为 **当量/秒**（即每秒移动的当量数）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_position_velocity(int id, int* velocity)`

获取最近一次设置的目标速度（行程当量速度）。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `velocity`：输出参数，接收目标速度值（单位：当量/秒）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_max_current(int id, int current)`

设置电机最大允许电流。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播设置所有电机
    - `id ≥ 1`：设置指定电机
  - `current`：最大电流限制，单位为 ‰ 千分比
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **注意**：用于过流保护和力矩控制上限

---

#### `int get_max_current(int id, int* current)`

获取最近一次设置的最大电流值。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `current`：输出参数，接收最大电流值（单位：‰ 千分比）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int move_motors(int id = 0)`

驱动电机执行已设置的运动指令。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：所有电机开始运动
    - `id ≥ 1`：指定电机开始运动
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **说明**：需先调用 `set_target_angle` 等设置目标值

---

#### `int stop_motors(int id = 0)`

停止电机运动。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：所有电机停止运动
    - `id ≥ 1`：指定电机停止运动
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

### **状态与反馈接口**

#### `int get_now_status(int id, int* status)`

获取电机当前运行状态。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `status`：输出参数，接收运行状态码（参考 `LST_*` 枚举）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_firmware_version(float* version)`

获取灵巧手固件版本号。

- **参数**：
  - `version`：输出参数，接收固件版本号（例如：0.32 表示版本 0.32）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  float ver;
  sdk_handle->get_firmware_version(&ver);
  ```

---

#### `int get_serial_number(char* sn, int size)`

获取灵巧手SN码（通过SDO读取，初始化时自动读取）。

- **参数**：
  - `sn`：输出缓冲区指针，接收SN码字符串
  - `size`：缓冲区大小
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```cpp
  char buf[64];
  sdk_handle->get_serial_number(buf, sizeof(buf));
  std::string sn(buf);
  ```

---

#### `int get_now_angle(int id, float* angle)`

获取电机当前角度位置。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `angle`：输出参数，接收当前角度（单位：°）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_now_position(int id, int* position)`

获取电机当前行程位置（当量）。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `position`：输出参数，接收当前行程位置（单位：当量，范围 `[0, 10000]`）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_now_velocity(int id, float* velocity)`

获取电机当前实际速度。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `velocity`：输出参数，接收当前速度（单位：°/s）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **注意**：此接口返回角度速度，推荐使用 `get_now_angular_velocity` 以明确语义

---

#### `int get_now_angular_velocity(int id, float* velocity)`

获取电机当前实际速度（单位：°/s）。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `velocity`：输出参数，接收当前角速度（单位：°/s）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_now_position_velocity(int id, int* velocity)`

获取电机当前实际速度（当量速度）。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `velocity`：输出参数，接收当前速度（单位：当量/秒）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_now_current(int id, int* current)`

获取电机当前电流。

- **参数**：
  - `id`：电机 ID，取值范围 `[1, DOF]`
  - `current`：输出参数，接收当前电流值（单位：‰ 千分比）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int sync_position_to_target(int id = 0)`

同步当前位置到目标位置。

- **参数**：
  - `id`：电机 ID，取值范围 `[0, DOF]`
    - `id = 0`：广播同步所有电机
    - `id ≥ 1`：同步指定电机
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  // 同步电机1的当前位置到目标位置
  sdk_handle->sync_position_to_target(1);
  // 同步所有电机的当前位置到目标位置
  sdk_handle->sync_position_to_target(0);
  ```

---

#### `int set_sensor_enable(int enable)`

开启或关闭触觉传感器监控（用于控制后台解析与数据采集负载）。

- **参数**：
  - `enable`：`1` 开启；`0` 关闭
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int set_sensor_data_format(int format)`

设置传感器数据格式。

- **参数**：
  - `format`：格式类型，默认为 0
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  sdk_handle->set_sensor_data_format(0);
  ```

---

#### `int set_sensor_order(const int* order, int size)`

设置传感器的映射顺序（按照大拇指、食指、中指、无名指、小拇指的顺序）。

- **参数**：
  - `order`：指向整数数组的指针
  - `size`：数组大小，范围 [1, LSS_MAX_COUNT]
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  int order[6] = {LSS_FINGER_1_1, LSS_FINGER_2_1, LSS_FINGER_3_1,
                  LSS_FINGER_4_1, LSS_FINGER_5_1, LSS_HAND_PALM};
  sdk_handle->set_sensor_order(order, 6);
  ```

---

### **触觉传感器接口**

#### `int get_finger_sensor_pos(int sensor_id, float* x_values, float* y_values, int* io_count)`

获取触觉传感器分布坐标（归一化 `[0.0, 1.0]`）。

- **参数**：
  - `sensor_id`：传感器 ID，取值参考 `LSS_*` 枚举
  - `x_values`：输出参数，X 坐标数组指针（可为 `nullptr` 仅获取数量）
  - `y_values`：输出参数，Y 坐标数组指针
  - `io_count`：输入时表示缓冲区大小，输出时返回实际元素数量
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  // 1. 获取count大小
  int count = 0;
  sdk_handle->get_finger_sensor_pos(sensor_id, nullptr, nullptr, &count);
  // 2. 获取实际坐标
  std::vector<float> x(count), y(count);
  sdk_handle->get_finger_sensor_pos(sensor_id, x.data(), y.data(), &count);
  ```

---

#### `int get_finger_pressure(int sensor_id, float* pressure_list, int* io_count)`

获取传感器压力值（范围 `[0.0, 1.0]`）。

- **参数**：
  - `sensor_id`：传感器 ID
  - `pressure_list`：输出参数，压力值数组指针（可为 `nullptr` 仅获取数量）
  - `io_count`：输入为缓冲区大小，输出为实际数量
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  // 1. 获取count大小
  int count = 0;
  sdk_handle->get_finger_pressure(sensor_id, nullptr, &count);
  // 2. 获取实际压力数组
  std::vector<float> pressures(count);
  sdk_handle->get_finger_pressure(sensor_id, pressures.data(), &count);
  ```

---

#### `int set_finger_pressure_reset()`

重置所有传感器基准值（差值从此刻开始计算）。

- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  sdk_handle->set_finger_pressure_reset();
  ```

---

#### `int get_finger_normal_force(int sensor_id, float* normal_force)`

获取法向力（垂直于表面）。

- **参数**：
  - `sensor_id`：传感器 ID
  - `normal_force`：输出参数，接收法向力
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  float normal_force;
  sdk_handle->get_finger_normal_force(sensor_id, &normal_force);
  ```

---

#### `int get_finger_normal_force_ex(int sensor_id, float* normal_force_list, int* io_count)`

获取指定位置的触觉传感器法向力数组。

- **参数**：
  - `sensor_id`：传感器的id,通过文件顶部的枚举输入
  - `normal_force_list`：返回的法向力数组指针,力的范围是[0.0-1.0]
  - `io_count`：返回的数组长度
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  // 1. 获取count大小
  int count = 0;
  sdk_handle->get_finger_normal_force_ex(sensor_id, nullptr, &count);
  // 2. 获取实际法向力数组
  std::vector<float> normal_force(count);
  sdk_handle->get_finger_normal_force_ex(sensor_id, normal_force.data(), &count);
  ```

---

#### `int get_finger_tangential_force(int sensor_id, float* tangential_force)`

获取切向力（平行于表面）。

- **参数**：
  - `sensor_id`：传感器 ID
  - `tangential_force`：输出参数，接收切向力
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  float tangential_force;
  sdk_handle->get_finger_tangential_force(sensor_id, &tangential_force);
  ```

---

#### `int get_finger_tangential_force_ex(int sensor_id, float* tangential_force_list, int* io_count)`

获取指定位置的触觉传感器切向力数组。

- **参数**：
  - `sensor_id`：传感器的id,通过文件顶部的枚举输入
  - `tangential_force_list`：返回的切向力数组指针,力的范围是[0.0-1.0]
  - `io_count`：返回的数组长度
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  // 1. 获取count大小
  int count = 0;
  sdk_handle->get_finger_tangential_force_ex(sensor_id, nullptr, &count);
  // 2. 获取实际切向力数组
  std::vector<float> tangential_force(count);
  sdk_handle->get_finger_tangential_force_ex(sensor_id, tangential_force.data(), &count);
  ```

---

#### `int get_finger_force_direction(int sensor_id, float* force_direction)`

获取受力方向。

- **参数**：
  - `sensor_id`：传感器 ID
  - `force_direction`：输出参数，接收方向角度（范围 `[0, 360]`，单位：°）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  float force_direction;
  sdk_handle->get_finger_force_direction(sensor_id, &force_direction);
  ```

---

#### `int get_finger_force_direction_ex(int sensor_id, float* force_direction_list, int* io_count)`

获取指定位置的触觉传感器受力方向数组。

- **参数**：
  - `sensor_id`：传感器的id,通过文件顶部的枚举输入
  - `force_direction_list`：返回的受力方向数组指针,方向的范围是[0-360]
  - `io_count`：返回的数组长度
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  // 1. 获取count大小
  int count = 0;
  sdk_handle->get_finger_force_direction_ex(sensor_id, nullptr, &count);
  // 2. 获取实际受力方向数组
  std::vector<float> force_direction(count);
  sdk_handle->get_finger_force_direction_ex(sensor_id, force_direction.data(), &count);
  ```

---

#### `int get_finger_proximity(int sensor_id, float* proximity)`

获取接近程度。

- **参数**：
  - `sensor_id`：传感器 ID
  - `proximity`：输出参数，接收接近程度（范围 `[0.0, 1.0]`）
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  float proximity;
  sdk_handle->get_finger_proximity(sensor_id, &proximity);
  ```

---

#### `int get_finger_proximity_ex(int sensor_id, float* proximity_list, int* io_count)`

获取指定位置的触觉传感器接近程度数组。

- **参数**：
  - `sensor_id`：传感器的id,通过文件顶部的枚举输入
  - `proximity_list`：返回的接近程度数组指针
  - `io_count`：返回的数组长度
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`
- **示例**：
  ```c_cpp
  // 1. 获取count大小
  int count = 0;
  sdk_handle->get_finger_proximity_ex(sensor_id, nullptr, &count);
  // 2. 获取实际接近程度数组
  std::vector<float> proximity(count);
  sdk_handle->get_finger_proximity_ex(sensor_id, proximity.data(), &count);
  ```

---

### **SDO参数读写接口**

#### `int set_sdo_drive_param(unsigned int index, unsigned char subindex, unsigned int value)`

通过SDO设置驱动参数。

- **参数**：
  - `index`：参数索引
  - `subindex`：参数子索引
  - `value`：参数值
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int get_sdo_drive_param(unsigned int index, unsigned char subindex, unsigned int* value)`

通过SDO获取驱动参数。

- **参数**：
  - `index`：参数索引
  - `subindex`：参数子索引
  - `value`：输出参数，接收参数值
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int save_sdo_drive_param()`

保存SDO驱动参数（发送保存指令，兼容新旧固件版本）。

- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

### **日志系统**

#### `void log_on(bool on = false, int maxsize = 10000)`

开启或关闭日志打印功能。

- **参数**：
  - `on`：`true` 开启日志，`false` 关闭（默认）
  - `maxsize`：最大缓存日志条数，防止内存溢出

---

#### `int log_send(int* cmd, int count)`

设置需要打印的发送指令地址过滤列表。

- **参数**：
  - `cmd`：指令地址数组指针
    - `nullptr` 表示打印所有发送数据
  - `count`：数组元素个数
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `int log_recv(int* cmd, int count)`

设置需要打印的接收指令地址过滤列表。

- **参数**：
  - `cmd`：指令地址数组指针
    - `nullptr` 表示打印所有接收数据
  - `count`：数组元素个数
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `void log_reset(bool send = true, bool recv = true)`

清空日志过滤地址列表。

- **参数**：
  - `send`：是否清空发送过滤列表
  - `recv`：是否清空接收过滤列表

---

#### `int log_save(const char* file_name)`

将当前日志保存到文件。

- **参数**：
  - `file_name`：文件路径
- **返回值**：
  - `0`：执行成功
  - 其他：参考错误码 `LER_*`

---

#### `void log_clear()`

清空当前内存中的日志记录。

---

#### `void set_log_callback(LogAddCallback callback)`

设置日志回调函数（实时接收日志字符串）。

- **参数**：
  - `callback`：函数指针，原型为 `void(const char*)`
    - 输入参数为日志字符串（以 `\n` 结尾）
- **示例**：
  ```c_cpp
  sdk_handle->set_log_callback([](const char* msg) {
      printf("[SDK_LOG] %s", msg);
  });
  ```

---

## **使用流程示例（EtherCAT 通讯）**

```c_cpp
#include "LHandProLib.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <functional>
#include "EthercatMaster.h" // EtherCAT主站库

int main() {
  /****** 初始化 ******/
  // 初始化LHandProLib对象
  auto sdk_handle = std::make_shared<lhplib::LHandProLib>();
  // 初始化EtherCAT主站对象
  auto ec_master_ = std::make_shared<EthercatMaster>();

  // 初始化EtherCAT主站
  std::vector<std::string> names = ec_master_->scanNetworkInterfaces();
  int channel_index = 0; // 根据扫描到的网口列表选择正确的通道
  if (!ec_master_->init(channel_index)) {
    std::cout << "初始化失败" << std::endl;
    return -1;
  }
  std::cout << "连接成功" << std::endl;

  // EtherCAT主站进入OP
  if (!ec_master_->start()) {
    std::cout << "启动设备失败" << std::endl;
    return -1;
  }

  int slave_id = 1; // 通过 getSlaveInfoList() 选择目标从站 ID
  const auto slave_list = ec_master_->getSlaveInfoList();
  for (const auto& slave : slave_list) {
    std::cout << "从站 " << slave.index << ": " << slave.name << std::endl;
  }

  // EtherCAT主站启动后台数据交换通信（非阻塞）
  ec_master_->run();
  

  /****** 设置发送RPDO回调以及TPDO的处理 ******/
  // EtherCAT的发送函数
  auto send_func = [ec_master_, slave_id](const unsigned char* data, unsigned int size) {
    return ec_master_->setSlaveOutputs(slave_id, data, size);
  };
  // 使用std::function包装
  std::function<bool(const unsigned char*, unsigned int)> func = send_func;
  
  // 设置EtherCAT发送数据的回调
  sdk_handle->set_send_rpdo_callback_ex(&func);
  
  // 建立监控线程, 通过接口解析TPDO数据
  std::atomic<bool> stop{false};
  std::thread t_monitor([=, &stop]() {
    while (!stop) {
      // 循环获取TPDO的数据, 并交由SDK进行解析
      int input_size = ec_master_->getSlaveInputSize(slave_id);
      std::vector<uint8_t> in_buffer(input_size);
      if (ec_master_->getSlaveInputs(slave_id, in_buffer.data(), input_size)) {
        sdk_handle->set_tpdo_data_decode(in_buffer.data(), input_size);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // 初始化库
  if (sdk_handle->initial(lhplib::LCN_ECAT) != lhplib::LER_NONE) {
    std::cerr << "LHandProLib 初始化失败！" << std::endl;
    return -1;
  }


  /****** 运动的接口调用 ******/
  // 设置控制模式
  sdk_handle->set_control_mode(0, lhplib::LCM_POSITION);
  // 全部使能
  sdk_handle->set_enable(0, 1);


  // 获取当前自由度
  int dof_total = 0, dof_active = 0;
  sdk_handle->get_dof(&dof_total, &dof_active);
  
  // 每个关节的目标角度
  std::vector<float> target_angles = {30.0f, 10.0f, 60.0f, 60.0f, 60.0f, 60.0f};
  
  // 设置运动参数
  for (int i = 0; i < dof_active; ++i) {
    sdk_handle->set_target_angle(i + 1, target_angles[i]);  // 设置目标角度
    sdk_handle->set_angular_velocity(i + 1, 200);           // 设置速度（200°/s）
    sdk_handle->set_max_current(i + 1, 1000);                // 设置最大电流限制（1000‰）
  }
   // 驱动所有电机运动
  sdk_handle->move_motors();                        
  // 等待运动结束
  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

  // 获取当前角度
  for (int i = 0; i < dof_active; ++i) {
    float angle = 0;
    if (sdk_handle->get_now_angle(i + 1, &angle) == lhplib::LER_NONE) {
      std::cout << "当前关节: " << i + 1 << " 角度：" << angle << "°" << std::endl;
    }
  }

  /****** 传感器的接口调用 ******/
  // 读取大拇指指尖压力分布
  int count = 0;
  sdk_handle->get_finger_pressure(lhplib::LSS_FINGER_1_1, nullptr, &count);
  std::vector<float> pressures(count);
  sdk_handle->get_finger_pressure(lhplib::LSS_FINGER_1_1, pressures.data(), &count);

  for (int i = 0; i < count; ++i) {
      std::cout << "传感器: " << i << " 压力: " << pressures[i] << std::endl;
  }
  
  // 读取法向力
  float normal_force;
  sdk_handle->get_finger_normal_force(lhplib::LSS_FINGER_1_1, &normal_force);
  std::cout << "传感器: " << lhplib::LSS_FINGER_1_1 << " 法向力: " << normal_force << std::endl;
  
  // 读取切向力
  float tangential_force;
  sdk_handle->get_finger_tangential_force(lhplib::LSS_FINGER_1_1, &tangential_force);
  std::cout << "传感器: " << lhplib::LSS_FINGER_1_1 << " 切向力: " << tangential_force << std::endl;
  
  // 读取切向力方向
  float force_direction;
  sdk_handle->get_finger_force_direction(lhplib::LSS_FINGER_1_1, &force_direction);
  std::cout << "传感器: " << lhplib::LSS_FINGER_1_1 << " 切向力方向: " << force_direction << std::endl;
  
  // 读取接近程度
  float proximity;
  sdk_handle->get_finger_proximity(lhplib::LSS_FINGER_1_1, &proximity);
  std::cout << "传感器: " << lhplib::LSS_FINGER_1_1 << " 接近程度: " << proximity << std::endl;

  // 关闭
  sdk_handle->close();

  return 0;
}
```

## **其他示例**

详见SDK开发包中的各通讯版本的示例工程。

> LHandProLib-API-share-LHandProLib-examples

```
LHandProLib-API
 ├── bin
 │   ├── LHandProLib.dll
 │   ├── LHandProLib.pdb
 │   ├── LHandProLibd.dll
 │   ├── LHandProLibd.pdb
 │   ├── LHandProLib_EtherCAT_Test.exe
 │   ├── LHandProLib_RS485_Test.exe
 │   └── LHandProLib_RS485_DualNode_Test.exe
 ├── include
 │   └── LHandProLib
 │       └── LHandProLib.hpp
 │       └── LHandProLib.h
 ├── lib
 │   ├── LHandProLib.lib
 │   └── LHandProLibd.lib
 └── share
     └── LHandProLib
         └── examples
             ├── EtherCAT_cpp
             ├── CANFD_cpp
             ├── RS485_cpp
             └── EtherCAT_python
```

---

## **常见问题**

### Q: 为什么 `initial()` 返回 `LER_KEY_FUNC_UNINIT`？

A: 未设置 `set_send_rpdo_callback` 回调函数。

### Q: 如何查看通讯数据？

A: 使用 `log_on(true)` 并调用 `log_send(nullptr, 0)` 和 `log_recv(nullptr, 0)` 打印所有数据。

### Q: 传感器数据不更新？

A: 确保已周期性调用 `set_tpdo_data_decode`（EtherCAT）。

### Q: 同一串口连接两个 RS485 设备时，为什么不能分别运行两个轮询循环？

A: 两个独立循环没有共享发送时序，会抢占同一条半双工总线。请创建一个
`RS485SharedBus`，将两个 SDK 实例分别绑定为不同 NodeID，并将物理串口接收
数据只送入共享总线一次。

---

## **版本更新记录**

- **v1.0**：初始版本, 支持 EtherCAT, 6/15 DOF, 触觉传感器读取, 日志系统。
- **v1.1**：新增当量控制接口, 优化EtherCAT C++示例程序。
- **v1.2**：提供C封装头文件, 并提供EtherCAT python示例程序, 优化部分实现逻辑, 增强稳定性。
- **v1.3**：新增CANFD通讯支持。
- **v1.4**：完善文档, 修复已知问题。
- **v1.5**：完善传感器接口。
- **v1.6**：新增SN码读取、CANFD波特率/CAN节点号配置、SDO参数读写接口，更新传感器映射示例。
- **v1.7**：新增 `RS485SharedBus`，支持多个 SDK 实例共享一条 RS485 物理总线，并增加 NodeID 1、2 双设备示例。

---

*文档版本：v1.7 | 更新日期：2026年07月24日*
