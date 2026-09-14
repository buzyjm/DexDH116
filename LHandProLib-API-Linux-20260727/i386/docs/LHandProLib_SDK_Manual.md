# **LHandProLib SDK Manual**

> **Version：1.7**
>
> **Target Platforms: Windows/Linux, supporting EtherCAT/CANFD/RS485 communication protocols**

---

## **Introduction**

`LHandProLib` is a C++ SDK for controlling dexterous hands, supporting multiple communication protocols (EtherCAT, CANFD, RS485). It offers core functionalities including dexterous hand degree-of-freedom (DoF) control, sensor data acquisition, homing, and status monitoring. This manual provides detailed descriptions of all interfaces, enumeration types, and usage examples included in the SDK.

---

## **Namespace**

All functionalities are defined within the lhplib namespace

```c_cpp
namespace lhplib { ... }
```

---

## **Error Codes (LER)**

|Error Code|Macro Definition|Description|  
|--|--|--|  
|`0`|`LER_NONE`|Execution Successful|  
|`1`|`LER_PARAMETER`|Invalid Parameter|  
|`2`|`LER_KEY_FUNC_UNINIT`|Key Function Not Initialized (e.g., callback not set)|  
|`3`|`LER_GET_CONFIGURATION`|Failed to Read Configuration|  
|`4`|`LER_DATA_ANOMALY`|Data Anomaly Error|  
|`5`|`LER_COMM_CONNECT`|Communication Connection Error|  
|`6`|`LER_COMM_SEND`|Communication Transmission Error|  
|`7`|`LER_COMM_RECV`|Communication Reception Error|  
|`8`|`LER_COMM_DATA_FORMAT`|Communication Data Format Error|  
|`9`|`LER_INVALID_PATH`|Invalid File Path|  
|`10`|`LER_LOG_SAVE_FAIL`|Failed to Save Log File|  
|`11`|`LER_NOT_HOME`|Homing Not Completed Error|  
|`999`|`LER_UNKNOWN`|Unknown Error|

---

## **Hand Type (LAC) Enumeration**

|Enumeration Value|Description|
|--|--|
|`LAC_DOF_6`|6 DoF (Default)|
|`LAC_DOF_6_S`|6 DoF (S version)|
|`LAC_DOF_16`|16 DoF (5 fingers × 3DOF + 1 independent axis)|

---

## **Communication Type (LCN) Enumeration**

|Enumeration Value|Description|
|--|--|
|`LCN_ECAT`|EtherCAT Communication|
|`LCN_CANFD`|CANFD Communication|
|`LCN_RS485`|RS485 Communication|
|`LCN_CAN`|Standard CAN Communication|

---

## **6-DOF Hand Joint Enumeration (LAC_DOF_6, LAC_DOF_6_S)**

|Enumeration Value|Description|
|--|--|
|`LMI6_ALL_JOINTS`|All joints (broadcast)|
|`LMI6_THUMB_ABDUCTION`|Thumb abduction|
|`LMI6_THUMB_FLEXION`|Thumb flexion|
|`LMI6_INDEX_FLEXION`|Index finger flexion|
|`LMI6_MIDDLE_FLEXION`|Middle finger flexion|
|`LMI6_RING_FLEXION`|Ring finger flexion|
|`LMI6_PINKY_FLEXION`|Pinky finger flexion|

---

## **16-DOF Hand Joint Enumeration (LAC_DOF_16)**

|Enumeration Value|Description|
|--|--|
|`LMI16_ALL_JOINTS`|All joints (broadcast)|
|`LMI16_THUMB_LATERAL_EX`|Thumb lateral extended (independent axis)|
|`LMI16_THUMB_LATERAL`|Thumb lateral (abduction)|
|`LMI16_THUMB_PROXIMAL`|Thumb proximal|
|`LMI16_THUMB_DISTAL`|Thumb distal|
|`LMI16_INDEX_LATERAL`|Index finger lateral|
|`LMI16_INDEX_PROXIMAL`|Index finger proximal|
|`LMI16_INDEX_DISTAL`|Index finger distal|
|`LMI16_MIDDLE_LATERAL`|Middle finger lateral|
|`LMI16_MIDDLE_PROXIMAL`|Middle finger proximal|
|`LMI16_MIDDLE_DISTAL`|Middle finger distal|
|`LMI16_RING_LATERAL`|Ring finger lateral|
|`LMI16_RING_PROXIMAL`|Ring finger proximal|
|`LMI16_RING_DISTAL`|Ring finger distal|
|`LMI16_PINKY_LATERAL`|Pinky finger lateral|
|`LMI16_PINKY_PROXIMAL`|Pinky finger proximal|
|`LMI16_PINKY_DISTAL`|Pinky finger distal|

---

## **Alarm Type (LAM) Enumeration**

|Enumeration Value|Description|
|--|--|
|`LAM_NULL`|No alarm|
|`LAM_POS_ERR`|Position error|
|`LAM_OVER_SPD`|Over speed|
|`LAM_OVER_CUR`|Over current|
|`LAM_OVER_LOAD`|Over load|
|`LAM_OVER_VOL`|Over voltage|
|`LAM_UNDER_VOL`|Under voltage|
|`LAM_ENC_ERR`|Encoder error|
|`LAM_STALL`|Stall|
|`LAM_OTHER`|Other alarm|

---

## **Control Mode (LCM) Enumeration**

|Enumeration Value Description|Description|
|--|--|
|`LCM_POSITION`|Position Control|
|`LCM_VELOCITY`|Velocity Control|
|`LCM_TORQUE`|Torque Control|
|`LCM_VEL_TOR`|Velocity + Torque Hybrid Control|
|`LCM_POS_TOR`|Position + Torque Hybrid Control|
|`LCM_HOME`|Homing Mode|

---

## **Operating State (LST) Enumeration**

|Enumeration Value Description|Description|
|--|--|
|`LST_STOPPED`|Normal Stop State|
|`LST_RUNNING`|Normal Operating State|
|`LST_ALARM`|Alarm Stop State|
|`LST_POS_LIMIT`|Positive Limit State|
|`LST_NEG_LIMIT`|Negative Limit State|
|`LST_BOTH_LIMIT`|Both Positive and Negative Limits Triggered|
|`LST_EMG_STOP`|Emergency Stop State|
|`LST_HOMING`|Homing Operating State|

---

## **Sensor ID (LSS) Enumeration**

|Enumeration Value Description|Description|
|--|--|
|`LSS_FINGER_1_1`|Thumb Tip|
|`LSS_FINGER_1_2`|Thumb Pad|
|`LSS_FINGER_2_1`|Index Finger Tip|
|`LSS_FINGER_2_2`|Index Finger Pad|
|`LSS_FINGER_3_1`|Middle Finger Tip|
|`LSS_FINGER_3_2`|Middle Finger Pad|
|`LSS_FINGER_4_1`|Ring Finger Tip|
|`LSS_FINGER_4_2`|Ring Finger Pad|
|`LSS_FINGER_5_1`|Little Finger Tip|
|`LSS_FINGER_5_2`|Little Finger Pad|
|`LSS_HAND_PALM`|Palm Sensor|
|`LSS_MAX_COUNT`|Maximum Number of Sensors (for array allocation)|

---

## **Hand Orientation (LDR) Enumeration**

|Enumeration Value Description|Description|
|--|--|
|`LDR_HAND_RIGHT`|Right Hand (Default)|
|`LDR_HAND_LEFT`|Left Hand|

---

## **Function Pointer Types**

|Enumeration Value Description|Description|
|--|--|
|`LogAddCallback`|Log Callback Function：`void(*)(const char*)`|
|`ECSendDataCallback`|EtherCAT RPDO Transmission Callback：`bool(*)(const unsigned char*, unsigned int)`|
|`CANFDSendDataCallback`|CANFD Transmission Callback：`bool(*)(const unsigned char*, unsigned int)`|
|`RS485SendDataCallback`|RS485 Transmission Callback：`bool(*)(const unsigned char*, unsigned int)`|
|`CANSendDataCallback`|Standard CAN Transmission Callback：`bool(*)(const unsigned char*, unsigned int)`|

---

## **RS485SharedBus Class Interface**

`RS485SharedBus` represents one physical RS485 bus shared by multiple
`LHandProLib` instances. It does not open, close, or own the serial port. The
application still owns the serial-port lifetime and synchronous writes. The SDK
uses this object to provide:

- One priority scheduler for user commands, SDO, OTA, and background polling
- One global 30 ms transaction interval for the physical bus
- NodeID round-robin background polling
- Address framing, CRC, stream reassembly, and NodeID receive routing

Create exactly one `RS485SharedBus` for each physical serial port. The shared
bus object must outlive every `LHandProLib` instance bound to it.

#### `RS485SharedBus()`

Creates the shared scheduler and starts its internal bus worker.

#### `~RS485SharedBus()`

Stops the worker and releases its resources.

#### `void set_send_callback(RS485SendDataCallback callback)`

Sets the physical serial write callback. The callback receives an opaque frame
whose address and CRC have already been added by the SDK.

#### `void set_send_callback_ex(void* callback_impl)`

Sets the physical serial write callback as a
`std::function<bool(const unsigned char*, unsigned int)>`.

#### `int set_receive_data(const unsigned char* data_ptr, int data_size)`

Feeds an arbitrary raw byte chunk from the physical serial receive stream.
Split and joined frames are supported. Feed each physical receive chunk to the
shared bus exactly once; do not forward the same chunk to every hand instance.

#### `void close()`

Stops the shared worker and cancels queued requests. A closed shared bus cannot
be restarted.

---

## **LHandProLib Class Interface Description**

---

### **Constructors and Destructor**

#### `LHandProLib()`

Default Constructor: Initializes internal resources and private objects (per Pimpl idiom).

#### `~LHandProLib()`

Destructor: Releases resources and automatically invokes `close()`.

---

### **Initialization and Shutdown**

#### `int initial(int mode)`

Initializes the dexterous hand driver.

- **Parameter**：
  - `mode`：Communication type. Valid values `LCN_ECAT` / `LCN_CANFD` / `LCN_RS485`
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Preconditions**：
i. The underlying communication device is connected.
ii. Transmission callback is configured (`set_send_rpdo_callback`).
iii. Received data processing is configured (`set_tpdo_data_decode`).

---

#### `int initial(int mode, unsigned int node_id)`

Initializes the dexterous hand driver (with node ID specification).

- **Parameter**：
  - `mode`：Communication type. Valid values `LCN_CANFD` / `LCN_RS485`
  - `node_id`：CANFD or RS485 node ID
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Preconditions**：
  i. The underlying communication device is connected.
  ii. Transmission callback is configured.
  iii. Received data processing is configured.

---

#### `void close()`

Shuts down the driver, releases resources, and terminates all connections.

---

#### `void start_monitor()`

Starts motor/sensor status monitoring; initialization starts it automatically.
Shared RS485 mode does not create one polling thread per SDK instance. Instead,
each node is registered with the single `RS485SharedBus` worker. Monitoring can
be resumed manually after calling `stop_monitor()`.

---

#### `void stop_monitor()`

Stops the background monitor thread. Useful during OTA firmware upgrades or other scenarios requiring exclusive bus access to avoid polling interference. Call `initial()` or `start_monitor()` to resume polling.

---

#### `void set_dry_run_mode(bool enable)`

Sets dry-run mode. When enabled, command data is generated normally but not sent via callback. Useful for protocol debugging and unit testing without hardware.

- **Parameter**：
  - `enable`：`true` to enable dry-run, `false` to disable
- **Note**：
  - In dry-run mode, `initial()` skips SDO reads and will not timeout without hardware
  - In dry-run mode, `move_motors` skips homing status checks
  - All four communication modes (EtherCAT/CANFD/RS485/Standard CAN) are supported

---

#### `bool get_dry_run_mode()`

Returns the current dry-run mode state.

- **Return Value**：`true` if enabled, `false` if disabled

---

### **Communication Callback Configuration**

#### `void set_send_rpdo_callback(ECSendDataCallback callback)`

Configures EtherCAT RPDO transmission callback (for output data).

- **Parameter**：
  - `callback`：Function pointer with prototype `bool(const unsigned char* data, int size)`
    - `data`：RPDO data to be transmitted
    - `size`：Total data length
    - Return Value：`true` Successful，`false` Failed
- **Note**：Triggered by `get_pre_send_rpdo_data` (if configured).

---

#### `void set_send_rpdo_callback_ex(void* callback_impl)`

Configures EtherCAT transmission callback wrapped in `std::function`.

- **Parameter**：
  - `callback_impl`：Pointer to `std::function<bool(const unsigned char*, unsigned int)>`
- **Note**：Lifecycle management is the user's responsibility.

---

#### `void set_send_canfd_callback(CANFDSendDataCallback callback)`

Configures CANFD transmission callback (for output data).

- **Parameter**：
  - `callback`：Function pointer with prototype `bool(const unsigned char*, unsigned int)`
    - `data`：CANFD data to be transmitted
    - `size`：Total data length
    - Return Value：`true` Successful，`false` Failed
- **Note**：Triggered by `get_pre_send_canfd_data` (if configured).

---

#### `void set_send_canfd_callback_ex(void* callback_impl)`

Configures CANFD transmission callback wrapped in `std::function`.

- **Parameter**：
  - `callback_impl`：Pointer to `std::function<bool(const unsigned char*, unsigned int)>`
- **Note**：Lifecycle management is the user's responsibility.

---

#### `int get_pre_send_rpdo_data(unsigned char* data_ptr, int* io_size)`

Retrieves RPDO data to be transmitted (for EtherCAT output area).

- **Parameter**：
  - `data_ptr`：Output buffer pointer
    - `nullptr` to only get required buffer size
  - `io_size`：Input - Buffer capacity; Output - Actually required or written byte count
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：Automatically triggers transmission if `set_send_rpdo_callback` is configured.

---

#### `int get_pre_send_canfd_data(unsigned char* data_ptr, int* io_size)`

Retrieves CANFD data to be transmitted (for CANFD output area).

- **Parameter**：
  - `data_ptr`：Output buffer pointer
    - `nullptr` to only get required buffer size
  - `io_size`：Input - Buffer capacity; Output - Actually required or written byte count
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_tpdo_data_decode(const unsigned char* data_ptr, int data_size)`

Decodes TPDO data received from EtherCAT (input data).

- **Parameter**：
  - `data_ptr`：TPDO data pointer
  - `data_size`：Total data length (bytes)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：Latest status can be retrieved via interfaces like `get_now_angle` after decoding.

---

#### `int set_canfd_data_decode(const unsigned char* data_ptr, int data_size)`

Decodes data received from CANFD (input data).

- **Parameter**：
  - `data_ptr`：CANFD data pointer
  - `data_size`：Total data length (bytes)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：Latest status can be retrieved via interfaces like `get_now_angle` after decoding.

---

#### `void set_send_rs485_callback(RS485SendDataCallback callback)`

Configures RS485 transmission callback (for output data).

- **Parameter**：
  - `callback`：Function pointer with prototype `bool(const unsigned char* data, unsigned int size)`
    - `data`：RS485 data to be transmitted
    - `size`：Total data length
    - Return Value：`true` Successful，`false` Failed
- **Note**：
  - This API remains available for the legacy implicit single-device bus mode.
  - For multiple devices on one physical bus, set the callback on
    `RS485SharedBus`.
  - `get_pre_send_rs485_data` triggers transmission when configured.

---

#### `void set_send_rs485_callback_ex(void* callback_impl)`

Configures RS485 transmission callback wrapped in `std::function`.

- **Parameter**：
  - `callback_impl`：Pointer to `std::function<bool(const unsigned char*, unsigned int)>`
- **Note**：Lifecycle management is the user's responsibility.

---

#### `void set_rs485_shared_bus(RS485SharedBus* bus)`

Binds this SDK instance to a shared RS485 bus. Call it before
`initial(LCN_RS485, node_id)`.

- **Parameter**：
  - `bus`：Shared bus pointer. Pass `nullptr` to restore the legacy implicit
    single-device bus mode.
- **Constraints**：
  - NodeIDs must be unique within one shared bus.
  - The shared bus must outlive all bound hand instances.
  - Feed physical serial receive data only through
    `RS485SharedBus::set_receive_data`.

Two-node example:

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

The existing `initial` API remains unchanged. User requests take priority over
background polls, polling rotates between NodeID 1 and 2, and all sends share
one 30 ms transaction interval.

---

#### `int get_pre_send_rs485_data(unsigned char* data_ptr, int* io_size)`

Retrieves RS485 complete framed data to be transmitted (including address header + CRC).

- **Parameter**：
  - `data_ptr`：Output buffer pointer
    - `nullptr` to only get required buffer size
  - `io_size`：Input - Buffer capacity; Output - Actually required or written byte count
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Data Format**：`[Address Header 2B] + [Payload 64B] + [CRC 2B]` = 68 bytes
- **Note**：Automatically triggers transmission if `set_send_rs485_callback` is configured.

---

#### `int set_rs485_data_decode(const unsigned char* data_ptr, int data_size)`

Decodes data received from RS485 (input data).

- **Parameter**：
  - `data_ptr`：RS485 data pointer
  - `data_size`：Total data length (bytes)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：Latest status can be retrieved via interfaces like `get_now_angle` after decoding.
- **Shared-bus note**：The legacy single-device mode continues to use this
  method. In shared-bus mode, call `RS485SharedBus::set_receive_data` once for
  each physical serial receive chunk.

---

#### `void set_send_can_callback(CANSendDataCallback callback)`

Configures Standard CAN transmission callback (for LCN_CAN mode).

- **Parameter**：
  - `callback`：Function pointer with prototype `bool(const unsigned char* data, unsigned int size)`
    - `data`：CAN data to be transmitted
    - `size`：Total data length
    - Return Value：`true` Successful，`false` Failed

---

#### `void set_send_can_callback_ex(void* callback_impl)`

Configures Standard CAN transmission callback wrapped in `std::function`.

- **Parameter**：
  - `callback_impl`：Pointer to `std::function<bool(const unsigned char*, unsigned int)>`
- **Note**：Lifecycle management is the user's responsibility.

---

#### `int set_can_data_decode(unsigned int id, const unsigned char* data_ptr, int data_size)`

Decodes incoming standard CAN data (called by user in LCN_CAN mode).

- **Parameter**：
  - `id`：CAN frame ID
  - `data_ptr`：CAN data pointer
  - `data_size`：Total data length (bytes)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：Latest status can be retrieved via interfaces like `get_now_angle` after decoding.

---

#### `int get_pre_send_can_data(unsigned char* data_ptr, int* io_size)`

Retrieves standard CAN data in pre-send area (dry-run capture). Each frame is 11 bytes: `[CAN_ID little-endian 4B] + [data 7B]`, multiple frames concatenated. Parameters same as `get_pre_send_rpdo_data`.

---

### **Hand Type and Orientation Configuration**

#### `int set_hand_type(int hand_type)`

Configures dexterous hand type.

- **Parameter**：
  - `hand_type`：DOF type. Valid values:  `LAC_DOF_6` `LAC_DOF_6_S` `LAC_DOF_16`
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Default Value**：`LAC_DOF_6`

---

#### `int get_dof(int* total, int* active)`

Retrieves current dexterous hand DOF information.

- **Parameter**：
  - `total`：Output parameter, returns total joint count (including passive joints)
  - `active`：Output parameter, returns active motor count
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_hand_type(int* hand_type)`

Retrieves current dexterous hand type.

- **Parameter**：
  - `hand_type`：Output parameter, receives hand type value
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_hand_direction(int dir)`

Configures hand orientation (affects motion mapping logic).

- **Parameter**：
  - `dir`：Orientation. Valid values: `LDR_HAND_RIGHT` `LDR_HAND_LEFT`
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_hand_direction(int* dir)`

Retrieves current hand orientation.

- **Parameter**：
  - `dir`：Output parameter, receives orientation value: `LDR_HAND_RIGHT` `LDR_HAND_LEFT`
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

### **Motor Control Interfaces**

#### `int set_control_mode(int id, int mode = LCM_POSITION)`

Configures motor control mode.

- **Parameter**：
  - `id`：Motor ID. Valid range: `[0, DOF]`
    - `id = 0`：Broadcast mode (configure all motors) 
    - `id ≥ 1`：Configure specified motor
  - `mode`：Control mode. Refer to `LCM_*`
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_control_mode(int id, int* mode)`

Gets the control mode of the specified motor.

- **Parameters**：
  - `id`：Motor ID. Value range: `[1, DOF]`
  - mode: Output parameter. Receives the current control mode
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_safe_current_enable(int enable)`

Sets the safe current enable state

- **Parameters**：
  - `enable`：Safe current enable state. `1` enables, `0` disables
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_safe_current_enable(int* enable)`

Gets the safe current enable state

- **Parameters**：
  - `enable`：Output parameter. Receives the current safe current enable state. `1` enabled, `0` disabled
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_home_current(int current)`

Sets the homing current

- **Parameters**：
  - `current`：Homing current value, unit `%` (percentage). For example, `100` means 100%
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_home_current(int* current)`

Gets the homing current

- **Parameters**：
  - `current`：Output parameter. Receives the current homing current value, unit `%` (percentage)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_can_node_id(int node_id)`

Set CAN node ID (written to drive parameters via SDO).

- **Parameters**：
  - `node_id`：Node ID
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  sdk_handle->set_can_node_id(1);
  ```

---

#### `int get_can_node_id(int* node_id)`

Get CAN node ID.

- **Parameters**：
  - `node_id`：Output parameter, receives the node ID
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  int node_id = 0;
  sdk_handle->get_can_node_id(&node_id);
  ```

---

#### `int set_canfd_arb_baudrate(int baudrate)`

Set CANFD arbitration segment baud rate (written to drive parameters via SDO).

- **Parameters**：
  - `baudrate`：Baud rate value (e.g., 1000000)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  sdk_handle->set_canfd_arb_baudrate(1000000);
  ```

---

#### `int get_canfd_arb_baudrate(int* baudrate)`

Get CANFD arbitration segment baud rate.

- **Parameters**：
  - `baudrate`：Output parameter, receives the baud rate value
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  int baudrate = 0;
  sdk_handle->get_canfd_arb_baudrate(&baudrate);
  ```

---

#### `int set_canfd_data_baudrate(int baudrate)`

Set CANFD data segment baud rate (written to drive parameters via SDO).

- **Parameters**：
  - `baudrate`：Baud rate value (e.g., 5000000)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  sdk_handle->set_canfd_data_baudrate(5000000);
  ```

---

#### `int get_canfd_data_baudrate(int* baudrate)`

Get CANFD data segment baud rate.

- **Parameters**：
  - `baudrate`：Output parameter, receives the baud rate value
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  int baudrate = 0;
  sdk_handle->get_canfd_data_baudrate(&baudrate);
  ```

---

#### `int set_rs485_node_id(int node_id)`

Set RS485 node ID (written to drive parameters via SDO).

- **Parameters**：
  - `node_id`：Node ID
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  sdk_handle->set_rs485_node_id(1);
  ```

---

#### `int get_rs485_node_id(int* node_id)`

Get RS485 node ID.

- **Parameters**：
  - `node_id`：Output parameter, receives the node ID
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  int node_id = 0;
  sdk_handle->get_rs485_node_id(&node_id);
  ```

---

#### `int set_rs485_baudrate(int baudrate)`

Set RS485 baud rate (written to drive parameters via SDO).

- **Parameters**：
  - `baudrate`：Baud rate value (e.g., 500000)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  sdk_handle->set_rs485_baudrate(500000);
  ```

---

#### `int get_rs485_baudrate(int* baudrate)`

Get RS485 baud rate.

- **Parameters**：
  - `baudrate`：Output parameter, receives the baud rate value
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  int baudrate = 0;
  sdk_handle->get_rs485_baudrate(&baudrate);
  ```

---

#### `int set_enable(int id, int enable)`

Enables or disables the motor driver

- **Parameters**：
  - `id`：Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Broadcast mode (applies to all motors)
    - `id ≥ 1`：Applies to the specified motor
  - `enable`： Enable flag，`1` Enable，`0` Disable
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_enable(int id, int* enable)`

Gets the current enable status of the motor

- **Parameters**：
  - `id`：Motor ID. Value range: `[1, DOF]`
  - `enable`：Output parameter，`1`  Enabled，`0` Disabled
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_position_reached(int id, int* reached)`

Gets whether the motor has reached the target position

- **Parameters**：
  - `id`：Motor ID. Value range:  `[1, DOF]`
  - `reached`：Output parameter，`1` Position reached，`0` In motion
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_torque_reached(int id, int* reached)`

Gets whether the motor has reached the target torque

- **Parameters**：
  - `id`：Motor ID. Value range:`[1, DOF]`
  - `reached`：Output parameter，`1` Torque reached，`0` Torque not reached
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_clear_alarm(int id = 0)`

Clear motor alarm status

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Broadcast to clear all motor alarms
    - `id ≥ 1`：Clear alarm for the specified motor
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_now_alarm(int id, int* alarm)`

Get the motor's current alarm status code

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `alarm`：Output parameter，receive alarm codes (refer to the hardware manual for specific interpretations)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int home_motors(int id)`

Initiate motor homing

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Initiate homing for all motors
    - `id ≥ 1`：Initiate homing for the specified motor
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_move_no_home(int enable = 0)`

Configure whether to allow movement without homing

- **Parameters**：
  - `enable`：Enable flag
    - `0`：Disallow movement without homing (default)
    - `1`：Allow movement without homing
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int play_gesture(int gesture_id, int velocity, int current)`

Execute a predefined gesture

- **Parameters**：
  - `gesture_id`：Gesture ID to execute
  - `velocity`：Target velocity (unit: counts/second)
  - `current`：Maximum current (unit: ‰, per mille)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  // Execute gesture 1, target velocity 20000 counts/second, maximum current 1000‰
  sdk_handle->play_gesture(1, 20000, 1000);
  ```

---

#### `int get_limit_target_angle(int id, float* min_angle, float* max_angle)`

Get the upper and lower limits of the motor's target angle

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `min_angle`：Output parameter，Minimum allowable angle (Unit: °)
  - `max_angle`：Output parameter，Maximum allowable angle (Unit: °)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_target_angle(int id, float angle)`

Set the motor's target angle

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Broadcast mode (applies to all motors)
    - `id ≥ 1`：Applies to the specified motor
  - `angle`：arget angle unit: ° (degrees)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：The actual achievable range is determined by the mechanical structure.

---

#### `int get_target_angle(int id, float* angle)`

Get the last set target angle

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `angle`：Output parameter，Receives the target angle value (Unit: °)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_target_position(int id, int position)`

Set the motor's target position (stroke equivalent)

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Broadcast mode (applies to all motors)
    - `id ≥ 1`：Applies to the specified motor
  - `position`：arget position unit: equivalent，Value range `[0, 10000]`
    - `0` Indicates the start position
    - `10000` Indicates the full stroke
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：The actual stroke is determined by the mechanical structure and calibration

---

#### `int get_target_position(int id, int* position)`

Get the last set target position (stroke equivalent)

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `position`：Output parameter，Receives the target position value (Unit: equivalent; Value range: `[0, 10000]`）
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_velocity(int id, float velocity)`

Set the motor's target speed (default unit: angular velocity (°/s))

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Broadcast mode (applies to all motors)
    - `id ≥ 1`：Applies to the specified motor
  - `velocity`：Target speed unit: °/s
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：The actual speed is subject to hardware limitations; it is recommended to use `set_angular_velocity`for clear semantics

---

#### `int get_velocity(int id, float* velocity)`

Get the last set target speed (returns angular velocity by default)

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `velocity`：Output parameter，Receives the target speed value (Unit: °/s)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：It is recommended to use `get_angular_velocity`  for clear semantics

---

#### `int set_angular_velocity(int id, float velocity)`

Set the motor's target angular velocity (Unit: °/s)

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Broadcast mode (applies to all motors)
    - `id ≥ 1`：Applies to the specified motor
  - `velocity`：Target angular velocity unit: °/s
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：This interface is consistent in functionality with `set_velocity` but more clearly named; it is recommended for priority use

---

#### `int get_angular_velocity(int id, float* velocity)`

Get the last set target speed(Unit: °/s)

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `velocity`：Output parameter，Receives the target angular velocity value (Unit: °/s)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_position_velocity(int id, int velocity)`

Set the motor's target speed (stroke equivalent speed)

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Broadcast mode (applies to all motors)
    - `id ≥ 1`：Applies to the specified motor
  - `velocity`：Target speed unit: equivalent/s (i.e., equivalents per second)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_position_velocity(int id, int* velocity)`

Get the last set target speed(i.e., equivalents per second)

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `velocity`：Output parameter，Receives the target speed value (Unit: equivalent/s)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_max_current(int id, int current)`

Set the motor's maximum allowable current

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Broadcast mode (applies to all motors)
    - `id ≥ 1`：Applies to the specified motor
  - `current`：Maximum current limit (Unit: ‰)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：Used for overcurrent protection and as the upper limit for torque control

---

#### `int get_max_current(int id, int* current)`

Get the last set maximum current value

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `current`：Output parameter，Receives the maximum current value (Unit: ‰)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int move_motors(int id = 0)`

Drive the motor to execute the set motion command

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]`
    - `id = 0`：Start motion for all motors
    - `id ≥ 1`：Start motion for the specified motor
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：The target value must be set first (e.g., via `set_target_angle`)

---

#### `int stop_motors(int id = 0)`

Stop motor movement.

- **Parameters**：
  - `id`： Motor ID. Value range: `[0, DOF]` (`0` for broadcast), otherwise controls specific ID
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

### **Status and Feedback Interfaces**

#### `int get_now_status(int id, int* status)`

Get the motor's current operation status

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `status`：Output parameter，Receives the operation status code (refer to the`LST_*`enumeration)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_firmware_version(float* version)`

Get the dexterous hand firmware version

- **Parameters**：
  - `version`：Output parameter，Receives the firmware version number (e.g., 0.32 means version 0.32)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  float ver;
  sdk_handle->get_firmware_version(&ver);
  ```

---

#### `int get_serial_number(char* sn, int size)`

Get the dexterous hand serial number (read via SDO during initialization).

- **Parameters**：
  - `sn`：Output buffer pointer, receives the serial number string
  - `size`：Buffer size
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  char buf[64];
  sdk_handle->get_serial_number(buf, sizeof(buf));
  std::string sn(buf);
  ```

---

#### `int get_now_angle(int id, float* angle)`

Get the motor's current angular position

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `angle`：Output parameter，Receives the current angle (Unit: °)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_now_position(int id, int* position)`

Get the motor's current stroke position (equivalent)

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `position`：Output parameter，Receives the current stroke position (Unit: equivalent; Value range: `[0, 10000]`）
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_now_velocity(int id, float* velocity)`

Get the motor's current actual speed

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `velocity`：Output parameter，Receives the current speed (Unit: °/s)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Note**：This interface returns angular velocity by default; it is recommended to use `get_now_angular_velocity` for clear semantics

---

#### `int get_now_angular_velocity(int id, float* velocity)`

Get the motor's current actual angular velocity (Unit: °/s)

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `velocity`：Output parameter，Receives the current angular velocity (Unit: °/s)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_now_position_velocity(int id, int* velocity)`

Get the motor's current actual speed (stroke equivalent speed)

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `velocity`：Output parameter，Receives the current speed (Unit: equivalent/s)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_now_current(int id, int* current)`

Get the motor's actual current

- **Parameters**：
  - `id`： Motor ID. Value range: `[1, DOF]`
  - `current`：Output parameter，Receives the actual current value (Unit: ‰)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int sync_position_to_target(int id = 0)`

Sync current position to target position.

- **Parameters**:
  - `id`: Motor ID, range `[0, DOF]` (`0` for broadcast), otherwise controls specific ID
- **Return Value**:
  - `0`: Execution successful
  - Others: Refer to `LER_*` error codes
- **Example**:
  ```c_cpp
  // Sync current position of motor 1 to target position
  sdk_handle->sync_position_to_target(1);
  // Sync current position of all motors to target position
  sdk_handle->sync_position_to_target(0);
  ```

---

#### `int set_sensor_enable(int enable)`

Enable or disable sensor monitoring.

- **Parameters**：
  - `enable`：`1` enable, `0` disable
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_sensor_data_format(int format)`

Set sensor data format.

- **Parameters**:
  - `format`: Format type, default is 0
- **Return Value**:
  - `0`: Execution successful
  - Others: Refer to `LER_*` error codes
- **Example**:
  ```c_cpp
  sdk_handle->set_sensor_data_format(0);
  ```

---

#### `int set_sensor_order(const int* order, int size)`

Set sensor mapping order (in the order of thumb, index finger, middle finger, ring finger, little finger).

- **Parameters**:
  - `order`: Pointer to integer array
  - `size`: Array size, range [1, LSS_MAX_COUNT]
- **Return Value**:
  - `0`: Execution successful
  - Others: Refer to `LER_*` error codes
- **Example**:
  ```c_cpp
  int order[6] = {LSS_FINGER_1_1, LSS_FINGER_2_1, LSS_FINGER_3_1,
                  LSS_FINGER_4_1, LSS_FINGER_5_1, LSS_HAND_PALM};
  sdk_handle->set_sensor_order(order, 6);
  ```

---

### **Haptic Sensor Interfaces**

#### `int get_finger_sensor_pos(int sensor_id, float* x_values, float* y_values, int* io_count)`

Get the haptic sensor's distribution coordinates (normalized `[0.0, 1.0]`）。

- **Parameters**：
  - `sensor_id`：Sensor ID，Value references the  `LSS_*` enumeration
  - `x_values`：Output parameter，X-coordinate array pointer (can be  `nullptr` to retrieve only the count)
  - `y_values`：Output parameter，Y-coordinate array pointer
  - `io_count`：Input: Indicates buffer size; Output: Returns the actual number of elements
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_finger_pressure(int sensor_id, float* pressure_list, int* io_count)`

Get the sensor pressure value (Value range: `[0.0, 1.0]`）。

- **Parameters**：
  - `sensor_id`：Sensor ID
  - `pressure_list`：Output parameter，Pressure value array pointer (can be`nullptr`to retrieve only the count)
  - `io_count`：Input: Buffer size; Output: Actual number of entries
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int set_finger_pressure_reset()`

Reset the sensor reference value (with the difference calculated from this moment)

- **Description**：
  - Sets the reference tactile sensor current value, i.e., resets all sensor current states to 0 state
  - The sensor difference returned by `get_finger_pressure` is based on this reference
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```cpp
  sdk_handle->set_finger_pressure_reset();
  ```

---

#### `int get_finger_normal_force(int sensor_id, float* normal_force)`

Get the normal force (perpendicular to the surface)

- **Parameters**：
  - `sensor_id`：Sensor ID
  - `normal_force`：Output parameter，Receives the normal force
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_finger_normal_force_ex(int sensor_id, float* normal_force_list, int* io_count)`

Get the tactile sensor normal force array at specified location.

- **Parameters**：
  - `sensor_id`：Sensor ID, refer to `LSS_*` enumeration
  - `normal_force_list`：Output parameter, normal force array pointer (range [0.0-1.0])
  - `io_count`：Output parameter, returned array length
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```c_cpp
  int count = 0;
  sdk_handle->get_finger_normal_force_ex(sensor_id, nullptr, &count);
  std::vector<float> normal_force(count);
  sdk_handle->get_finger_normal_force_ex(sensor_id, normal_force.data(), &count);
  ```

---

#### `int get_finger_tangential_force(int sensor_id, float* tangential_force)`

Get the tangential force (parallel to the surface)

- **Parameters**：
  - `sensor_id`：Sensor ID
  - `tangential_force`：Output parameter，Receives the tangential force
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_finger_tangential_force_ex(int sensor_id, float* tangential_force_list, int* io_count)`

Get the tactile sensor tangential force array at specified location.

- **Parameters**：
  - `sensor_id`：Sensor ID, refer to `LSS_*` enumeration
  - `tangential_force_list`：Output parameter, tangential force array pointer (range [0.0-1.0])
  - `io_count`：Output parameter, returned array length
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```c_cpp
  int count = 0;
  sdk_handle->get_finger_tangential_force_ex(sensor_id, nullptr, &count);
  std::vector<float> tangential_force(count);
  sdk_handle->get_finger_tangential_force_ex(sensor_id, tangential_force.data(), &count);
  ```

---

#### `int get_finger_force_direction(int sensor_id, float* force_direction)`

Get the force direction

- **Parameters**：
  - `sensor_id`：Sensor ID
  - `force_direction`：Output parameter，Receives the direction angle (Value range `[0, 360]`，Unit: °)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_finger_force_direction_ex(int sensor_id, float* force_direction_list, int* io_count)`

Get the tactile sensor force direction array at specified location.

- **Parameters**：
  - `sensor_id`：Sensor ID, refer to `LSS_*` enumeration
  - `force_direction_list`：Output parameter, force direction array pointer (range [0-360])
  - `io_count`：Output parameter, returned array length
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```c_cpp
  int count = 0;
  sdk_handle->get_finger_force_direction_ex(sensor_id, nullptr, &count);
  std::vector<float> force_direction(count);
  sdk_handle->get_finger_force_direction_ex(sensor_id, force_direction.data(), &count);
  ```

---

#### `int get_finger_proximity(int sensor_id, float* proximity)`

Get the proximity level (e.g., for proximity sensors)

- **Parameters**：
  - `sensor_id`：Sensor ID
  - `proximity`：Output parameter，Receives the proximity level (Value range: `[0.0, 1.0]`)
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_finger_proximity_ex(int sensor_id, float* proximity_list, int* io_count)`

Get the tactile sensor proximity array at specified location.

- **Parameters**：
  - `sensor_id`：Sensor ID, refer to `LSS_*` enumeration
  - `proximity_list`：Output parameter, proximity array pointer
  - `io_count`：Output parameter, returned array length
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes
- **Example**：
  ```c_cpp
  int count = 0;
  sdk_handle->get_finger_proximity_ex(sensor_id, nullptr, &count);
  std::vector<float> proximity(count);
  sdk_handle->get_finger_proximity_ex(sensor_id, proximity.data(), &count);
  ```

---

### **SDO Parameter Read/Write Interfaces**

#### `int set_sdo_drive_param(unsigned int index, unsigned char subindex, unsigned int value)`

Set drive parameters via SDO.

- **Parameters**：
  - `index`：Parameter index
  - `subindex`：Parameter subindex
  - `value`：Parameter value
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int get_sdo_drive_param(unsigned int index, unsigned char subindex, unsigned int* value)`

Get drive parameters via SDO.

- **Parameters**：
  - `index`：Parameter index
  - `subindex`：Parameter subindex
  - `value`：Output parameter, receives the parameter value
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int save_sdo_drive_param()`

Save SDO drive parameters (sends save command, compatible with both old and new firmware versions).

- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

### **Log System**

#### `void log_on(bool on = false, int maxsize = 10000)`

Enable or disable log printing functionality

- **Parameters**：
  - `on`：`true` Enable logging，`false` Disable logging (default)
  - `maxsize`：Maximum cached log entries: Prevents memory overflow

---

#### `int log_send(int* cmd, int count)`

Set the filter list for transmit command addresses to be printed

- **Parameters**：
  - `cmd`：Transmit command address array pointer
    - `nullptr` Print all transmitted data
  - `count`：Number of array elements
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `int log_recv(int* cmd, int count)`

Set the filter list of receiving instruction addresses that need to be printed

- **Parameters**：
  - `cmd`：ransmit command address array pointe
    - `nullptr` Print all transmitted data
  - `count`：Number of array elements
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `void log_reset(bool send = true, bool recv = true)`

Set the filter list for receive command addresses to be printed

- **Parameters**：
  - `send`：Whether to clear the transmit filter list
  - `recv`： Whether to clear the receive filter list

---

#### `int log_save(const char* file_name)`

Save the current logs to a file

- **Parameters**：
  - `file_name`：File path
- **Return Value**：
  - `0`：Execution successful
  - Others：Refer to `LER_*` error codes

---

#### `void log_clear()`

Clear the log records in the current memory.

---

#### `void set_log_callback(LogAddCallback callback)`

Set a log callback function (to receive log strings in real-time)

- **Parameters**：
- `callback`: Function pointer with the prototype `void(const char*)`
  - The input parameter is a log string (ending with `\n`)
- **Example**：
  ```c_cpp
  sdk_handle->set_log_callback([](const char* msg) {
      printf("[SDK_LOG] %s", msg);
  });
  ```

---

## **Example of Usage Process (EtherCAT Communication)**

```c_cpp
#include "LHandProLib.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <functional>
#include "EthercatMaster.h" // EtherCAT Main Station Library

int main() {
  /****** Initialization ******/
  // Initialize LHandProLib object
  auto sdk_handle = std::make_shared<lhplib::LHandProLib>();
  // Initialize EtherCAT master object
  auto ec_master_ = std::make_shared<EthercatMaster>();
  // Initialize EtherCAT master
 std::vector<std::string> names = ec_master_->scanNetworkInterfaces();
  int channel_index = 0; // Select the correct channel according to the scanned network interface list
  if (!ec_master_->init(channel_index)) {
    std::cout << "Initialization failed" << std::endl;
    return -1;
  }
  std::cout << "Connection Success" << std::endl;
  // EtherCAT master enters OP state
  if (!ec_master_->start()) {
    std::cout << "Failed to start device" << std::endl;
    return -1;
  }

  int slave_id = 1; // Select the target slave ID from getSlaveInfoList()
  const auto slave_list = ec_master_->getSlaveInfoList();
  for (const auto& slave : slave_list) {
    std::cout << "Slave " << slave.index << ": " << slave.name << std::endl;
  }
  // EtherCAT master starts background data exchange communication (non-blocking)
  ec_master_->run();
  
  /****** Setting up RPDO sending callback and TPDO processing ******/
  // EtherCAT sending function
  auto send_func = [ec_master_, slave_id](const unsigned char* data, unsigned int size) {
    return ec_master_->setSlaveOutputs(slave_id, data, size);
  };
  // Wrapping with std::function
  std::function<bool(const unsigned char*, unsigned int)> func = send_func;
  
  // Setting EtherCAT data sending callback
  sdk_handle->set_send_rpdo_callback_ex(&func);
  
  // Creating a monitoring thread to parse TPDO data through the interface
  std::atomic<bool> stop{false};
  std::thread t_monitor([=, &stop]() {
    while (!stop) {
      // Continuously obtaining TPDO data and passing it to the SDK for parsing
      int input_size = ec_master_->getSlaveInputSize(slave_id);
      std::vector<uint8_t> in_buffer(input_size);
      if (ec_master_->getSlaveInputs(slave_id, in_buffer.data(), input_size)) {
        sdk_handle->set_tpdo_data_decode(in_buffer.data(), input_size);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // Initializing the library
  if (sdk_handle->initial(lhplib::LCN_ECAT) != lhplib::LER_NONE) {
    std::cerr << "LHandProLib initialization failed!" << std::endl;
    return -1;
  }


  /****** Motion interface calls ******/
  // Set control mode
  sdk_handle->set_control_mode(0, lhplib::LCM_POSITION);
  // Enable all
  sdk_handle->set_enable(0, 1);


  // Obtain the current degree of freedom
  int dof_total = 0, dof_active = 0;
  sdk_handle->get_dof(&dof_total, &dof_active);
  
  // Target angle for each joint
  std::vector<float> target_angles = {30.0f, 10.0f, 60.0f, 60.0f, 60.0f, 60.0f};
  
  // Set motion parameters
  for (int i = 0; i < dof_active; ++i) {
    sdk_handle->set_target_angle(i + 1, target_angles[i]);  // Set motion parameters
    sdk_handle->set_angular_velocity(i + 1, 200);           // Set speed (200 °/s)
    sdk_handle->set_max_current(i + 1, 1000);                // Set maximum current limit (1000‰)
  }
  // Drive all motors to move
  sdk_handle->move_motors();                       
  // Waiting for the end of the exercise
  std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 

  // Obtain the current angle
  for (int i = 0; i < dof_active; ++i) {
    float angle = 0;
    if (sdk_handle->get_now_angle(i + 1, &angle) == lhplib::LER_NONE) {
      std::cout << "Current joint: " << i + 1 << " Angle：" << angle << "°" << std::endl;
    }
  }

  /****** Sensor interface calls ******/
  // Read the pressure distribution at the tip of the thumb
  int count = 0;
  sdk_handle->get_finger_pressure(lhplib::LSS_FINGER_1_1, nullptr, &count);
  std::vector<float> pressures(count);
  sdk_handle->get_finger_pressure(lhplib::LSS_FINGER_1_1, pressures.data(), &count);

  for (int i = 0; i < count; ++i) {
      std::cout << "Sensor: " << i << " pressure: " << pressures[i] << std::endl;
  }
  
  // Read normal force
  float normal_force;
  sdk_handle->get_finger_normal_force(lhplib::LSS_FINGER_1_1, &normal_force);
  std::cout << "Sensor: " << lhplib::LSS_FINGER_1_1 << " Normal force: " << normal_force << std::endl;
  
  // Read tangential force
  float tangential_force;
  sdk_handle->get_finger_tangential_force(lhplib::LSS_FINGER_1_1, &tangential_force);
  std::cout << "Sensor: " << lhplib::LSS_FINGER_1_1 << " Tangential force: " << tangential_force << std::endl;
  
  // Read the direction of tangential force
  float force_direction;
  sdk_handle->get_finger_force_direction(lhplib::LSS_FINGER_1_1, &force_direction);
  std::cout << "Sensor: " << lhplib::LSS_FINGER_1_1 << " direction: " << force_direction << std::endl;
  
  // Read proximity
  float proximity;
  sdk_handle->get_finger_proximity(lhplib::LSS_FINGER_1_1, &proximity);
  std::cout << "Sensor: " << lhplib::LSS_FINGER_1_1 << " proximity: " << proximity << std::endl;

  // Close
  sdk_handle->close();

  return 0;
}
```

## **Other examples**

Please refer to the example projects for each communication version in the SDK development kit.

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

## **Frequently Asked Questions**

### Q: Why does `initial()` return `LER_KEY_FUNC_UNINIT`?

A: The send callback is not set. For EtherCAT, set `set_send_rpdo_callback`. For CANFD, set `set_send_canfd_callback`.

### Q: How to view communication data?

A: Use `log_on(true)` and call `log_send(nullptr, 0)` and `log_recv(nullptr, 0)` to print all data.

### Q: Sensor data is not updating?

A: Ensure that `set_tpdo_data_decode` (EtherCAT) is called periodically.

### Q: Why should two RS485 devices on one serial port not run independent polling loops?

A: Independent loops do not share transmission timing and will contend for the
same half-duplex bus. Create one `RS485SharedBus`, bind the two SDK instances
with unique NodeIDs, and feed the physical serial receive stream to the shared
bus only once.

---

## **Version Update History**

- **v1.0**: Initial version, supporting EtherCAT, 6/15 DOF, tactile sensor reading, and logging system.
- **v1.1**: Added equivalent control interface and optimized the EtherCAT C++ sample program.
- **v1.2**: Provided C wrapper header files and EtherCAT Python sample programs, optimized some implementation logic, and enhanced stability.
- **v1.3**: Added CANFD communication support.
- **v1.4**: Improved documentation and fixed known issues.
- **v1.5**: Improved sensor interface.
- **v1.6**: Added SN code reading, CANFD baud rate / CAN node ID configuration, SDO parameter read/write interfaces; updated sensor mapping examples.
- **v1.7**: Added `RS485SharedBus` for multiple SDK instances sharing one physical RS485 bus, plus a two-device NodeID 1 and 2 sample.

---

*Document version: v1.7 | Update date: July 24, 2026*
