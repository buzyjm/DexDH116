#ifndef LHANDPROLIB_H
#define LHANDPROLIB_H

#ifdef __cplusplus
extern "C" {
#endif

/// @brief 定义导出/导入符号 / Define export/import symbols
#if defined(_WIN32) || defined(_WIN64)
#ifdef SDK_EXPORTS
#define SDK_API __declspec(dllexport)
#else
#define SDK_API __declspec(dllimport)
#endif
#else
#define SDK_API __attribute__((visibility("default")))
#endif

#ifndef SDK_C_API_PREFIX
#define SDK_C_API_PREFIX lhandprolib
#endif

#define SDK_JOIN_IMPL(a, b) a##b
#define SDK_JOIN(a, b) SDK_JOIN_IMPL(a, b)
#define SDK_C_API(name) SDK_JOIN(SDK_C_API_PREFIX, name)

#include <stdbool.h>
/// @brief 错误码 / Error codes
#define C_LER_NONE 0                ///< 执行成功 / Execution successful
#define C_LER_PARAMETER 1           ///< 参数错误 / Parameter error
#define C_LER_KEY_FUNC_UNINIT 2     ///< 关键函数未初始化 / Key function not initialized
#define C_LER_GET_CONFIGURATION 3   ///< 读取配置失败 / Failed to read configuration
#define C_LER_DATA_ANOMALY 4        ///< 数据异常 / Data anomaly
#define C_LER_COMM_CONNECT 5        ///< 通讯连接错误 / Communication connection error
#define C_LER_COMM_SEND 6           ///< 通讯发送错误 / Communication send error
#define C_LER_COMM_RECV 7           ///< 通讯接收错误 / Communication receive error
#define C_LER_COMM_DATA_FORMAT 8    ///< 通讯数据格式错误 / Communication data format error
#define C_LER_INVALID_PATH 9        ///< 无效的文件路径 / Invalid file path
#define C_LER_LOG_SAVE_FAIL 10      ///< 日志文件保存失败 / Log file save failed
#define C_LER_NOT_HOME 11           ///< 没回零错误 / Not homed error
#define C_LER_UNKNOWN 999           ///< 未知错误 / Unknown error

/// @brief 自由度枚举 / Degrees of freedom enum
#define C_LAC_DOF_6 0    ///< 6自由度 / 6 degrees of freedom
#define C_LAC_DOF_6_S 1  ///< 6自由度(S版本) / 6 degrees of freedom (S version)
#define C_LAC_DOF_16 2   ///< 16自由度 / 16 degrees of freedom

/// @brief 通讯类型枚举 / Communication type enum
#define C_LCN_ECAT 0   ///< EtherCAT
#define C_LCN_CANFD 1  ///< CANFD
#define C_LCN_RS485 2  ///< RS485
#define C_LCN_CAN 3    ///< 标准CAN / Standard CAN

/// @brief 灵巧手方向枚举 / Hand direction enum
#define C_LDR_HAND_RIGHT 0  ///< 右手 / Right hand
#define C_LDR_HAND_LEFT 1   ///< 左手 / Left hand

/// @brief 控制模式枚举 / Control mode enum
#define C_LCM_POSITION 0  ///< 位置控制 / Position control
#define C_LCM_VELOCITY 1  ///< 速度控制 / Velocity control
#define C_LCM_TORQUE 2    ///< 力矩控制 / Torque control
#define C_LCM_VEL_TOR 3   ///< 速度+力矩混合控制 / Velocity + torque hybrid control
#define C_LCM_POS_TOR 4   ///< 位置+力矩混合控制 / Position + torque hybrid control
#define C_LCM_HOME 5      ///< 回零 / Homing

/// @brief 运行状态枚举 / Running status enum
#define C_LST_STOPPED 0     ///< 正常停止状态 / Normal stopped state
#define C_LST_RUNNING 1     ///< 正常运行状态 / Normal running state
#define C_LST_ALARM 2       ///< 报警停止状态 / Alarm stopped state
#define C_LST_POS_LIMIT 3   ///< 正限位状态 / Positive limit state
#define C_LST_NEG_LIMIT 4   ///< 负限位状态 / Negative limit state
#define C_LST_BOTH_LIMIT 5  ///< 正负限位状态 / Both limits state
#define C_LST_EMG_STOP 6    ///< 急停状态 / Emergency stop state
#define C_LST_HOMING 7      ///< 回零运行状态 / Homing running state

/// @brief 报警类型枚举 / Alarm type enum
#define C_LAM_NULL 0       ///< 无报警 / No alarm
#define C_LAM_POS_ERR 1    ///< 位置超差 / Position error
#define C_LAM_OVER_SPD 2   ///< 超速 / Over speed
#define C_LAM_OVER_CUR 3   ///< 过流 / Over current
#define C_LAM_OVER_LOAD 4  ///< 过载 / Over load
#define C_LAM_OVER_VOL 5   ///< 过压 / Over voltage
#define C_LAM_UNDER_VOL 6  ///< 欠压 / Under voltage
#define C_LAM_ENC_ERR 7    ///< 编码器错误 / Encoder error
#define C_LAM_STALL 8      ///< 堵转 / Stall
#define C_LAM_OTHER 9      ///< 其他报警 / Other alarm

/// @brief 6自由度关节枚举 / 6 degrees of freedom joint enum
#define C_LMI6_ALL_JOINTS 0       ///< 全部关节 / All joints
#define C_LMI6_THUMB_ABDUCTION 1  ///< 大拇指侧摆 / Thumb abduction
#define C_LMI6_THUMB_FLEXION 2    ///< 大拇指弯曲 / Thumb flexion
#define C_LMI6_INDEX_FLEXION 3    ///< 食指弯曲 / Index finger flexion
#define C_LMI6_MIDDLE_FLEXION 4   ///< 中指弯曲 / Middle finger flexion
#define C_LMI6_RING_FLEXION 5     ///< 无名指弯曲 / Ring finger flexion
#define C_LMI6_PINKY_FLEXION 6    ///< 小拇指弯曲 / Pinky finger flexion

/// @brief 16自由度关节枚举 / 16 degrees of freedom joint enum
#define C_LMI16_ALL_JOINTS 0          ///< 全部关节 / All joints
#define C_LMI16_THUMB_LATERAL_EX 1    ///< 大拇指加强侧摆（独立轴）/ Thumb lateral extended (independent axis)
#define C_LMI16_THUMB_LATERAL 2       ///< 大拇指侧摆 / Thumb lateral (abduction)
#define C_LMI16_THUMB_PROXIMAL 3      ///< 大拇指近节 / Thumb proximal
#define C_LMI16_THUMB_DISTAL 4        ///< 大拇指远节 / Thumb distal
#define C_LMI16_INDEX_LATERAL 5       ///< 食指侧摆 / Index lateral
#define C_LMI16_INDEX_PROXIMAL 6      ///< 食指近节 / Index proximal
#define C_LMI16_INDEX_DISTAL 7        ///< 食指远节 / Index distal
#define C_LMI16_MIDDLE_LATERAL 8      ///< 中指侧摆 / Middle lateral
#define C_LMI16_MIDDLE_PROXIMAL 9     ///< 中指近节 / Middle proximal
#define C_LMI16_MIDDLE_DISTAL 10      ///< 中指远节 / Middle distal
#define C_LMI16_RING_LATERAL 11       ///< 无名指侧摆 / Ring lateral
#define C_LMI16_RING_PROXIMAL 12      ///< 无名指近节 / Ring proximal
#define C_LMI16_RING_DISTAL 13        ///< 无名指远节 / Ring distal
#define C_LMI16_PINKY_LATERAL 14      ///< 小拇指侧摆 / Pinky lateral
#define C_LMI16_PINKY_PROXIMAL 15     ///< 小拇指近节 / Pinky proximal
#define C_LMI16_PINKY_DISTAL 16       ///< 小拇指远节 / Pinky distal

/// @brief 传感器id枚举 / Sensor id enum
#define C_LSS_FINGER_1_1 1   ///< 大拇指指尖 / Thumb tip
#define C_LSS_FINGER_1_2 2   ///< 大拇指指腹 / Thumb pad
#define C_LSS_FINGER_2_1 3   ///< 食指指尖 / Index finger tip
#define C_LSS_FINGER_2_2 4   ///< 食指指腹 / Index finger pad
#define C_LSS_FINGER_3_1 5   ///< 中指指尖 / Middle finger tip
#define C_LSS_FINGER_3_2 6   ///< 中指指腹 / Middle finger pad
#define C_LSS_FINGER_4_1 7   ///< 无名指指尖 / Ring finger tip
#define C_LSS_FINGER_4_2 8   ///< 无名指指腹 / Ring finger pad
#define C_LSS_FINGER_5_1 9   ///< 小拇指指尖 / Little finger tip
#define C_LSS_FINGER_5_2 10  ///< 小拇指指腹 / Little finger pad
#define C_LSS_HAND_PALM 11   ///< 手掌 / Palm
#define C_LSS_MAX_COUNT 12   ///< 最大传感器数量 / Maximum sensor count

/// @brief 基本类型定义 / Basic type definition
typedef void* lhandprolib_handle;
typedef void* lhandprolib_rs485_shared_bus_handle;

/// @brief 回调函数类型定义 / Callback function type definition

/// @brief 日志回调 / Log callback
/// @param const char* 日志内容 / Log message
typedef void (*LogAddCallbackWrapper)(const char*);

/// @brief EtherCAT 发送数据回调 / EtherCAT send data callback
/// @param const unsigned char* 数据指针 / Data pointer
/// @param unsigned int 数据长度 / Data size
/// @return true-成功 / true on success
typedef bool (*ECSendDataCallbackWrapper)(const unsigned char*, unsigned int);

/// @brief CANFD 发送数据回调 / CANFD send data callback
/// @param unsigned int CAN ID
/// @param const unsigned char* 数据指针 / Data pointer
/// @param unsigned int 数据长度 / Data size
/// @param int 是否扩展帧 / Extended frame flag
/// @return true-成功 / true on success
typedef bool (*CANFDSendDataCallbackWrapper)(unsigned int, const unsigned char*,
                                             unsigned int, int);

/// @brief RS485 发送数据回调 / RS485 send data callback
/// @param const unsigned char* 数据指针 / Data pointer
/// @param unsigned int 数据长度 / Data size
/// @return true-成功 / true on success
typedef bool (*RS485SendDataCallbackWrapper)(const unsigned char*,
                                             unsigned int);

/// @brief 标准CAN 发送数据回调 / Standard CAN send data callback
/// @param unsigned int CAN ID
/// @param const unsigned char* 数据指针 / Data pointer
/// @param unsigned int 数据长度 / Data size
/// @return true-成功 / true on success
typedef bool (*CANSendDataCallbackWrapper)(unsigned int, const unsigned char*,
                                           unsigned int);

/// @brief 创建共享 RS485 总线 / Create a shared RS485 bus
SDK_API lhandprolib_rs485_shared_bus_handle
    lhandprolib_rs485_shared_bus_create();

/// @brief 销毁共享 RS485 总线 / Destroy a shared RS485 bus
SDK_API void lhandprolib_rs485_shared_bus_destroy(
    lhandprolib_rs485_shared_bus_handle bus_handle);

/// @brief 设置共享总线物理串口发送回调
///        Set the physical serial-port send callback of a shared bus
SDK_API void lhandprolib_rs485_shared_bus_set_send_callback(
    lhandprolib_rs485_shared_bus_handle bus_handle,
    RS485SendDataCallbackWrapper callback);

/// @brief 输入共享总线的串口原始接收数据
///        Feed raw serial receive data to a shared bus
SDK_API int lhandprolib_rs485_shared_bus_set_receive_data(
    lhandprolib_rs485_shared_bus_handle bus_handle,
    const unsigned char* data_ptr, int data_size);

/// @brief 停止共享 RS485 总线 / Stop a shared RS485 bus
SDK_API void lhandprolib_rs485_shared_bus_close(
    lhandprolib_rs485_shared_bus_handle bus_handle);

/// @brief 创建LHandProLib实例
///      Create LHandProLib instance
/// @return 成功返回实例句柄，失败返回NULL
///      Successfully returns instance handle, returns NULL on failure
/// @example
///           lhandprolib_handle handle = lhandprolib_create();
SDK_API lhandprolib_handle lhandprolib_create();

/// @brief 销毁LHandProLib实例
///      Destroy LHandProLib instance
/// @param handle 实例句柄
///        handle: Instance handle
/// @example
///           lhandprolib_destroy(handle);
SDK_API void lhandprolib_destroy(lhandprolib_handle handle);

/// @brief 释放由 C API 分配的缓冲区
///        Free buffers allocated by the C API
/// @param ptr 由 SDK_C_API(_get_finger_*) 等接口返回的数组指针
///        ptr: Array pointer returned by SDK_C_API(_get_finger_*) APIs
/// @note 该接口等价于释放 SDK 内部分配的 malloc 缓冲区；空指针安全
///       Equivalent to freeing SDK-owned malloc buffers; null is safe
SDK_API void lhandprolib_free_buffer(void* ptr);

/// @brief 初始化灵巧手驱动程序
///      Initialize dexterous hand driver
/// @param handle 实例句柄
///        handle: Instance handle
/// @param mode 通讯模式 C_LCN_ECAT/C_LCN_CANFD/C_LCN_RS485
///        mode: Communication mode C_LCN_ECAT/C_LCN_CANFD/C_LCN_RS485
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///      Successfully returns C_LER_NONE(0), others - refer to error codes
/// @example
///           // 1. 创建实例
///           lhandprolib_handle handle = lhandprolib_create();
///           // 2. 初始化（使用EtherCAT模式）
///           int result = lhandprolib_initial(handle, C_LCN_ECAT);
SDK_API int lhandprolib_initial(lhandprolib_handle handle, int mode);

/// @brief 初始化灵巧手驱动程序（自定义节点ID）
///      Initialize dexterous hand driver (custom node ID)
/// @param handle 实例句柄
///        handle: Instance handle
/// @param mode 通讯模式 C_LCN_CANFD/C_LCN_RS485
///        mode: Communication mode C_LCN_CANFD/C_LCN_RS485
/// @param node_id CANFD或RS485的节点ID
///        node_id: CANFD or RS485 node ID
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///      Successfully returns C_LER_NONE(0), others - refer to error codes
SDK_API int lhandprolib_initial_ex(lhandprolib_handle handle, int mode,
                                        int node_id);

/// @brief 关闭灵巧手驱动程序
///      Close dexterous hand driver
/// @param handle 实例句柄
///        handle: Instance handle
/// @example
///           lhandprolib_close(handle);
SDK_API void lhandprolib_close(lhandprolib_handle handle);

/// @brief 启动后台监控线程
///      Start background monitor thread
/// @param handle 实例句柄
///        handle: Instance handle
SDK_API void lhandprolib_start_monitor(lhandprolib_handle handle);

/// @brief 停止后台监控线程
///      Stop background monitor thread
/// @param handle 实例句柄
///        handle: Instance handle
SDK_API void lhandprolib_stop_monitor(lhandprolib_handle handle);

/// @brief 设置 dry-run 模式，开启后命令数据正常生成但不通过回调发送
///      Set dry-run mode; when enabled, command data is generated normally but not sent via callback
/// @param handle 实例句柄
///        handle: Instance handle
/// @param enable true - 开启 dry-run, false - 关闭
///        enable: true - enable dry-run, false - disable
SDK_API void lhandprolib_set_dry_run_mode(lhandprolib_handle handle,
                                               bool enable);

/// @brief 获取当前 dry-run 模式状态
///      Get current dry-run mode state
/// @param handle 实例句柄
///        handle: Instance handle
/// @return true - dry-run 已开启, false - 已关闭
///         true - dry-run enabled, false - disabled
SDK_API bool lhandprolib_get_dry_run_mode(lhandprolib_handle handle);

/// @brief 设置EtherCAT发送数据回调函数
///      Set EtherCAT send data callback function
/// @param handle 实例句柄
///        handle: Instance handle
/// @param callback 回调函数指针，用于发送RPDO数据
///        callback: Callback function pointer for sending RPDO data
/// @example
///           // 定义回调函数
///           // Define callback function
///           bool send_rpdo_callback(const unsigned char* data, unsigned int size) {
///               // 自行实现EtherCAT数据发送逻辑
///               // Implement EtherCAT data sending logic yourself
///               return true; // 发送成功返回true
///           }
///           // 设置回调
///           // Set callback
///           lhandprolib_set_send_rpdo_callback(handle, send_rpdo_callback);
SDK_API void lhandprolib_set_send_rpdo_callback(
    lhandprolib_handle handle, ECSendDataCallbackWrapper callback);

/// @brief 设置CANFD发送数据回调函数
///      Set CANFD send data callback function
/// @param handle 实例句柄
///        handle: Instance handle
/// @param callback 回调函数指针，用于发送CANFD数据
///        callback: Callback function pointer for sending CANFD data
/// @example
///           // 定义回调函数
///           // Define callback function
///           bool send_canfd_callback(const unsigned char* data, unsigned int size) {
///               // 自行实现CANFD数据发送逻辑
///               return true; // 发送成功返回true
///           }
///           // 设置回调
///           // Set callback
///           lhandprolib_set_send_canfd_callback(handle, send_canfd_callback);
SDK_API void lhandprolib_set_send_canfd_callback(
    lhandprolib_handle handle, CANFDSendDataCallbackWrapper callback);

/// @brief 设置日志回调函数
///      Set log callback function
/// @param handle 实例句柄
///        handle: Instance handle
/// @param callback 回调函数指针，用于输出日志
///        callback: Callback function pointer for outputting logs
/// @example
///           // 定义回调函数
///           // Define callback function
///           void log_callback(const char* log) {
///               printf("%s\n", log);
///           }
///           // 设置回调
///           // Set callback
///           lhandprolib_set_log_callback(handle, log_callback);
SDK_API void lhandprolib_set_log_callback(lhandprolib_handle handle,
                                               LogAddCallbackWrapper callback);

// 数据接收处理
/// @brief 解码传入的TPDO数据（EtherCAT）
///      Decode incoming TPDO data (EtherCAT)
/// @param handle 实例句柄
///        handle: Instance handle
/// @param data_ptr 待解码的数据指针
///        data_ptr: Pointer to data to be decoded
/// @param data_size 数据长度
///        data_size: Data length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///         C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 假设已经从EtherCAT获取到TPDO数据
///           // Assume TPDO data has been received from EtherCAT
///           unsigned char tpdo_data[128];
///           int tpdo_size = 128;
///           // 解码数据
///           // Decode data
///           lhandprolib_set_tpdo_data_decode(handle, tpdo_data, tpdo_size);
///           // 解码后可以使用其他接口获取数据
///           // After decoding, you can use other interfaces to get data
SDK_API int lhandprolib_set_tpdo_data_decode(lhandprolib_handle handle,
                                                  const unsigned char* data_ptr,
                                                  int data_size);

/// @brief 解码传入的CANFD数据
///      Decode incoming CANFD data
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id CANFD消息ID
///        id: CANFD message ID
/// @param data_ptr 待解码的数据指针
///        data_ptr: Pointer to data to be decoded
/// @param data_size 数据长度
///        data_size: Data length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 假设已经从CANFD获取到数据
///           // Assume CANFD data has been received
///           unsigned int id;
///           unsigned char canfd_data[64];
///           int canfd_size = 64;
///           // 解码数据
///           // Decode data
///           lhandprolib_set_canfd_data_decode(handle, id, canfd_data, canfd_size);
///           // 解码后可以使用其他接口获取数据
SDK_API int lhandprolib_set_canfd_data_decode(
    lhandprolib_handle handle, unsigned int id, const unsigned char* data_ptr,
    int data_size);

/// @brief 设置RS485发送数据回调函数
///      Set RS485 send data callback function
/// @param handle 实例句柄
///        handle: Instance handle
/// @param callback 回调函数指针，用于发送RS485数据
///        callback: Callback function pointer for sending RS485 data
/// @example
///           // Define callback function
///           bool send_rs485_callback(const unsigned char* data, unsigned int size) {
///               // Implement RS485 data sending logic
///               return true; // Return true on success
///           }
///           // Set callback
///           lhandprolib_set_send_rs485_callback(handle, send_rs485_callback);
SDK_API void lhandprolib_set_send_rs485_callback(
    lhandprolib_handle handle, RS485SendDataCallbackWrapper callback);

/// @brief 将SDK实例绑定到共享RS485总线；在initial_ex前调用
///        Bind an SDK instance to a shared RS485 bus; call before initial_ex
/// @param bus_handle 传入NULL恢复旧的单设备隐式总线模式
///        Pass NULL to restore the legacy implicit single-device bus
SDK_API void lhandprolib_set_rs485_shared_bus(
    lhandprolib_handle handle,
    lhandprolib_rs485_shared_bus_handle bus_handle);

/// @brief 获取预发送区的RS485完整帧数据（含地址头 + CRC）
///      Get pre-send RS485 complete framed data (with address header + CRC)
/// @param handle 实例句柄
///        handle: Instance handle
/// @param data_ptr 返回的数据指针（nullptr时仅获取长度）
///        data_ptr: Returned data pointer (only get length when nullptr)
/// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
///        io_size: Input indicates buffer size, output returns actual data size
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 1. 第一次调用获取数据长度
///           // First call to get data length
///           int size = 0;
///           lhandprolib_get_pre_send_rs485_data(handle, nullptr, &size);
///           // 2. 第二次调用获取实际数据
///           // Second call to get actual data
///           unsigned char* rs485_data = (unsigned char*)malloc(size);
///           lhandprolib_get_pre_send_rs485_data(handle, rs485_data, &size);
///           // 3. 使用数据后释放内存
///           // After using the data, free the memory
///           free(rs485_data);
SDK_API int lhandprolib_get_pre_send_rs485_data(lhandprolib_handle handle,
                                                     unsigned char* data_ptr,
                                                     int* io_size);

/// @brief 解码传入的RS485数据
///      Decode incoming RS485 data
/// @param handle 实例句柄
///        handle: Instance handle
/// @param data_ptr 待解码的数据指针，可以是任意长度的串口接收片段
///        data_ptr: Data pointer to be decoded; may be any serial receive chunk
/// @param data_size 数据长度
///        data_size: Data length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 传入串口读取到的原始数据片段
///           // Feed raw serial bytes to the SDK
///           unsigned char rs485_data[64];
///           int rs485_size = 64;
///           lhandprolib_set_rs485_data_decode(handle, rs485_data, rs485_size);
///           // 解码后可以使用其他接口获取数据
SDK_API int lhandprolib_set_rs485_data_decode(
    lhandprolib_handle handle, const unsigned char* data_ptr, int data_size);

/// @brief 设置标准CAN发送数据回调函数
///      Set standard CAN send data callback function
/// @param handle 实例句柄
///        handle: Instance handle
/// @param callback 回调函数指针，用于发送标准CAN数据
///        callback: Callback function pointer for sending standard CAN data
/// @example
///           // Define callback function
///           bool send_can_callback(unsigned int id, const unsigned char* data, unsigned int size) {
///               // Implement CAN data sending logic
///               return true; // Return true on success
///           }
///           // Set callback
///           lhandprolib_set_send_can_callback(handle, send_can_callback);
SDK_API void lhandprolib_set_send_can_callback(
    lhandprolib_handle handle, CANSendDataCallbackWrapper callback);

/// @brief 解码传入的标准CAN数据
///      Decode incoming standard CAN data
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id CAN消息ID
///        id: CAN message ID
/// @param data_ptr 待解码的数据指针
///        data_ptr: Pointer to data to be decoded
/// @param data_size 数据长度
///        data_size: Data length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
SDK_API int lhandprolib_set_can_data_decode(lhandprolib_handle handle,
                                                 unsigned int id,
                                                 const unsigned char* data_ptr,
                                                 int data_size);

/// @brief 获取预发送区的标准CAN数据（dry-run捕获），每帧11字节: CAN_ID(4B) + data(7B)
///      Get standard CAN data in pre-send area (dry-run capture), each frame 11 bytes: CAN_ID(4B) + data(7B)
/// @param handle 实例句柄
///        handle: Instance handle
/// @param data_ptr 返回的数据指针（nullptr时仅获取长度）
///        data_ptr: Returned data pointer (only get length when nullptr)
/// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
///        io_size: Input indicates buffer size, output returns actual data size
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
SDK_API int lhandprolib_get_pre_send_can_data(lhandprolib_handle handle,
                                                    unsigned char* data_ptr,
                                                    int* io_size);

// RPDO数据处理
/// @brief 获取预发送区的RPDO数据（EtherCAT）
///     Get pre-send RPDO data (EtherCAT)
/// @param handle 实例句柄
///        handle: Instance handle
/// @param data_ptr 返回的数据指针（nullptr时仅获取长度）
///        data_ptr: Returned data pointer (only get length when nullptr)
/// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
///        io_size: Input indicates buffer size, output returns actual data size
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 1. 第一次调用获取数据长度
///           // First call to get data length
///           int size = 0;
///           lhandprolib_get_pre_send_rpdo_data(handle, nullptr, &size);
///           // 2. 第二次调用获取实际数据
///           // Second call to get actual data
///           unsigned char* rpdo_data = (unsigned char*)malloc(size);
///           lhandprolib_get_pre_send_rpdo_data(handle, rpdo_data, &size);
///           // 3. 使用数据后释放内存
///           // After using the data, free the memory
///           free(rpdo_data);
SDK_API int lhandprolib_get_pre_send_rpdo_data(lhandprolib_handle handle,
                                                    unsigned char* data_ptr,
                                                    int* io_size);

/// @brief 获取预发送区的CANFD数据
///      Get pre-send CANFD data
/// @param handle 实例句柄
///        handle: Instance handle
/// @param data_ptr 返回的数据指针（nullptr时仅获取长度）
///        data_ptr: Returned data pointer (only get length when nullptr)
/// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
///        io_size: Input indicates buffer size, output returns actual data size
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 1. 第一次调用获取数据长度
///           // First call to get data length
///           int size = 0;
///           lhandprolib_get_pre_send_canfd_data(handle, nullptr, &size);
///           // 2. 第二次调用获取实际数据
///           // Second call to get actual data
///           unsigned char* canfd_data = (unsigned char*)malloc(size);
///           lhandprolib_get_pre_send_canfd_data(handle, canfd_data, &size);
///           // 3. 使用数据后释放内存
///           // After using the data, free the memory
///           free(canfd_data);
SDK_API int lhandprolib_get_pre_send_canfd_data(lhandprolib_handle handle,
                                                     unsigned char* data_ptr,
                                                     int* io_size);

/// @brief 获取固件版本号
///        Get firmware version
/// @param handle 实例句柄
///        handle: Instance handle
/// @param version 返回的固件版本号（例如：0.32 表示版本 0.32）
///        version: Returned firmware version (e.g., 0.32 means version 0.32)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float ver;
///           lhandprolib_get_firmware_version(handle, &ver);
SDK_API int lhandprolib_get_firmware_version(lhandprolib_handle handle,
                                                  float* version);

/// @brief 获取灵巧手SN码
///        Get dexterous hand serial number
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sn 输出缓冲区, 用于存放SN码字符串
///        sn: Output buffer for serial number string
/// @param size 缓冲区大小
///        size: Buffer size
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           char sn[64];
///           lhandprolib_get_serial_number(handle, sn, sizeof(sn));
SDK_API int lhandprolib_get_serial_number(lhandprolib_handle handle,
                                                char* sn, int size);

/// @brief SDO设置驱动参数
///        Set SDO drive parameters
/// @param handle 实例句柄
///        handle: Instance handle
/// @param index 参数索引
///        index: Parameter index
/// @param subindex 参数子索引
///        subindex: Parameter subindex
/// @param value 参数值
///        value: Parameter value
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_sdo_drive_param(handle, 0x200E, 0x1D, 0x01);
SDK_API int lhandprolib_set_sdo_drive_param(lhandprolib_handle handle,
                                                 unsigned int index,
                                                 unsigned char subindex,
                                                 unsigned int value);

/// @brief SDO获取驱动参数
///        Get SDO drive parameters
/// @param handle 实例句柄
///        handle: Instance handle
/// @param index 参数索引
///        index: Parameter index
/// @param subindex 参数子索引
///        subindex: Parameter subindex
/// @param value 返回的参数值指针
///        value: Returned parameter value pointer
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           unsigned int val = 0;
///           lhandprolib_get_sdo_drive_param(handle, 0x200E, 0x1D, &val);
SDK_API int lhandprolib_get_sdo_drive_param(lhandprolib_handle handle,
                                                 unsigned int index,
                                                 unsigned char subindex,
                                                 unsigned int* value);

/// @brief SDO保存驱动参数
///        Save SDO drive parameters
/// @param handle 实例句柄
///        handle: Instance handle
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_save_sdo_drive_param(handle);
SDK_API int lhandprolib_save_sdo_drive_param(lhandprolib_handle handle);

/// @brief 获取当前灵巧手自由度数量
///        Get current degree of freedom (DOF) of the dexterous hand
/// @param handle 实例句柄
///        handle: Instance handle
/// @param total 输出参数，自由度总数量（关节数量）
///        total: Output parameter, total degree of freedom (joint number)
/// @param active 输出参数，主动自由度数量（电机数量）
///        active: Output parameter, active degree of freedom (motor number)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int total_dof, active_dof;
///           lhandprolib_get_dof(handle, &total_dof, &active_dof);
SDK_API int lhandprolib_get_dof(lhandprolib_handle handle, int* total,
                                     int* active);

/// @brief 设置手的类型
///        Set the type of the dexterous hand
/// @param handle 实例句柄
///        handle: Instance handle
/// @param hand_type 手的类型 C_LAC_DOF_6/C_LAC_DOF_6_S/C_LAC_DOF_16
///        hand_type: Hand type C_LAC_DOF_6/C_LAC_DOF_6_S/C_LAC_DOF_16
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_hand_type(handle, C_LAC_DOF_6);
SDK_API int lhandprolib_set_hand_type(lhandprolib_handle handle,
                                           int hand_type);

/// @brief 获取手的类型
///        Get the type of the dexterous hand
/// @param handle 实例句柄
///        handle: Instance handle
/// @param hand_type 输出参数，手的类型 C_LAC_DOF_6/C_LAC_DOF_6_S/C_LAC_DOF_16
///        hand_type: Output parameter, hand type C_LAC_DOF_6/C_LAC_DOF_6_S/C_LAC_DOF_16
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int hand_type;
///           lhandprolib_get_hand_type(handle, &hand_type);
SDK_API int lhandprolib_get_hand_type(lhandprolib_handle handle,
                                           int* hand_type);

/// @brief 设置手的方向
///        Set the direction of the dexterous hand
/// @param handle 实例句柄
///        handle: Instance handle
/// @param dir 手的方向 C_LDR_HAND_RIGHT/C_LDR_HAND_LEFT
///        dir: Hand direction C_LDR_HAND_RIGHT/C_LDR_HAND_LEFT
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_hand_direction(handle, C_LDR_HAND_RIGHT);
SDK_API int lhandprolib_set_hand_direction(lhandprolib_handle handle,
                                                int dir);

/// @brief 获取手的方向
///        Get the direction of the dexterous hand
/// @param handle 实例句柄
///        handle: Instance handle
/// @param dir 输出参数，手的方向 C_LDR_HAND_RIGHT/C_LDR_HAND_LEFT
///        dir: Output parameter, hand direction C_LDR_HAND_RIGHT/C_LDR_HAND_LEFT
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int hand_dir;
///           lhandprolib_get_hand_direction(handle, &hand_dir);
SDK_API int lhandprolib_get_hand_direction(lhandprolib_handle handle,
                                                int* dir);

/// @brief 设置电机控制模式
///      Set motor control mode
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @param mode 控制模式 C_LCM_POSITION/C_LCM_VELOCITY/C_LCM_TORQUE等
///        mode: Control mode C_LCM_POSITION/C_LCM_VELOCITY/C_LCM_TORQUE, etc.
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 设置电机1为位置控制模式
///             // Set motor 1 to position control mode
///           lhandprolib_set_control_mode(handle, 1, C_LCM_POSITION);
SDK_API int lhandprolib_set_control_mode(lhandprolib_handle handle, int id,
                                              int mode);

/// @brief 获取电机控制模式
///      Get motor control mode
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param mode 输出参数，控制模式
///        mode: Output parameter, control mode
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int control_mode;
///           lhandprolib_get_control_mode(handle, 1, &control_mode);
SDK_API int lhandprolib_get_control_mode(lhandprolib_handle handle, int id,
                                              int* mode);

/// @brief 设置安全电流使能
///      Set safe current enable
/// @param handle 实例句柄
///        handle: Instance handle
/// @param enable 安全电流使能，1-使能，0-去使能
///        enable: Safe current enable, 1-enable, 0-disable
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_safe_current_enable(handle, 1);
SDK_API int lhandprolib_set_safe_current_enable(lhandprolib_handle handle,
                                                     int enable);

/// @brief 获取安全电流使能状态
///      Get safe current enable state
/// @param handle 实例句柄
///        handle: Instance handle
/// @param enable 输出参数，安全电流使能状态，1-使能，0-去使能
///        enable: Output parameter, safe current enable state, 1-enable, 0-disable
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int enable = 0;
///           lhandprolib_get_safe_current_enable(handle, &enable);
SDK_API int lhandprolib_get_safe_current_enable(lhandprolib_handle handle,
                                                     int* enable);

/// @brief 设置回零电流
///      Set homing current
/// @param handle 实例句柄
///        handle: Instance handle
/// @param current 回零电流值, 单位 %(百分比)
///        current: Homing current value, unit %(percentage)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_home_current(handle, 100);
SDK_API int lhandprolib_set_home_current(lhandprolib_handle handle,
                                              int current);

/// @brief 获取回零电流
///      Get homing current
/// @param handle 实例句柄
///        handle: Instance handle
/// @param current 输出参数，回零电流值, 单位 %(百分比)
///        current: Output parameter, homing current value, unit %(percentage)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int current = 0;
///           lhandprolib_get_home_current(handle, &current);
SDK_API int lhandprolib_get_home_current(lhandprolib_handle handle,
                                              int* current);

/// @brief 设置CAN节点号
///      Set CAN node ID
/// @param handle 实例句柄
///        handle: Instance handle
/// @param node_id 节点号
///        node_id: Node ID
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_can_node_id(handle, 1);
SDK_API int lhandprolib_set_can_node_id(lhandprolib_handle handle,
                                             int node_id);

/// @brief 获取CAN节点号
///      Get CAN node ID
/// @param handle 实例句柄
///        handle: Instance handle
/// @param node_id 输出参数，节点号
///        node_id: Output parameter, node ID
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int node_id = 0;
///           lhandprolib_get_can_node_id(handle, &node_id);
SDK_API int lhandprolib_get_can_node_id(lhandprolib_handle handle,
                                             int* node_id);

/// @brief 设置CANFD仲裁段波特率
///      Set CANFD arbitration segment baud rate
/// @param handle 实例句柄
///        handle: Instance handle
/// @param baudrate 波特率值
///        baudrate: Baud rate value
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_canfd_arb_baudrate(handle, 1000000);
SDK_API int lhandprolib_set_canfd_arb_baudrate(lhandprolib_handle handle,
                                                    int baudrate);

/// @brief 获取CANFD仲裁段波特率
///      Get CANFD arbitration segment baud rate
/// @param handle 实例句柄
///        handle: Instance handle
/// @param baudrate 输出参数，波特率值
///        baudrate: Output parameter, baud rate value
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int baudrate = 0;
///           lhandprolib_get_canfd_arb_baudrate(handle, &baudrate);
SDK_API int lhandprolib_get_canfd_arb_baudrate(lhandprolib_handle handle,
                                                    int* baudrate);

/// @brief 设置CANFD数据段波特率
///      Set CANFD data segment baud rate
/// @param handle 实例句柄
///        handle: Instance handle
/// @param baudrate 波特率值
///        baudrate: Baud rate value
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_canfd_data_baudrate(handle, 5000000);
SDK_API int lhandprolib_set_canfd_data_baudrate(lhandprolib_handle handle,
                                                     int baudrate);

/// @brief 获取CANFD数据段波特率
///      Get CANFD data segment baud rate
/// @param handle 实例句柄
///        handle: Instance handle
/// @param baudrate 输出参数，波特率值
///        baudrate: Output parameter, baud rate value
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int baudrate = 0;
///           lhandprolib_get_canfd_data_baudrate(handle, &baudrate);
SDK_API int lhandprolib_get_canfd_data_baudrate(lhandprolib_handle handle,
                                                     int* baudrate);

/// @brief 设置RS485节点号
///      Set RS485 node ID
/// @param handle 实例句柄
///        handle: Instance handle
/// @param node_id 节点号
///        node_id: Node ID
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_rs485_node_id(handle, 1);
SDK_API int lhandprolib_set_rs485_node_id(lhandprolib_handle handle,
                                               int node_id);

/// @brief 获取RS485节点号
///      Get RS485 node ID
/// @param handle 实例句柄
///        handle: Instance handle
/// @param node_id 输出参数，节点号
///        node_id: Output parameter, node ID
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int node_id = 0;
///           lhandprolib_get_rs485_node_id(handle, &node_id);
SDK_API int lhandprolib_get_rs485_node_id(lhandprolib_handle handle,
                                               int* node_id);

/// @brief 设置RS485波特率
///      Set RS485 baud rate
/// @param handle 实例句柄
///        handle: Instance handle
/// @param baudrate 波特率值
///        baudrate: Baud rate value
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_rs485_baudrate(handle, 500000);
SDK_API int lhandprolib_set_rs485_baudrate(lhandprolib_handle handle,
                                                int baudrate);

/// @brief 获取RS485波特率
///      Get RS485 baud rate
/// @param handle 实例句柄
///        handle: Instance handle
/// @param baudrate 输出参数，波特率值
///        baudrate: Output parameter, baud rate value
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int baudrate = 0;
///           lhandprolib_get_rs485_baudrate(handle, &baudrate);
SDK_API int lhandprolib_get_rs485_baudrate(lhandprolib_handle handle,
                                                int* baudrate);

/// @brief 设置电机使能状态
///      Set motor enable state
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @param enable 使能状态，1-使能，0-失能
///        enable: Enable state, 1-enable, 0-disable
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 使能电机1
///             // Enable motor 1
///           lhandprolib_set_enable(handle, 1, 1);
SDK_API int lhandprolib_set_enable(lhandprolib_handle handle, int id,
                                        int enable);

/// @brief 获取电机使能状态
///      Get motor enable state
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param enable 输出参数，使能状态，1-使能，0-失能
///        enable: Output parameter, enable state, 1-enable, 0-disable
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int enable_status;
///           lhandprolib_get_enable(handle, 1, &enable_status);
SDK_API int lhandprolib_get_enable(lhandprolib_handle handle, int id,
                                        int* enable);

/// @brief 获取位置到位状态
///      Get position reached state
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param reached 输出参数，到位状态，1-到位，0-未到位
///        reached: Output parameter, reached state, 1-reached, 0-not reached
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int pos_reached;
///           lhandprolib_get_position_reached(handle, 1, &pos_reached);
SDK_API int lhandprolib_get_position_reached(lhandprolib_handle handle,
                                                  int id, int* reached);

/// @brief 获取力矩到位状态
///      Get torque reached state
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param reached 输出参数，到位状态，1-到位，0-未到位
///        reached: Output parameter, reached state, 1-reached, 0-not reached
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int torque_reached;
///           lhandprolib_get_torque_reached(handle, 1, &torque_reached);
SDK_API int lhandprolib_get_torque_reached(lhandprolib_handle handle,
                                                int id, int* reached);

/// @brief 清除电机报警
///      Clear motor alarm
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 清除电机1的报警
///           lhandprolib_set_clear_alarm(handle, 1);
SDK_API int lhandprolib_set_clear_alarm(lhandprolib_handle handle, int id);

/// @brief 获取电机当前报警状态
///      Get current motor alarm state
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param alarm 输出参数，报警状态，0-无报警
///        alarm: Output parameter, alarm state, 0-no alarm
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int alarm_status;
///           lhandprolib_get_now_alarm(handle, 1, &alarm_status);
SDK_API int lhandprolib_get_now_alarm(lhandprolib_handle handle, int id,
                                           int* alarm);

/// @brief 是否允许在没回零状态下运动
///      Allow movement without homing
/// @param handle 实例句柄
///        handle: Instance handle
/// @param enable 0: 不允许 1: 允许
///        enable: 0: not allowed 1: allowed
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_move_no_home(handle, 0);
SDK_API int lhandprolib_set_move_no_home(lhandprolib_handle handle,
                                              int enable);

/// @brief 设置是否启用LAC_DOF_6_S手指角度/连杆角度公式转换
///      Enable LAC_DOF_6_S finger/linkage angle conversion formulas
/// @param handle 实例句柄
///        handle: Instance handle
/// @param enable 0: 关闭 1: 开启
///        enable: 0: disable 1: enable
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
SDK_API int lhandprolib_set_angle_conversion_enable(
    lhandprolib_handle handle, int enable);

/// @brief 获取角度公式转换开关状态
///      Get angle conversion formula enable state
/// @param handle 实例句柄
///        handle: Instance handle
/// @param enable 输出参数，0: 关闭 1: 开启
///        enable: Output parameter, 0: disable 1: enable
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
SDK_API int lhandprolib_get_angle_conversion_enable(
    lhandprolib_handle handle, int* enable);

/// @brief 电机回零
///      Home motors
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 电机1回零
///           lhandprolib_home_motors(handle, 1);
SDK_API int lhandprolib_home_motors(lhandprolib_handle handle, int id);

/// @brief 获取电机目标角度上下限
///        Get motor target angle limits
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param min_angle 输出参数，目标角度最小值
///        min_angle: Output parameter, minimum target angle
/// @param max_angle 输出参数，目标角度最大值
///        max_angle: Output parameter, maximum target angle
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float min_angle, max_angle;
///           lhandprolib_get_limit_target_angle(handle, 1, &min_angle, &max_angle);
SDK_API int lhandprolib_get_limit_target_angle(lhandprolib_handle handle,
                                                    int id, float* min_angle,
                                                    float* max_angle);

/// @brief 设置目标角度
///      Set target angle
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @param angle 目标角度（单位：度）
///        angle: Target angle (unit: degrees)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 设置电机1目标角度为90度
///             // Set motor 1 target angle to 90 degrees
///           lhandprolib_set_target_angle(handle, 1, 90.0f);
SDK_API int lhandprolib_set_target_angle(lhandprolib_handle handle, int id,
                                              float angle);

/// @brief 获取目标角度
///      Get target angle
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param angle 输出参数，目标角度（单位：度）
///        angle: Output parameter, target angle (unit: degrees)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///          C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float target_angle;
///           lhandprolib_get_target_angle(handle, 1, &target_angle);
SDK_API int lhandprolib_get_target_angle(lhandprolib_handle handle, int id,
                                              float* angle);

/// @brief 设置目标位置
///      Set target position
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @param position 目标位置（编码器计数）
///        position: Target position (encoder counts)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 设置电机1目标位置为10000
///             // Set motor 1 target position to 10000
///           lhandprolib_set_target_position(handle, 1, 10000);
SDK_API int lhandprolib_set_target_position(lhandprolib_handle handle,
                                                 int id, int position);

/// @brief 获取目标位置
///      Get target position
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param position 输出参数，目标位置（编码器计数）
///        position: Output parameter, target position (encoder counts)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int target_position;
///           lhandprolib_get_target_position(handle, 1, &target_position);
SDK_API int lhandprolib_get_target_position(lhandprolib_handle handle,
                                                 int id, int* position);

/// @brief 设置电机目标速度（默认使用角速度）
///        Set motor target velocity (default uses angular velocity)
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @param velocity 目标速度（单位：度/秒）
///        velocity: Target velocity (unit: degrees/second)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_velocity(handle, 1, 30.0f);
SDK_API int lhandprolib_set_velocity(lhandprolib_handle handle, int id,
                                          float velocity);

/// @brief 获取最近一次设置的电机目标速度（默认使用角速度）
///        Get last set motor target velocity (default uses angular velocity)
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param velocity 输出参数，目标速度（单位：度/秒）
///        velocity: Output parameter, target velocity (unit: degrees/second)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float velocity;
///           lhandprolib_get_velocity(handle, 1, &velocity);
SDK_API int lhandprolib_get_velocity(lhandprolib_handle handle, int id,
                                          float* velocity);

/// @brief 设置目标角速度
///      Set target angular velocity
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @param velocity 目标角速度（单位：度/秒）
///        velocity: Target angular velocity (unit: degrees/second)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 设置电机1目标角速度为30度/秒
///             // Set motor 1 target angular velocity to 30 degrees/second
///           lhandprolib_set_angular_velocity(handle, 1, 30.0f);
SDK_API int lhandprolib_set_angular_velocity(lhandprolib_handle handle,
                                                  int id, float velocity);

/// @brief 获取目标角速度
///      Get target angular velocity
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param velocity 输出参数，目标角速度（单位：度/秒）
///        velocity: Output parameter, target angular velocity (unit: degrees/second)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float target_velocity;
///           lhandprolib_get_angular_velocity(handle, 1, &target_velocity);
SDK_API int lhandprolib_get_angular_velocity(lhandprolib_handle handle,
                                                  int id, float* velocity);

/// @brief 设置目标位置速度
///      Set target position velocity
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @param velocity 目标位置速度（编码器计数/秒）
///        velocity: Target position velocity (encoder counts/second)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 设置电机1目标位置速度为1000
///             // Set motor 1 target position velocity to 1000
///           lhandprolib_set_position_velocity(handle, 1, 1000);
SDK_API int lhandprolib_set_position_velocity(lhandprolib_handle handle,
                                                   int id, int velocity);

/// @brief 获取目标位置速度
///      Get target position velocity
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param velocity 输出参数，目标位置速度（编码器计数/秒）
///        velocity: Output parameter, target position velocity (encoder counts/second)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 获取电机1目标位置速度
///             // Get motor 1 target position velocity
///           int target_pos_velocity;
///           lhandprolib_get_position_velocity(handle, 1, &target_pos_velocity);
SDK_API int lhandprolib_get_position_velocity(lhandprolib_handle handle,
                                                   int id, int* velocity);

/// @brief 设置最大电流
///      Set maximum current
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @param current 最大电流（mA）
///        current: Maximum current (mA)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 设置电机1最大电流为1000mA
///             // Set motor 1 maximum current to 1000mA
///           lhandprolib_set_max_current(handle, 1, 1000);
SDK_API int lhandprolib_set_max_current(lhandprolib_handle handle, int id,
                                             int current);

/// @brief 获取最大电流
///      Get maximum current
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param current 输出参数，最大电流（mA）
///        current: Output parameter, maximum current (mA)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 获取电机1最大电流
///             // Get motor 1 maximum current
///           int max_current;
///           lhandprolib_get_max_current(handle, 1, &max_current);
SDK_API int lhandprolib_get_max_current(lhandprolib_handle handle, int id,
                                             int* current);

/// @brief 电机运动
///      Move motors
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 电机1开始运动
///             // Move motor 1
///           lhandprolib_move_motors(handle, 1);
SDK_API int lhandprolib_move_motors(lhandprolib_handle handle, int id);

/// @brief 电机停止
///      Stop motors
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播
///        id: Motor ID, [0, DOF], 0 means broadcast
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 电机1停止运动
///             // Stop motor 1
///           lhandprolib_stop_motors(handle, 1);
SDK_API int lhandprolib_stop_motors(lhandprolib_handle handle, int id);

/// @brief 执行指定手势
///        Execute specified gesture
/// @param handle 实例句柄
///        handle: Instance handle
/// @param gesture_id 执行的手势ID
///        gesture_id: Gesture ID to execute
/// @param velocity 目标速度，单位：当量/秒
///        velocity: Target velocity, unit: equivalent/second
/// @param current 最大电流，单位：‰
///        current: Maximum current, unit: ‰ (per mille)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 执行手势1，目标速度为20000当量/秒，最大电流为1000‰
///           // Execute gesture 1, velocity 20000 equivalents/second, current 1000‰
///           lhandprolib_play_gesture(handle, 1, 20000, 1000);
SDK_API int lhandprolib_play_gesture(lhandprolib_handle handle,
                                          int gesture_id, int velocity,
                                          int current);

/// @brief 获取电机当前状态
///      Get current motor status
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param status 输出参数，运行状态 C_LST_STOPPED/C_LST_RUNNING等
///        status: Output parameter, running status C_LST_STOPPED/C_LST_RUNNING, etc.
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int motor_status;
///           lhandprolib_get_now_status(handle, 1, &motor_status);
SDK_API int lhandprolib_get_now_status(lhandprolib_handle handle, int id,
                                            int* status);

/// @brief 获取电机当前角度
///      Get current motor angle
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param angle 输出参数，当前角度（单位：度）
///        angle: Output parameter, current angle (unit: degrees)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///      C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float current_angle;
///           lhandprolib_get_now_angle(handle, 1, &current_angle);
SDK_API int lhandprolib_get_now_angle(lhandprolib_handle handle, int id,
                                           float* angle);

/// @brief 获取电机当前位置
///      Get current motor position
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param position 输出参数，当前位置（编码器计数）
///        position: Output parameter, current position (encoder counts)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int current_position;
///           lhandprolib_get_now_position(handle, 1, &current_position);
SDK_API int lhandprolib_get_now_position(lhandprolib_handle handle, int id,
                                              int* position);

/// @brief 获取电机当前速度（默认使用角速度）
///        Get current motor velocity (default uses angular velocity)
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param velocity 输出参数，当前速度（单位：度/秒）
///        velocity: Output parameter, current velocity (unit: degrees/second)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float current_velocity;
///           lhandprolib_get_now_velocity(handle, 1, &current_velocity);
SDK_API int lhandprolib_get_now_velocity(lhandprolib_handle handle, int id,
                                              float* velocity);

/// @brief 获取电机当前角速度
///      Get current motor angular velocity
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param velocity 输出参数，当前角速度（单位：度/秒）
///        velocity: Output parameter, current angular velocity (unit: degrees/second)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float current_velocity;
///           lhandprolib_get_now_angular_velocity(handle, 1, &current_velocity);
SDK_API int lhandprolib_get_now_angular_velocity(lhandprolib_handle handle,
                                                      int id, float* velocity);

/// @brief 获取电机当前位置速度
///      Get current motor position velocity
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param velocity 输出参数，当前位置速度（编码器计数/秒）
///        velocity: Output parameter, current position velocity (encoder counts/second)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int current_pos_velocity;
///           lhandprolib_get_now_position_velocity(handle, 1, &current_pos_velocity);
SDK_API int lhandprolib_get_now_position_velocity(
    lhandprolib_handle handle, int id, int* velocity);

/// @brief 获取电机当前电流
///      Get current motor current
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [1, DOF]
///        id: Motor ID, [1, DOF]
/// @param current 输出参数，当前电流（mA）
///        current: Output parameter, current current (mA)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int current_current;
///           lhandprolib_get_now_current(handle, 1, &current_current);
SDK_API int lhandprolib_get_now_current(lhandprolib_handle handle, int id,
                                             int* current);

/// @brief 同步当前位置到目标位置
///      Sync current position to target position
/// @param handle 实例句柄
///        handle: Instance handle
/// @param id 电机ID, [0, DOF]，0表示广播所有电机
///        id: Motor ID, [0, DOF], 0 means broadcast all motors
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 同步电机1的当前位置到目标位置
///           lhandprolib_sync_position_to_target(handle, 1);
///           // 同步所有电机的当前位置到目标位置
///           lhandprolib_sync_position_to_target(handle, 0);
SDK_API int lhandprolib_sync_position_to_target(lhandprolib_handle handle,
                                                      int id);

/// @brief 设置触觉传感器接收功能
///      Set tactile sensor reception function
/// @param handle 实例句柄
///        handle: Instance handle
/// @param enable 输入参数，是否启用传感器接收功能
///        enable: Input parameter, whether to enable sensor reception function
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_sensor_enable(handle, 1);
SDK_API int lhandprolib_set_sensor_enable(lhandprolib_handle handle,
                                               int enable);

/// @brief 设置传感器数据格式
///      Set sensor data format
/// @param handle 实例句柄
///        handle: Instance handle
/// @param format 格式类型, 默认为0
///        format: Format type, default is 0
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_sensor_data_format(handle, 0);
SDK_API int lhandprolib_set_sensor_data_format(lhandprolib_handle handle,
                                                    int format);

/// @brief 设置传感器的映射顺序
///      Set sensor mapping order
///           按照 大拇指, 食指, 中指, 无名指, 小拇指, 手掌 的顺序传入 sensor_id
///           In the order of thumb, index, middle, ring, little finger, palm, pass sensor_id
/// @param handle 实例句柄
///        handle: Instance handle
/// @param order sensor_id 数组指针, 使用 C_LSS_* 宏
///        order: sensor_id array pointer, use C_LSS_* macros
/// @param size 数组大小，范围 [1, C_LSS_MAX_COUNT]
///        size: Array size, range [1, C_LSS_MAX_COUNT]
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int order[6] = {C_LSS_FINGER_1_1, C_LSS_FINGER_2_1, C_LSS_FINGER_3_1,
///                           C_LSS_FINGER_4_1, C_LSS_FINGER_5_1, C_LSS_HAND_PALM};
///           lhandprolib_set_sensor_order(handle, order, 6);
SDK_API int lhandprolib_set_sensor_order(lhandprolib_handle handle,
                                              const int* order, int size);

/// @brief 获取触觉传感器位置坐标
///      Get tactile sensor position coordinates
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param x_values 输出参数，X坐标数组指针（需用户手动free释放）
///        x_values: Output parameter, X coordinate array pointer (needs manual free by user)
/// @param y_values 输出参数，Y坐标数组指针（需用户手动free释放）
///        y_values: Output parameter, Y coordinate array pointer (needs manual free by user)
/// @param count 输出参数，坐标点数量
///        count: Output parameter, number of coordinate points
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 1. 获取传感器1的位置坐标
///             // 1. Get the position coordinates of sensor 1
///           float* x_values = NULL;
///           float* y_values = NULL;
///           int count = 0;
///           lhandprolib_get_finger_sensor_pos(handle, C_LSS_FINGER_1_1, &x_values, &y_values, &count);
///           // 2. 使用坐标数据
///             // 2. Use the coordinate data
///           for (int i = 0; i < count; i++) {
///               printf("Point %d: (%.2f, %.2f)\n", i, x_values[i], y_values[i]);
///           }
///           // 3. 释放内存
///             // 3. Free memory
///           free(x_values);
///           free(y_values);
SDK_API int lhandprolib_get_finger_sensor_pos(lhandprolib_handle handle,
                                                   int sensor_id,
                                                   float** x_values,
                                                   float** y_values,
                                                   int* count);

/// @brief 获取触觉传感器压力值
///      Get tactile sensor pressure values
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param pressure_list 输出参数，压力值数组指针（需用户手动free释放）
///        pressure_list: Output parameter, pressure value array pointer (needs manual free by user)
/// @param count 输出参数，压力值数量
///        count: Output parameter, number of pressure values
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 1. 获取传感器1的压力值
///             // 1. Get the pressure values of sensor 1
///           float* pressures = NULL;
///           int count = 0;
///           lhandprolib_get_finger_pressure(handle, C_LSS_FINGER_1_1, &pressures, &count);
///           // 2. 使用压力数据
///             // 2. Use the pressure data
///           for (int i = 0; i < count; i++) {
///               printf("Pressure %d: %.2f\n", i, pressures[i]);
///           }
///           // 3. 释放内存
///             // 3. Free memory
///           free(pressures);
SDK_API int lhandprolib_get_finger_pressure(lhandprolib_handle handle,
                                                 int sensor_id,
                                                 float** pressure_list,
                                                 int* count);

/// @brief 重置触觉传感器压力值
///      Reset tactile sensor pressure values
/// @param handle 实例句柄
///        handle: Instance handle
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_set_finger_pressure_reset(handle);
SDK_API int lhandprolib_set_finger_pressure_reset(
    lhandprolib_handle handle);

/// @brief 获取触觉传感器法向力
///      Get tactile sensor normal force
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param normal_force 输出参数，法向力值（范围0.0-1.0）
///        normal_force: Output parameter, normal force value (range 0.0-1.0)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float normal_force;
///           lhandprolib_get_finger_normal_force(handle, C_LSS_FINGER_1_1, &normal_force);
SDK_API int lhandprolib_get_finger_normal_force(lhandprolib_handle handle,
                                                     int sensor_id,
                                                     float* normal_force);

/// @brief 获取触觉传感器法向力数组
///      Get tactile sensor normal force array
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param normal_force_list 输出参数，法向力数组指针（需用户手动free释放）
///        normal_force_list: Output parameter, normal force array pointer (needs manual free by user)
/// @param count 输出参数，数组长度
///        count: Output parameter, array length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 1. 获取传感器1的法向力数组
///           // 1. Get the normal force array of sensor 1
///           float* normal_forces = NULL;
///           int count = 0;
///           lhandprolib_get_finger_normal_force_ex(handle, C_LSS_FINGER_1_1, &normal_forces, &count);
///           // 2. 使用法向力数据
///             // 2. Use the normal force data
///           for (int i = 0; i < count; i++) {
///               printf("Normal force %d: %.2f\n", i, normal_forces[i]);
///           }
///           // 3. 释放内存
///             // 3. Free memory
///           free(normal_forces);
SDK_API int lhandprolib_get_finger_normal_force_ex(
    lhandprolib_handle handle, int sensor_id, float** normal_force_list,
    int* count);

/// @brief 获取触觉传感器切向力
///      Get tactile sensor tangential force
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param tangential_force 输出参数，切向力值
///        tangential_force: Output parameter, tangential force value
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float tangential_force;
///           lhandprolib_get_finger_tangential_force(handle, C_LSS_FINGER_1_1, &tangential_force);
SDK_API int lhandprolib_get_finger_tangential_force(
    lhandprolib_handle handle, int sensor_id, float* tangential_force);

/// @brief 获取触觉传感器切向力数组
///      Get tactile sensor tangential force array
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param tangential_force_list 输出参数，切向力数组指针（需用户手动free释放）
///        tangential_force_list: Output parameter, tangential force array pointer (needs manual free by user)
/// @param count 输出参数，数组长度
///        count: Output parameter, array length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 1. 获取传感器1的切向力数组
///           // 1. Get the tangential force array of sensor 1
///           float* tangential_forces = NULL;
///           int count = 0;
///           lhandprolib_get_finger_tangential_force_ex(handle, C_LSS_FINGER_1_1, &tangential_forces, &count);
///           // 2. 使用切向力数据
///             // 2. Use the tangential force data
///           for (int i = 0; i < count; i++) {
///               printf("Tangential force %d: %.2f\n", i, tangential_forces[i]);
///           }
///           // 3. 释放内存
///             // 3. Free memory
///           free(tangential_forces);
SDK_API int lhandprolib_get_finger_tangential_force_ex(
    lhandprolib_handle handle, int sensor_id, float** tangential_force_list,
    int* count);

/// @brief 获取触觉传感器力方向
///      Get tactile sensor force direction
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param force_direction 输出参数，力方向（角度）
///        force_direction: Output parameter, force direction (angle)
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float force_dir;
///           lhandprolib_get_finger_force_direction(handle, C_LSS_FINGER_1_1, &force_dir);
SDK_API int lhandprolib_get_finger_force_direction(
    lhandprolib_handle handle, int sensor_id, float* force_direction);

/// @brief 获取触觉传感器力方向数组
///      Get tactile sensor force direction array
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param force_direction_list 输出参数，力方向数组指针（需用户手动free释放）
///        force_direction_list: Output parameter, force direction array pointer (needs manual free by user)
/// @param count 输出参数，数组长度
///        count: Output parameter, array length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 1. 获取传感器1的力方向数组
///           // 1. Get the force direction array of sensor 1
///           float* force_directions = NULL;
///           int count = 0;
///           lhandprolib_get_finger_force_direction_ex(handle, C_LSS_FINGER_1_1, &force_directions, &count);
///           // 2. 使用力方向数据
///             // 2. Use the force direction data
///           for (int i = 0; i < count; i++) {
///               printf("Force direction %d: %.2f\n", i, force_directions[i]);
///           }
///           // 3. 释放内存
///             // 3. Free memory
///           free(force_directions);
SDK_API int lhandprolib_get_finger_force_direction_ex(
    lhandprolib_handle handle, int sensor_id, float** force_direction_list,
    int* count);

/// @brief 获取触觉传感器接近度
///      Get tactile sensor proximity
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param proximity 输出参数，接近度值
///        proximity: Output parameter, proximity value
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           float proximity;
///           lhandprolib_get_finger_proximity(handle, C_LSS_FINGER_1_1, &proximity);
SDK_API int lhandprolib_get_finger_proximity(lhandprolib_handle handle,
                                                  int sensor_id,
                                                  float* proximity);

/// @brief 获取触觉传感器接近度数组
///      Get tactile sensor proximity array
/// @param handle 实例句柄
///        handle: Instance handle
/// @param sensor_id 传感器ID，参考C_LSS_*枚举
///        sensor_id: Sensor ID, refer to C_LSS_* enum
/// @param proximity_list 输出参数，接近度数组指针（需用户手动free释放）
///        proximity_list: Output parameter, proximity array pointer (needs manual free by user)
/// @param count 输出参数，数组长度
///        count: Output parameter, array length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           // 1. 获取传感器1的接近度数组
///           // 1. Get the proximity array of sensor 1
///           float* proximities = NULL;
///           int count = 0;
///           lhandprolib_get_finger_proximity_ex(handle, C_LSS_FINGER_1_1, &proximities, &count);
///           // 2. 使用接近度数据
///             // 2. Use the proximity data
///           for (int i = 0; i < count; i++) {
///               printf("Proximity %d: %.2f\n", i, proximities[i]);
///           }
///           // 3. 释放内存
///             // 3. Free memory
///           free(proximities);
SDK_API int lhandprolib_get_finger_proximity_ex(lhandprolib_handle handle,
                                                     int sensor_id,
                                                     float** proximity_list,
                                                     int* count);

/// @brief 开启或关闭日志
///      Turn log on or off
/// @param handle 实例句柄
///      handle: Instance handle
/// @param on true-开启日志，false-关闭日志
///      on: true-turn on log, false-turn off log
/// @param maxsize 日志文件最大大小（KB）
///      maxsize: Maximum log file size (KB)
/// @example
///           // 开启日志，最大5MB
///           lhandprolib_log_on(handle, true, 5000);
SDK_API void lhandprolib_log_on(lhandprolib_handle handle, bool on,
                                     int maxsize);

/// @brief 设置需要打印的发送数据地址数组
///        Set send data address array to be printed
/// @param handle 实例句柄
///        handle: Instance handle
/// @param cmd 需要打印的地址数组，NULL表示全打印
///        cmd: Address array to be printed, NULL means print all
/// @param count 指令数组长度
///        count: Command array length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int send_cmds[] = {0x00, 0x01, 0x0B};
///           lhandprolib_log_send(handle, send_cmds, 3);
SDK_API int lhandprolib_log_send(lhandprolib_handle handle, int* cmd,
                                      int count);

/// @brief 设置需要打印的接收数据地址数组
///        Set receive data address array to be printed
/// @param handle 实例句柄
///        handle: Instance handle
/// @param cmd 需要打印的地址数组，NULL表示全打印
///        cmd: Address array to be printed, NULL means print all
/// @param count 指令数组长度
///        count: Command array length
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           int recv_cmds[] = {0x00, 0x40};
///           lhandprolib_log_recv(handle, recv_cmds, 2);
SDK_API int lhandprolib_log_recv(lhandprolib_handle handle, int* cmd,
                                      int count);

/// @brief 清空日志地址数组
///        Clear log address array
/// @param handle 实例句柄
///        handle: Instance handle
/// @param send 是否清空发送日志地址数组
///        send: Whether to clear send log address array
/// @param recv 是否清空接收日志地址数组
///        recv: Whether to clear receive log address array
/// @example
///           // 清空发送和接收日志地址数组
///           // Clear send and receive log address arrays
///           lhandprolib_log_reset(handle, true, true);
SDK_API void lhandprolib_log_reset(lhandprolib_handle handle, bool send,
                                        bool recv);

/// @brief 保存日志到文件
///      Save log to file
/// @param handle 实例句柄
///      handle: Instance handle
/// @param file_name 日志文件名
///      file_name: Log file name
/// @return C_LER_NONE(0) - 执行成功, 其他 - 参考错误码
///        C_LER_NONE(0) - Execution successful, others - refer to error codes
/// @example
///           lhandprolib_log_save(handle, "hand_log.txt");
SDK_API int lhandprolib_log_save(lhandprolib_handle handle,
                                      const char* file_name);

/// @brief 清除日志
///      Clear log
/// @param handle 实例句柄
///      handle: Instance handle
/// @example
///           lhandprolib_log_clear(handle);
SDK_API void lhandprolib_log_clear(lhandprolib_handle handle);


#ifdef __cplusplus
}
#endif

#endif  // LHANDPROLIB_H
