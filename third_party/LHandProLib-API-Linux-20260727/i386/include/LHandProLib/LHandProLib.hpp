#ifndef LHANDPROLIB_HPP
#define LHANDPROLIB_HPP

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

/// @brief 名字空间 / Namespace
#ifndef SDK_CPP_NAMESPACE
#define SDK_CPP_NAMESPACE lhplib
#endif

#ifndef SDK_CPP_CLASS_NAME
#define SDK_CPP_CLASS_NAME LHandProLib
#endif

#ifndef SDK_CPP_PRIVATE_CLASS_NAME
#define SDK_CPP_PRIVATE_CLASS_NAME LHandProLibPrivate
#endif


namespace lhplib {
/// @brief 错误码枚举 / Error code enum
enum {
  LER_NONE = 0,           ///< 执行成功 / Success
  LER_PARAMETER,          ///< 参数错误 / Parameter error
  LER_KEY_FUNC_UNINIT,    ///< 关键函数未初始化 / Key function uninitialized
  LER_GET_CONFIGURATION,  ///< 读取配置失败 / Failed to read configuration
  LER_DATA_ANOMALY,       ///< 数据异常 / Data anomaly
  LER_COMM_CONNECT,       ///< 通讯连接错误 / Communication connection error
  LER_COMM_SEND,          ///< 通讯发送错误 / Communication send error
  LER_COMM_RECV,          ///< 通讯接收错误 / Communication receive error
  LER_COMM_DATA_FORMAT,  ///< 通讯数据格式错误 / Communication data format error
  LER_INVALID_PATH,      ///< 无效的文件路径 / Invalid file path
  LER_LOG_SAVE_FAIL,     ///< 日志文件保存失败 / Log file save failed
  LER_NOT_HOME,          ///< 没回零错误 / Not homed error
  LER_UNKNOWN = 999,     ///< 未知错误 / Unknown error
};

/// @brief 灵巧手类型枚举 / Dexterous hand type enum
enum {
  LAC_DOF_6 = 0,  ///< 6自由度 / 6 DOF
  LAC_DOF_6_S,    ///< 6自由度(S版本) / 6 DOF (S version)
  LAC_DOF_16,  ///< 16自由度（5指×3DOF + 1独立轴） / 16 DOF (5 fingers × 3DOF + 1 independent axis)
};

/// @brief 通讯类型枚举 / Communication type enum
enum {
  LCN_ECAT = 0,  ///< EtherCAT
  LCN_CANFD,     ///< CANFD
  LCN_RS485,     ///< RS485
  LCN_CAN,       ///< 标准CAN / Standard CAN
};

/// @brief 灵巧手方向 / Hand direction
enum {
  LDR_HAND_RIGHT = 0,  ///< 右手 / Right hand
  LDR_HAND_LEFT        ///< 左手 / Left hand
};

/// @brief 控制模式枚举 / Control mode enum
enum {
  LCM_POSITION = 0,  ///< 位置控制 / Position control
  LCM_VELOCITY,      ///< 速度控制 / Velocity control
  LCM_TORQUE,        ///< 力矩控制 / Torque control
  LCM_VEL_TOR,       ///< 速度+力矩混合控制 / Velocity+Torque hybrid control
  LCM_POS_TOR,       ///< 位置+力矩混合控制 / Position+Torque hybrid control
  LCM_HOME,          ///< 回零 / Homing
};


/// @brief 运行状态枚举 / Running status enum
enum {
  LST_STOPPED = 0,  ///< 正常停止状态 / Normal stop status
  LST_RUNNING,      ///< 正常运行状态 / Normal running status
  LST_ALARM,        ///< 报警停止状态 / Alarm stop status
  LST_POS_LIMIT,    ///< 正限位状态 / Positive limit status
  LST_NEG_LIMIT,    ///< 负限位状态 / Negative limit status
  LST_BOTH_LIMIT,   ///< 正负限位状态 / Both limit status
  LST_EMG_STOP,     ///< 急停状态 / Emergency stop status
  LST_HOMING,       ///< 回零运行状态 / Homing running status
};

/// @brief 报警类型枚举 / Alarm type enum
enum {
  LAM_NULL = 0,   ///< 无报警 / No alarm
  LAM_POS_ERR,    ///< 位置超差 / Position error
  LAM_OVER_SPD,   ///< 超速 / Over speed
  LAM_OVER_CUR,   ///< 过流 / Over current
  LAM_OVER_LOAD,  ///< 过载 / Over load
  LAM_OVER_VOL,   ///< 过压 / Over voltage
  LAM_UNDER_VOL,  ///< 欠压 / Under voltage
  LAM_ENC_ERR,    ///< 编码器错误 / Encoder error
  LAM_STALL,      ///< 堵转 / Stall
  LAM_OTHER,      ///< 其他报警 / Other alarm
};

/// @brief 6自由度灵巧手关节枚举（LAC_DOF_6，LAC_DOF_6_S）
///        6-DOF dexterous hand joint enum (LAC_DOF_6, LAC_DOF_6_S)
enum {
  LMI6_ALL_JOINTS = 0,   ///< 全部关节 / All joints
  LMI6_THUMB_ABDUCTION,  ///< 大拇指侧摆 / Thumb abduction
  LMI6_THUMB_FLEXION,    ///< 大拇指弯曲 / Thumb flexion
  LMI6_INDEX_FLEXION,    ///< 食指弯曲 / Index finger flexion
  LMI6_MIDDLE_FLEXION,   ///< 中指弯曲 / Middle finger flexion
  LMI6_RING_FLEXION,     ///< 无名指弯曲 / Ring finger flexion
  LMI6_PINKY_FLEXION,    ///< 小拇指弯曲 / Pinky finger flexion
};

/// @brief 16自由度灵巧手关节枚举（LAC_DOF_16）
///        16-DOF dexterous hand joint enum (LAC_DOF_16)
enum {
  LMI16_ALL_JOINTS = 0,  ///< 全部关节 / All joints
  LMI16_THUMB_LATERAL_EX,  ///< 大拇指加强侧摆（独立轴）/ Thumb lateral extended (independent axis)
  LMI16_THUMB_LATERAL,   ///< 大拇指侧摆 / Thumb lateral (abduction)
  LMI16_THUMB_PROXIMAL,  ///< 大拇指近节 / Thumb proximal
  LMI16_THUMB_DISTAL,    ///< 大拇指远节 / Thumb distal

  LMI16_INDEX_LATERAL,   ///< 食指侧摆 / Index lateral
  LMI16_INDEX_PROXIMAL,  ///< 食指近节 / Index proximal
  LMI16_INDEX_DISTAL,    ///< 食指远节 / Index distal

  LMI16_MIDDLE_LATERAL,   ///< 中指侧摆 / Middle lateral
  LMI16_MIDDLE_PROXIMAL,  ///< 中指近节 / Middle proximal
  LMI16_MIDDLE_DISTAL,    ///< 中指远节 / Middle distal

  LMI16_RING_LATERAL,   ///< 无名指侧摆 / Ring lateral
  LMI16_RING_PROXIMAL,  ///< 无名指近节 / Ring proximal
  LMI16_RING_DISTAL,    ///< 无名指远节 / Ring distal

  LMI16_PINKY_LATERAL,   ///< 小拇指侧摆 / Pinky lateral
  LMI16_PINKY_PROXIMAL,  ///< 小拇指近节 / Pinky proximal
  LMI16_PINKY_DISTAL,    ///< 小拇指远节 / Pinky distal
};

/// @brief 传感器id枚举 / Sensor ID enum
enum {
  LSS_FINGER_1_1 = 1,  ///< 大拇指指尖 / Thumb tip
  LSS_FINGER_1_2,      ///< 大拇指指腹 / Thumb pad
  LSS_FINGER_2_1,      ///< 食指指尖 / Index finger tip
  LSS_FINGER_2_2,      ///< 食指指腹 / Index finger pad
  LSS_FINGER_3_1,      ///< 中指指尖 / Middle finger tip
  LSS_FINGER_3_2,      ///< 中指指腹 / Middle finger pad
  LSS_FINGER_4_1,      ///< 无名指指尖 / Ring finger tip
  LSS_FINGER_4_2,      ///< 无名指指腹 / Ring finger pad
  LSS_FINGER_5_1,      ///< 小拇指指尖 / Little finger tip
  LSS_FINGER_5_2,      ///< 小拇指指腹 / Little finger pad
  LSS_HAND_PALM,       ///< 手掌 / Palm
  LSS_MAX_COUNT,       ///< 最大传感器数量 / Maximum sensor count
};

/// @brief 函数指针类型定义 / Function pointer type definition

/// @brief 日志回调 / Log callback
/// @param const char* 日志内容 / Log message
typedef void (*LogAddCallback)(const char*);

/// @brief EtherCAT 发送数据回调 / EtherCAT send data callback
/// @param const unsigned char* 数据指针 / Data pointer
/// @param unsigned int 数据长度 / Data size
/// @return true-成功 / true on success
typedef bool (*ECSendDataCallback)(const unsigned char*, unsigned int);

/// @brief CANFD 发送数据回调 / CANFD send data callback
/// @param unsigned int CAN ID
/// @param const unsigned char* 数据指针 / Data pointer
/// @param unsigned int 数据长度 / Data size
/// @param int 是否扩展帧 / Extended frame flag
/// @return true-成功 / true on success
typedef bool (*CANFDSendDataCallback)(unsigned int, const unsigned char*,
                                      unsigned int, int);

/// @brief RS485 发送数据回调 / RS485 send data callback
/// @param const unsigned char* 数据指针 / Data pointer
/// @param unsigned int 数据长度 / Data size
/// @return true-成功 / true on success
typedef bool (*RS485SendDataCallback)(const unsigned char*, unsigned int);

/// @brief 标准CAN 发送数据回调 / Standard CAN send data callback
/// @param unsigned int CAN ID
/// @param const unsigned char* 数据指针 / Data pointer
/// @param unsigned int 数据长度 / Data size
/// @return true-成功 / true on success
typedef bool (*CANSendDataCallback)(unsigned int, const unsigned char*,
                                    unsigned int);

#define L_DECLARE_PRIVATE(Class) \
  Class##Private* d_ptr;         \
  friend class Class##Private;

/// @brief 前置声明 / Forward declaration
class LHandProLibPrivate;
class RS485SharedBusPrivate;

/// @brief 一条物理 RS485 总线的共享调度对象
///        Shared scheduler for one physical RS485 bus
///
/// 多个 LHandProLib 实例可以绑定到同一个 RS485SharedBus。总线对象在 SDK
/// 内部统一处理发送时序、后台轮询、分包/粘包、CRC 校验和按节点分发。
/// 应用程序只负责同步写串口，并将收到的原始字节片段送回该对象。
/// Multiple LHandProLib instances may bind to one RS485SharedBus. The SDK keeps
/// scheduling, polling, framing, CRC validation, and node dispatch internal.
class SDK_API RS485SharedBus {
  L_DECLARE_PRIVATE(RS485SharedBus)
  friend class LHandProLib;

 public:
  RS485SharedBus();
  ~RS485SharedBus();

  RS485SharedBus(const RS485SharedBus&) = delete;
  RS485SharedBus& operator=(const RS485SharedBus&) = delete;

  /// @brief 设置物理串口发送回调
  ///        Set the physical serial-port send callback
  void set_send_callback(RS485SendDataCallback callback);

  /// @brief 设置 std::function 形式的物理串口发送回调
  ///        Set a std::function based physical serial-port send callback
  /// @param callback_impl 指向
  ///        std::function<bool(const unsigned char*, unsigned int)> 的指针
  ///        Pointer to std::function<bool(const unsigned char*, unsigned int)>
  void set_send_callback_ex(void* callback_impl);

  /// @brief 输入物理串口收到的原始字节片段
  ///        Feed a raw byte chunk received from the physical serial port
  int set_receive_data(const unsigned char* data_ptr, int data_size);

  /// @brief 停止总线工作线程并取消待发送数据
  ///        Stop the bus worker and cancel pending transmissions
  void close();
};

/// @brief LHandProLib类 / LHandProLib class
class SDK_API LHandProLib {
  L_DECLARE_PRIVATE(LHandProLib)
 public:
  /// @brief 构造函数 / Constructor
  LHandProLib();

  /// @brief 析构函数 / Destructor
  ~LHandProLib();

  /// @brief 初始化灵巧手驱动程序
  ///        Initialize dexterous hand driver
  /// @param mode 通讯模式 LCN_ECAT/LCN_CANFD/LCN_485
  ///        mode: Communication mode LCN_ECAT/LCN_CANFD/LCN_485
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           [注意] 在调用initial之前, 请先按下方流程进行
  ///           [Note] Before calling initial, please follow the steps below
  ///           1. 自行初始化通讯对象, 例如EtherCAT
  ///           1. Initialize communication object by yourself, such as EtherCAT
  ///           2. 对通讯对象进行连接
  ///           2. Connect the communication object
  ///           3. 进行发送数据[回调]函数的处理, 详见set_send_rpdo_callback(ex)
  ///           3. Handle send data [callback] function, see set_send_rpdo_callback(ex)
  ///           4. 进行接收数据[回调]函数的处理, 详见set_tpdo_data_decode
  ///           4. Handle receive data [callback] function, see set_tpdo_data_decode
  ///           5. 对库进行初始化initial(LCN_ECAT)
  ///           5. Initialize the library with initial(LCN_ECAT)
  int initial(int mode);

  /// @brief 初始化灵巧手驱动程序(指定节点ID)
  ///        Initialize dexterous hand driver (specify node ID)
  /// @param mode 通讯模式 LCN_CANFD/LCN_RS485
  ///        mode: Communication mode LCN_CANFD/LCN_RS485
  /// @param node_id CANFD或RS485的节点ID
  ///        node_id: CANFD or RS485 node ID
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  int initial(int mode, unsigned int node_id);

  /// @brief 关闭灵巧手驱动程序
  ///        Close dexterous hand driver
  /// @example
  ///           sdk_handle->close();
  ///           sdk_handle->close();
  void close();

  /// @brief 启动后台监控线程（轮询电机/传感器状态）
  ///        Start background monitor thread (poll motor/sensor status)
  void start_monitor();

  /// @brief 停止后台监控线程
  ///        Stop background monitor thread
  void stop_monitor();

  /// @brief 设置 dry-run 模式，开启后命令数据正常生成但不通过回调发送
  ///        Set dry-run mode; when enabled, command data is generated normally but not sent via callback
  /// @param enable true - 开启 dry-run, false - 关闭
  ///        enable: true - enable dry-run, false - disable
  void set_dry_run_mode(bool enable);

  /// @brief 获取当前 dry-run 模式状态
  ///        Get current dry-run mode state
  /// @return true - dry-run 已开启, false - 已关闭
  ///         true - dry-run enabled, false - disabled
  bool get_dry_run_mode() const;

  /// @brief 发送数据回调函数(EtherCAT)
  ///        Send data callback function (EtherCAT)
  /// @param callback 函数指针
  ///        callback: Function pointer
  /// @example
  ///           // 1. 定义EtherCAT主站对象
  ///           // 1. Define EtherCAT master object
  ///           static auto ec_master_ = std::make_shared<EthercatMaster>();
  ///           // 2. 处理EtherCAT发送数据的回调
  ///           // 2. Handle EtherCAT send data callback
  ///           static int slave_id = 1; // 选择目标从站 / Select target slave
  ///           sdk_handle->set_send_rpdo_callback(
  ///               [](const unsigned char* data, unsigned int size) {
  ///                 return ec_master_->setSlaveOutputs(slave_id, data, size);
  ///               });
  void set_send_rpdo_callback(ECSendDataCallback callback);

  /// @brief 发送数据回调函数(EtherCAT)(使用时需使用std::function包装)
  ///        Send data callback function (EtherCAT) (must use std::function wrapper when using)
  /// @param callback_impl 函数指针(必须保证该 std::function 对象生命周期长于 LHandProLib 实例)
  ///        callback_impl: Function pointer (must ensure the std::function object lifetime is longer than LHandProLib instance)
  /// @example
  ///           // 1. 定义EtherCAT主站对象
  ///           // 1. Define EtherCAT master object
  ///           auto ec_master_ = std::make_shared<EthercatMaster>();
  ///           // 2. EtherCAT的发送函数
  ///           // 2. EtherCAT send function
  ///           int slave_id = 1; // 选择目标从站 / Select target slave
  ///           auto send_func = [ec_master_, slave_id](const unsigned char* data,
  ///                                    unsigned int size) {
  ///             return ec_master_->setSlaveOutputs(slave_id, data, size);
  ///           };
  ///           // 3. 使用std::function包装(必须保证该 std::function 对象生命周期长于 LHandProLib 实例)
  ///           // 3. Use std::function wrapper (must ensure the std::function
  ///           // object lifetime is longer than LHandProLib instance)
  ///           std::function<bool(const unsigned char*, unsigned int)> func = send_func;
  ///           // 4. 处理EtherCAT发送数据的回调
  ///           // 4. Handle EtherCAT send data callback
  ///           sdk_handle->set_send_rpdo_callback_ex(&func);
  void set_send_rpdo_callback_ex(void* callback_impl);

  /// @brief 获取预发送区的 RPDO 数据,如设置了set_send_rpdo_callback(ex)回调,则会同时调用发送
  ///        Get RPDO data in pre-send area, if set_send_rpdo_callback(ex) callback is set, it will call send simultaneously
  /// @param data_ptr 返回的数据指针（nullptr 时仅获取长度）
  ///        data_ptr: Returned data pointer (only get length when nullptr)
  /// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
  ///        io_size: Input indicates buffer size, output returns actual data size
  /// @return 0 - 成功，其他 - 错误码
  ///        0 - Success, others - Error code
  /// @example
  ///           // 1. 第一次调用获取数据长度
  ///           // 1. First call to get data length
  ///           int size = 0;
  ///           sdk_handle->get_pre_send_rpdo_data(nullptr, &size);
  ///           // 2. 第二次调用获取实际预发送区的字节数组
  ///           // 2. Second call to get actual pre-send area byte array
  ///           std::vector<unsigned char> rpdo_data(size);
  ///           sdk_handle->get_pre_send_rpdo_data(rpdo_data.data(), &size);
  int get_pre_send_rpdo_data(unsigned char* data_ptr, int* io_size);

  /// @brief 解码传入的 TPDO 数据
  ///        Decode incoming TPDO data
  /// @param data_ptr 待解码的数据指针
  ///        data_ptr: Data pointer to be decoded
  /// @param data_size 数据长度
  ///        data_size: Data length
  /// @return 0 成功，其他错误码
  ///        0 - Success, others - Error code
  /// @example
  ///           // 1. 使用SDK 对 EtherCAT 获取上来的TPDO数据进行解码
  ///           // 1. Use SDK to decode TPDO data obtained from EtherCAT
  ///           std::vector<unsigned char> tpdo_data;
  ///           sdk_handle->set_tpdo_data_decode(tpdo_data.data(), tpdo_data.size());
  ///           // 2. 解码后则可以使用SDK的接口获取目标数据,如get_now_angle,get_finger_pressure等
  ///           // 2. After decoding, you can use SDK interfaces to get target data, such as get_now_angle, get_finger_pressure, etc.
  int set_tpdo_data_decode(const unsigned char* data_ptr, int data_size);

  /// @brief 发送数据回调函数(CANFD)
  ///        Send data callback function (CANFD)
  /// @param callback 函数指针
  ///        callback: Function pointer
  /// @example
  ///           // 1. 定义CANFD主站对象
  ///           // 1. Define CANFD master object
  ///           auto canfd_master_ = std::make_shared<CANFDMaster>();
  ///           // 2. 处理CANFD发送数据的回调
  ///           // 2. Handle CANFD send data callback
  ///           sdk_handle->set_send_canfd_callback(
  ///               [](unsigned int id, const unsigned char* data, unsigned int size) {
  ///                 return canfd_master_->sendData(id, data, size);
  ///               });
  void set_send_canfd_callback(CANFDSendDataCallback callback);

  /// @brief 发送数据回调函数(CANFD)(使用时需使用std::function包装)
  ///        Send data callback function (CANFD) (must use std::function wrapper when using)
  /// @param callback_impl 函数指针(必须保证该 std::function 对象生命周期长于 LHandProLib 实例)
  ///        callback_impl: Function pointer (must ensure the std::function object lifetime is longer than LHandProLib instance)
  /// @example
  ///           // 1. 定义CANFD主站对象
  ///           // 1. Define CANFD master object
  ///           auto canfd_master_ = std::make_shared<CANFDMaster>();
  ///           // 2. CANFD的发送函数
  ///           // 2. CANFD send function
  ///           auto send_func = [canfd_master_](const unsigned char* data,
  ///           unsigned int size) {
  ///             return canfd_master_->sendData(0x500 + 1, data, size);
  ///           };
  ///           // 3. 使用std::function包装(必须保证该 std::function 对象生命周期长于 LHandProLib 实例)
  ///           // 3. Use std::function wrapper (must ensure the std::function object lifetime is longer than LHandProLib instance)
  ///           std::function<bool(const unsigned char*, unsigned int)> func = send_func;
  ///           // 4. 处理CANFD发送数据的回调
  ///           // 4. Handle CANFD send data callback
  ///           sdk_handle->set_send_canfd_callback_ex(&func);
  void set_send_canfd_callback_ex(void* callback_impl);

  /// @brief 获取预发送区的 CANFD 数据,如设置了set_send_canfd_callback(ex)回调,则会同时调用发送
  ///        Get CANFD data in pre-send area, if set_send_canfd_callback(ex) callback is set, it will call send simultaneously
  /// @param data_ptr 返回的数据指针（nullptr 时仅获取长度）
  ///        data_ptr: Returned data pointer (only get length when nullptr)
  /// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
  ///        io_size: Input indicates buffer size, output returns actual data size
  /// @return 0 - 成功，其他 - 错误码
  ///        0 - Success, others - Error code
  /// @example
  ///           // 1. 第一次调用获取数据长度
  ///           // 1. First call to get data length
  ///           int size = 0;
  ///           sdk_handle->get_pre_send_canfd_data(nullptr, &size);
  ///           // 2. 第二次调用获取实际预发送区的字节数组
  ///           // 2. Second call to get actual pre-send area byte array
  ///           std::vector<unsigned char> canfd_data(size);
  ///           sdk_handle->get_pre_send_canfd_data(canfd_data.data(), &size);
  int get_pre_send_canfd_data(unsigned char* data_ptr, int* io_size);

  /// @brief 解码传入的 CANFD 数据
  ///        Decode incoming CANFD data
  /// @param id CANFD ID
  ///        id: CANFD ID
  /// @param data_ptr 待解码的数据指针
  ///        data_ptr: Data pointer to be decoded
  /// @param data_size 数据长度
  ///        data_size: Data length
  /// @return 0 成功，其他错误码
  ///        0 - Success, others - Error code
  /// @example
  ///           // 1. 使用SDK 对 CANFD 反馈的数据进行解码
  ///           // 1. Use SDK to decode CANFD feedback data
  ///           std::vector<unsigned char> canfd_data;
  ///           sdk_handle->set_canfd_data_decode(canfd_data.data(),
  ///           canfd_data.size());
  ///           // 2. 解码后则可以使用SDK的接口获取目标数据,如get_now_angle,get_finger_pressure等
  ///           // 2. After decoding, you can use SDK interfaces to get target data, such as get_now_angle, get_finger_pressure, etc.
  int set_canfd_data_decode(unsigned int id, const unsigned char* data_ptr,
                            int data_size);

  /// @brief 发送数据回调函数(RS485)
  ///        Send data callback function (RS485)
  ///
  /// @note 多设备共用一条物理总线时，应先调用 set_rs485_shared_bus()，并在
  ///       RS485SharedBus 上设置发送回调。本接口保留用于旧的单设备用法。
  ///       For multiple devices on one physical bus, call
  ///       set_rs485_shared_bus() first and configure the callback on
  ///       RS485SharedBus. This API remains for legacy single-device usage.
  /// @param callback 函数指针
  ///        callback: Function pointer
  /// @example
  ///           // 1. Define RS485 master object
  ///           auto rs485_master_ = std::make_shared<RS485Master>();
  ///           // 2. Handle RS485 send data callback
  ///           sdk_handle->set_send_rs485_callback(
  ///               [](const unsigned char* data, unsigned int size) {
  ///                 return rs485_master_->sendData(data, size);
  ///               });
  void set_send_rs485_callback(RS485SendDataCallback callback);

  /// @brief 发送数据回调函数(RS485)(使用时需使用std::function包装)
  ///        Send data callback function (RS485) (must use std::function wrapper when using)
  /// @param callback_impl 函数指针(必须保证该 std::function 对象生命周期长于 LHandProLib 实例)
  ///        callback_impl: Function pointer (must ensure the std::function object lifetime is longer than LHandProLib instance)
  void set_send_rs485_callback_ex(void* callback_impl);

  /// @brief 绑定共享 RS485 总线；必须在 initial(LCN_RS485, node_id) 前调用
  ///        Bind a shared RS485 bus; call before initial(LCN_RS485, node_id)
  /// @param bus 总线对象指针。传入 nullptr 恢复旧的单设备隐式总线模式
  ///        bus: Bus object pointer. Pass nullptr to use the legacy implicit
  ///        single-device bus
  void set_rs485_shared_bus(RS485SharedBus* bus);

  /// @brief 获取预发送区的 RS485 完整帧数据（含地址头 + CRC），如设置了set_send_rs485_callback(ex)回调,则会同时调用发送
  ///        Get RS485 complete framed data (with address header + CRC) in pre-send area, if set_send_rs485_callback(ex) callback is set, it will call send simultaneously
  /// @param data_ptr 返回的数据指针（nullptr 时仅获取长度）
  ///        data_ptr: Returned data pointer (only get length when nullptr)
  /// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
  ///        io_size: Input indicates buffer size, output returns actual data size
  /// @return 0 - 成功，其他 - 错误码
  ///        0 - Success, others - Error code
  /// @example
  ///           // 1. First call to get data length
  ///           int size = 0;
  ///           sdk_handle->get_pre_send_rs485_data(nullptr, &size);
  ///           // 2. Second call to get actual pre-send area byte array
  ///           std::vector<unsigned char> rs485_data(size);
  ///           sdk_handle->get_pre_send_rs485_data(rs485_data.data(), &size);
  int get_pre_send_rs485_data(unsigned char* data_ptr, int* io_size);

  /// @brief 解码传入的 RS485 串口数据片段，SDK 内部会处理半包/粘包
  ///        Decode incoming RS485 serial chunks; the SDK handles partial and joined frames internally
  /// @param data_ptr 待解码的数据指针，可以是任意长度的串口接收片段
  ///        data_ptr: Data pointer to be decoded; may be any serial receive chunk
  /// @param data_size 数据长度
  ///        data_size: Data length
  /// @return 0 成功，其他错误码
  ///        0 - Success, others - Error code
  /// @example
  ///           // 1. Feed raw serial bytes to the SDK
  ///           std::vector<unsigned char> rs485_data;
  ///           sdk_handle->set_rs485_data_decode(rs485_data.data(),
  ///           rs485_data.size());
  ///           // 2. After decoding, you can use SDK interfaces to get target data
  int set_rs485_data_decode(const unsigned char* data_ptr, int data_size);

  /// @brief 发送数据回调函数(标准CAN，用于LCN_CAN模式)
  ///        Send data callback function (Standard CAN, for LCN_CAN mode)
  /// @param callback 函数指针
  void set_send_can_callback(CANSendDataCallback callback);

  /// @brief 发送数据回调函数(标准CAN)(使用时需使用std::function包装)
  ///        Send data callback function (Standard CAN) (must use std::function wrapper)
  /// @param callback_impl 函数指针(必须保证该 std::function 对象生命周期长于 LHandProLib 实例)
  void set_send_can_callback_ex(void* callback_impl);

  /// @brief 解码传入的标准CAN数据（LCN_CAN模式下由用户调用）
  ///        Decode incoming standard CAN data (called by user in LCN_CAN mode)
  /// @param id CAN帧ID
  /// @param data_ptr 待解码的数据指针
  /// @param data_size 数据长度
  /// @return 0 成功，其他错误码
  int set_can_data_decode(unsigned int id, const unsigned char* data_ptr,
                          int data_size);

  /// @brief 获取预发送区的标准CAN数据（dry-run捕获），每帧11字节: CAN_ID(4B) + data(7B)
  ///        Get standard CAN data in pre-send area (dry-run capture), each frame 11 bytes: CAN_ID(4B) + data(7B)
  /// @param data_ptr 返回的数据指针（nullptr 时仅获取长度）
  ///        data_ptr: Returned data pointer (only get length when nullptr)
  /// @param io_size 输入时表示缓冲区大小，输出时返回实际数据大小
  ///        io_size: Input indicates buffer size, output returns actual data size
  /// @return 0 - 成功，其他 - 错误码
  ///        0 - Success, others - Error code
  int get_pre_send_can_data(unsigned char* data_ptr, int* io_size);

  /// @brief 获取固件版本号
  ///        Get firmware version
  /// @param version 返回的固件版本号（例如：0.32 表示版本 0.32）
  ///        version: Returned firmware version (e.g., 0.32 means version 0.32)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float ver;
  ///           sdk_handle->get_firmware_version(&ver);
  int get_firmware_version(float* version);

  /// @brief 获取灵巧手SN码
  ///        Get dexterous hand serial number
  /// @param sn 输出缓冲区
  ///        sn: Output buffer
  /// @param size 缓冲区大小
  ///        size: Buffer size
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           char buf[64];
  ///           sdk_handle->get_serial_number(buf, sizeof(buf));
  ///           std::string sn(buf);
  int get_serial_number(char* sn, int size);

  /// @brief 获取当前灵巧手自由度数量
  ///        Get current dexterous hand degree of freedom
  /// @param total 自由度总数量, 即关节数量
  ///        total: Total number of degrees of freedom, i.e., joint count
  /// @param active 主动自由度数量, 即电机数量
  ///        active: Active degrees of freedom, i.e., motor count
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int total_dof, active_dof;
  ///           sdk_handle->get_dof(&total_dof, &active_dof);
  int get_dof(int* total, int* active);

  /// @brief 设置灵巧手类型,根据枚举进行设置 默认 LAC_DOF_6
  ///        Set dexterous hand type, set according to enum, default LAC_DOF_6
  /// @param hand_type 灵巧手类型
  ///        hand_type: Dexterous hand type
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_hand_type(LAC_DOF_6);
  int set_hand_type(int hand_type);

  /// @brief 获取当前灵巧手类型
  ///        Get current dexterous hand type
  /// @param hand_type 灵巧手类型
  ///        hand_type: Dexterous hand type
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int hand_type;
  ///           sdk_handle->get_hand_type(&hand_type);
  int get_hand_type(int* hand_type);

  /// @brief 设置手的方向(默认即为右手LDR_HAND_RIGHT)
  ///        Set hand direction (default is right hand LDR_HAND_RIGHT)
  /// @param dir LDR_HAND_RIGHT/LDR_HAND_LEFT
  ///        dir: LDR_HAND_RIGHT/LDR_HAND_LEFT
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_hand_direction(LDR_HAND_RIGHT);
  int set_hand_direction(int dir);

  /// @brief 获取手的方向
  ///        Get hand direction
  /// @param dir LDR_HAND_RIGHT/LDR_HAND_LEFT
  ///        dir: LDR_HAND_RIGHT/LDR_HAND_LEFT
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int hand_dir;
  ///           sdk_handle->get_hand_direction(&hand_dir);
  int get_hand_direction(int* dir);

  /// @brief 设置电机控制模式
  ///        Set motor control mode
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///        id: Motor ID, [0, DOF=degree of freedom], 0 means broadcast,
  ///       1~DOF表示单个ID设置
  ///        1~DOF means single ID setting
  /// @param mode 控制模式 0:位置控制 1:速度控制 2:力矩控制
  ///        mode: Control mode 0:Position control 1:Velocity control 2:Torque control
  ///       3:速度+力矩混合控制 4:位置+力矩混合控制 5:回零
  ///        3:Velocity+Torque hybrid control 4:Position+Torque hybrid control 5:Homing
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_control_mode(1, LCM_POSITION);
  int set_control_mode(int id, int mode = LCM_POSITION);

  /// @brief 获取电机控制模式
  ///        Get motor control mode
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param mode 控制模式 0:位置控制 1:速度控制 2:力矩控制
  ///        mode: Control mode 0:Position control 1:Velocity control 2:Torque control
  ///       3:速度+力矩混合控制 4:位置+力矩混合控制 5:回零
  ///        3:Velocity+Torque hybrid control 4:Position+Torque hybrid control 5:Homing
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int control_mode;
  ///           sdk_handle->get_control_mode(1, &control_mode);
  int get_control_mode(int id, int* mode);

  /// @brief 设置安全电流使能
  ///        Set safe current enable
  /// @param enable 安全电流使能 0:去使能 1:使能
  ///        enable: Safe current enable 0:Disable 1:Enable
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_safe_current_enable(1);
  int set_safe_current_enable(int enable);

  /// @brief 获取安全电流使能状态
  ///        Get safe current enable state
  /// @param enable 安全电流使能状态 0:去使能 1:使能
  ///        enable: Safe current enable state 0:Disable 1:Enable
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int enable = 0;
  ///           sdk_handle->get_safe_current_enable(&enable);
  int get_safe_current_enable(int* enable);

  /// @brief 设置回零电流
  ///        Set homing current
  /// @param current 回零电流值, 单位 %(百分比)
  ///        current: Homing current value, unit %(percentage)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_home_current(100);
  int set_home_current(int current);

  /// @brief 获取回零电流
  ///        Get homing current
  /// @param current 回零电流值, 单位 %(百分比)
  ///        current: Homing current value, unit %(percentage)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int current = 0;
  ///           sdk_handle->get_home_current(&current);
  int get_home_current(int* current);

  /// @brief 设置CAN节点号
  ///        Set CAN node ID
  /// @param node_id 节点号
  ///        node_id: Node ID
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_can_node_id(1);
  int set_can_node_id(int node_id);

  /// @brief 获取CAN节点号
  ///        Get CAN node ID
  /// @param node_id 返回的节点号
  ///        node_id: Returned node ID
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int node_id = 0;
  ///           sdk_handle->get_can_node_id(&node_id);
  int get_can_node_id(int* node_id);

  /// @brief 设置CANFD仲裁段波特率
  ///        Set CANFD arbitration segment baud rate
  /// @param baudrate 波特率值
  ///        baudrate: Baud rate value
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_canfd_arb_baudrate(1000000);
  int set_canfd_arb_baudrate(int baudrate);

  /// @brief 获取CANFD仲裁段波特率
  ///        Get CANFD arbitration segment baud rate
  /// @param baudrate 返回的波特率值
  ///        baudrate: Returned baud rate value
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int baudrate = 0;
  ///           sdk_handle->get_canfd_arb_baudrate(&baudrate);
  int get_canfd_arb_baudrate(int* baudrate);

  /// @brief 设置CANFD数据段波特率
  ///        Set CANFD data segment baud rate
  /// @param baudrate 波特率值
  ///        baudrate: Baud rate value
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_canfd_data_baudrate(5000000);
  int set_canfd_data_baudrate(int baudrate);

  /// @brief 获取CANFD数据段波特率
  ///        Get CANFD data segment baud rate
  /// @param baudrate 返回的波特率值
  ///        baudrate: Returned baud rate value
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int baudrate = 0;
  ///           sdk_handle->get_canfd_data_baudrate(&baudrate);
  int get_canfd_data_baudrate(int* baudrate);

  /// @brief 设置RS485节点号
  ///        Set RS485 node ID
  /// @param node_id 节点号
  ///        node_id: Node ID
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_rs485_node_id(1);
  int set_rs485_node_id(int node_id);

  /// @brief 获取RS485节点号
  ///        Get RS485 node ID
  /// @param node_id 返回的节点号
  ///        node_id: Returned node ID
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int node_id = 0;
  ///           sdk_handle->get_rs485_node_id(&node_id);
  int get_rs485_node_id(int* node_id);

  /// @brief 设置RS485波特率
  ///        Set RS485 baud rate
  /// @param baudrate 波特率值
  ///        baudrate: Baud rate value
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_rs485_baudrate(500000);
  int set_rs485_baudrate(int baudrate);

  /// @brief 获取RS485波特率
  ///        Get RS485 baud rate
  /// @param baudrate 返回的波特率值
  ///        baudrate: Returned baud rate value
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int baudrate = 0;
  ///           sdk_handle->get_rs485_baudrate(&baudrate);
  int get_rs485_baudrate(int* baudrate);

  /// @brief 设置电机使能/禁止
  ///        Set motor enable/disable
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///        id: Motor ID, [0, DOF=degree of freedom], 0 means broadcast,
  ///       1~DOF表示单个ID设置
  ///        1~DOF means single ID setting
  /// @param enable 使能/禁止 0:禁止 1:使能
  ///        enable: Enable/disable 0:Disable 1:Enable
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 使能电机1
  ///           // Enable motor 1
  ///           sdk_handle->set_enable(1, 1);
  int set_enable(int id, int enable);

  /// @brief 获取电机使能/禁止
  ///        Get motor enable/disable status
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param enable 使能/禁止 0:禁止 1:使能
  ///        enable: Enable/disable 0:Disable 1:Enable
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int is_enabled;
  ///           sdk_handle->get_enable(1, &is_enabled);
  int get_enable(int id, int* enable);

  /// @brief 获取电机到位信号
  ///        Get motor position reached signal
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param reached 是否到位, 1:到位 0:运动中
  ///        reached: Whether reached, 1:Reached 0:Moving
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int is_reached;
  ///           sdk_handle->get_position_reached(1, &is_reached);
  int get_position_reached(int id, int* reached);

  /// @brief 获取力矩到位信号
  ///        Get torque reached signal
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param reached 是否到位, 1:到位 0:没到位
  ///        reached: Whether reached, 1:Reached 0:Not reached
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int torque_reached;
  ///           sdk_handle->get_torque_reached(1, &torque_reached);
  int get_torque_reached(int id, int* reached);

  /// @brief 清除电机报警
  ///        Clear motor alarm
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///        id: Motor ID, [0, DOF=degree of freedom], 0 means broadcast,
  ///       1~DOF表示单个ID设置
  ///        1~DOF means single ID setting
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 清除电机1的报警
  ///           // Clear alarm of motor 1
  ///           sdk_handle->set_clear_alarm(1);
  ///           // 清除所有电机的报警
  ///           // Clear alarm of all motors
  ///           sdk_handle->set_clear_alarm();
  int set_clear_alarm(int id = 0);

  /// @brief 获取当前报警状态
  ///        Get current alarm status
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param alarm 报警状态, 参考 LAM_* 报警码枚举
  ///        alarm: Alarm status, refer to LAM_* alarm code enum
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int alarm_status;
  ///           sdk_handle->get_now_alarm(1, &alarm_status);
  int get_now_alarm(int id, int* alarm);


  /// @brief 是否允许在没回零状态下运动
  ///        Whether to allow movement without homing
  /// @param enable 0: 不允许 1: 允许
  ///        enable: 0: Disallow 1: Allow
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  int set_move_no_home(int enable = 0);

  /// @brief 设置是否启用LAC_DOF_6_S手指角度/连杆角度公式转换
  ///        Enable LAC_DOF_6_S finger/linkage angle conversion formulas
  /// @param enable 0: 关闭 1: 开启
  ///        enable: 0: disable 1: enable
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  int set_angle_conversion_enable(int enable = 1);

  /// @brief 获取角度公式转换开关状态
  ///        Get angle conversion formula enable state
  /// @param enable 输出参数，0: 关闭 1: 开启
  ///        enable: Output parameter, 0: disable 1: enable
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  int get_angle_conversion_enable(int* enable);

  /// @brief 启动电机回零
  ///        Start motor homing
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///        id: Motor ID, [0, DOF=degree of freedom], 0 means broadcast,
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 启动电机1回零
  ///           // Start motor 1 homing
  ///           sdk_handle->home_motors(1);
  ///           // 启动所有电机回零
  ///           // Start all motors homing
  ///           sdk_handle->home_motors(0);
  int home_motors(int id);

  /// @brief 获取电机目标角度上下限
  ///        Get motor target angle limits
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param min_angle 目标角度最小值
  ///        min_angle: Minimum target angle
  /// @param max_angle 目标角度最大值
  ///        max_angle: Maximum target angle
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float min_angle, max_angle;
  ///           sdk_handle->get_limit_target_angle(1, &min_angle, &max_angle);
  int get_limit_target_angle(int id, float* min_angle, float* max_angle);

  /// @brief 设置电机目标角度
  ///        Set motor target angle
  /// @param id 电机ID, [0, DOF=自由度数量]
  ///        id: Motor ID, [0, DOF=degree of freedom]
  /// @param angle 目标角度, 范围[0,MAX=电机可达最大角度值],
  ///        angle: Target angle, range [0,MAX=maximum reachable angle of motor],
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 设置电机1的目标角度为45度
  ///           // Set target angle of motor 1 to 45 degrees
  ///           sdk_handle->set_target_angle(1, 45.0f);
  int set_target_angle(int id, float angle);

  /// @brief 获取最近一次设置的电机目标角度
  ///        Get last set motor target angle
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param angle 目标角度, 范围[0,MAX=电机可达最大角度值]
  ///        angle: Target angle, range [0,MAX=maximum reachable angle of motor]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float target_angle;
  ///           sdk_handle->get_target_angle(1, &target_angle);
  int get_target_angle(int id, float* angle);

  /// @brief 设置电机目标位置（行程当量）
  ///        Set motor target position (stroke equivalent)
  /// @param id 电机ID, 范围 [0, DOF=自由度数量]
  ///        id: Motor ID, range [0, DOF=degree of freedom]
  /// @param position 目标位置, 范围 [0=起始位, 10000=满行程]
  ///        position: Target position, range [0=start position, 10000=full stroke]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 设置电机1的目标位置为5000（中间位置）
  ///           // Set target position of motor 1 to 5000 (middle position)
  ///           sdk_handle->set_target_position(1, 5000);
  int set_target_position(int id, int position);

  /// @brief 获取最近一次设置的电机目标位置（行程当量）
  ///        Get last set motor target position (stroke equivalent)
  /// @param id 电机ID, 范围 [1, DOF=自由度数量]
  ///        id: Motor ID, range [1, DOF=degree of freedom]
  /// @param position 目标位置, 范围 [0=起始位, 10000=满行程]
  ///        position: Target position, range [0=start position, 10000=full stroke]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int target_pos;
  ///           sdk_handle->get_target_position(1, &target_pos);
  int get_target_position(int id, int* position);

  /// @brief 设置电机目标速度(默认使用set_angular_velocity)
  ///        Set motor target velocity (default uses set_angular_velocity)
  /// @param id 电机ID, [0, DOF=自由度数量]
  ///        id: Motor ID, [0, DOF=degree of freedom]
  /// @param velocity 目标速度, 单位: °/s
  ///        velocity: Target velocity, unit: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 设置电机1的目标速度为30度/秒
  ///           // Set target velocity of motor 1 to 30 degrees/second
  ///           sdk_handle->set_velocity(1, 30.0f);
  int set_velocity(int id, float velocity);

  /// @brief 获取最近一次设置的电机目标速度(默认使用get_angular_velocity)
  ///        Get last set motor target velocity (default uses get_angular_velocity)
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param velocity 目标速度, 单位: °/s
  ///        velocity: Target velocity, unit: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float velocity;
  ///           sdk_handle->get_velocity(1, &velocity);
  int get_velocity(int id, float* velocity);

  /// @brief 设置电机目标速度
  ///        Set motor target velocity
  /// @param id 电机ID, [0, DOF=自由度数量]
  ///        id: Motor ID, [0, DOF=degree of freedom]
  /// @param velocity 目标速度, 单位: °/s
  ///        velocity: Target velocity, unit: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 设置电机1的目标速度为30°/s
  ///           // Set target velocity of motor 1 to 30°/s
  ///           sdk_handle->set_angular_velocity(1, 30.0f);
  int set_angular_velocity(int id, float velocity);

  /// @brief 获取最近一次设置的电机目标速度
  ///        Get last set motor target velocity
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param velocity 目标速度, 单位: °/s
  ///        velocity: Target velocity, unit: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float velocity;
  ///           sdk_handle->get_angular_velocity(1, &velocity);
  int get_angular_velocity(int id, float* velocity);

  /// @brief 设置电机目标速度（行程当量速度）
  ///        Set motor target velocity (stroke equivalent velocity)
  /// @param id 电机ID, 范围 [0, DOF=自由度数量]
  ///        id: Motor ID, range [0, DOF=degree of freedom]
  /// @param velocity 目标速度, 单位: 当量/秒（即每秒移动的当量数）
  ///        velocity: Target velocity, unit: equivalent/second (i.e., number of equivalents moved per second)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 设置电机1的目标位置速度为1000当量/秒
  ///           // Set target position velocity of motor 1 to 1000 equivalents/second
  ///           sdk_handle->set_position_velocity(1, 1000);
  int set_position_velocity(int id, int velocity);

  /// @brief 获取最近一次设置的电机目标速度（行程当量速度）
  ///        Get last set motor target velocity (stroke equivalent velocity)
  /// @param id 电机ID, 范围 [1, DOF=自由度数量]
  ///        id: Motor ID, range [1, DOF=degree of freedom]
  /// @param velocity 输出参数，返回目标位置速度值（单位: 当量/秒）
  ///        velocity: Output parameter, returns target position velocity value (unit: equivalent/second)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int pos_velocity;
  ///           sdk_handle->get_position_velocity(1, &pos_velocity);
  int get_position_velocity(int id, int* velocity);

  /// @brief 设置电机最大电流
  ///        Set motor maximum current
  /// @param id 电机ID, [0, DOF=自由度数量]
  ///        id: Motor ID, [0, DOF=degree of freedom]
  /// @param current 最大电流, 单位 ‰(千分比)
  ///        current: Maximum current, unit: ‰ (per mille)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 设置电机1的最大电流为500‰
  ///           // Set maximum current of motor 1 to 500‰
  ///           sdk_handle->set_max_current(1, 500);
  int set_max_current(int id, int current);

  /// @brief 获取最近一次设置的电机最大电流
  ///        Get last set motor maximum current
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param current 最大电流, 单位 ‰(千分比)
  ///        current: Maximum current, unit: ‰ (per mille)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int max_current;
  ///           sdk_handle->get_max_current(1, &max_current);
  int get_max_current(int id, int* current);

  /// @brief 驱动电机运动
  ///        Drive motor movement
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///        id: Motor ID, [0, DOF=degree of freedom], 0 means broadcast,
  ///       1~DOF表示单个ID设置
  ///        1~DOF means single ID setting
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 驱动电机1运动
  ///           // Drive motor 1 to move
  ///           sdk_handle->move_motors(1);
  ///           // 驱动所有电机运动
  ///           // Drive all motors to move
  ///           sdk_handle->move_motors();
  int move_motors(int id = 0);

  /// @brief 停止电机运动
  ///        Stop motor movement
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播,
  ///        id: Motor ID, [0, DOF=degree of freedom], 0 means broadcast,
  ///       1~DOF表示单个ID设置
  ///        1~DOF means single ID setting
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 停止电机1运动
  ///           // Stop motor 1 movement
  ///           sdk_handle->stop_motors(1);
  ///           // 停止所有电机运动
  ///           // Stop all motors movement
  ///           sdk_handle->stop_motors();
  int stop_motors(int id = 0);

  /// @brief 执行指定手势
  ///        Execute specified gesture
  /// @param gesture_id 执行的手势id
  ///        gesture_id: Executed gesture id
  /// @param velocity 目标速度, 单位: 当量/秒（即每秒移动的当量数）
  ///        velocity: Target velocity, unit: equivalent/second (i.e., number of equivalents moved per second)
  /// @param current 最大电流, 单位 ‰(千分比)
  ///        current: Maximum current, unit: ‰ (per mille)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 执行手势1, 目标速度为20000当量/秒, 最大电流为1000‰
  ///           // Execute gesture 1, target velocity 20000 equivalents/second, maximum current 1000‰
  ///           sdk_handle->play_gesture(1, 20000, 1000);
  int play_gesture(int gesture_id, int velocity, int current);

  /// @brief 获取电机当前状态
  ///        Get current motor status
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param status 运行状态, 0：正常停止状态 1：正常运行状态 2：报警停止状态
  ///        status: Running status, 0: Normal stop status 1: Normal running status 2: Alarm stop status
  ///       3：正限位状态 4：负限位状态 5：正负限位状态 6：急停状态
  ///        3: Positive limit status 4: Negative limit status 5: Both limit status 6: Emergency stop status
  ///       7：回零运行状态
  ///        7: Homing running status
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int motor_status;
  ///           sdk_handle->get_now_status(1, &motor_status);
  int get_now_status(int id, int* status);

  /// @brief 获取电机当前角度位置
  ///        Get current motor angle position
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param angle 当前角度位置, 单位: °
  ///        angle: Current angle position, unit: °
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float current_angle;
  ///           sdk_handle->get_now_angle(1, &current_angle);
  int get_now_angle(int id, float* angle);

  /// @brief 获取电机当前行程位置
  ///        Get current motor stroke position
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param position 当前行程位置, 范围 [0=起始位, 10000=满行程]
  ///        position: Current stroke position, range [0=start position, 10000=full stroke]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int current_position;
  ///           sdk_handle->get_now_position(1, &current_position);
  int get_now_position(int id, int* position);

  /// @brief 获取电机当前速度(单位:度, 默认使用get_now_angular_velocity)
  ///        Get current motor velocity (unit: degree, default uses get_now_angular_velocity)
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param velocity 当前速度, 单位: °/s
  ///        velocity: Current velocity, unit: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float current_velocity;
  ///           sdk_handle->get_now_velocity(1, &current_velocity);
  int get_now_velocity(int id, float* velocity);

  /// @brief 获取电机当前速度(单位:度)
  ///        Get current motor velocity (unit: degree)
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param velocity 当前速度, 单位: °/s
  ///        velocity: Current velocity, unit: °/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float current_angular_velocity;
  ///           sdk_handle->get_now_angular_velocity(1, &current_angular_velocity);
  int get_now_angular_velocity(int id, float* velocity);

  /// @brief 获取电机当前速度(单位:当量)
  ///        Get current motor velocity (unit: equivalent)
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param velocity 当前速度, 单位: 当量/s
  ///        velocity: Current velocity, unit: equivalent/s
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int current_pos_velocity;
  ///           sdk_handle->get_now_position_velocity(1, &current_pos_velocity);
  int get_now_position_velocity(int id, int* velocity);

  /// @brief 获取电机当前电流
  ///        Get current motor current
  /// @param id 电机ID, [1, DOF=自由度数量]
  ///        id: Motor ID, [1, DOF=degree of freedom]
  /// @param current 当前电流, 单位 ‰(千分比)
  ///        current: Current current, unit: ‰ (per mille)
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int current_current;
  ///           sdk_handle->get_now_current(1, &current_current);
  int get_now_current(int id, int* current);

  /// @brief 同步当前位置到目标位置
  ///        Sync current position to target position
  /// @param id 电机ID, [0, DOF=自由度数量], 0表示广播所有电机
  ///        id: Motor ID, [0, DOF=degree of freedom], 0 means broadcast all motors
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 同步电机1的当前位置到目标位置
  ///           // Sync current position of motor 1 to target position
  ///           sdk_handle->sync_position_to_target(1);
  ///           // 同步所有电机的当前位置到目标位置
  ///           // Sync current position of all motors to target position
  ///           sdk_handle->sync_position_to_target(0);
  int sync_position_to_target(int id = 0);

  /// @brief 设置是否开启监控传感器
  ///        Set whether to enable monitoring sensor
  /// @param enable 是否开启监控传感器
  ///        enable: Whether to enable monitoring sensor
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_sensor_enable(1);
  int set_sensor_enable(int enable);

  /// @brief 设置传感器数据格式
  ///        Set sensor data format
  /// @param format 格式类型, 默认为0
  ///        format: Format type, default is 0
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_sensor_data_format(0);
  int set_sensor_data_format(int format);

  /// @brief 设置传感器的映射顺序
  ///        Set sensor mapping order
  ///           按照 大拇指, 食指, 中指, 无名指, 小拇指, 手掌 的顺序传入 sensor_id
  ///           In the order of thumb, index, middle, ring, little finger, palm, pass sensor_id
  /// @param order sensor_id 数组指针, 使用 LSS_* 宏
  ///        order: sensor_id array pointer, use LSS_* macros
  /// @param size 数组大小, 范围 [1, LSS_MAX_COUNT]
  ///        size: Array size, range [1, LSS_MAX_COUNT]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           int order[6] = {LSS_FINGER_1_1, LSS_FINGER_2_1, LSS_FINGER_3_1,
  ///                           LSS_FINGER_4_1, LSS_FINGER_5_1, LSS_HAND_PALM};
  ///           sdk_handle->set_sensor_order(order, 6);
  int set_sensor_order(const int* order, int size);

  /// @brief 获取指定位置的触觉传感器分布位置
  ///        Get tactile sensor distribution position at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param x_values 返回的X坐标数组指针,位置的范围是[0.0-1.0]
  ///        x_values: Returned X coordinate array pointer, position range [0.0-1.0]
  /// @param y_values 返回的Y坐标数组指针,位置的范围是[0.0-1.0]
  ///        y_values: Returned Y coordinate array pointer, position range [0.0-1.0]
  /// @param io_count 返回的数组长度
  ///        io_count: Returned array length
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 1. 获取count大小
  ///           // 1. Get count size
  ///           int count = 0;
  ///           get_finger_sensor_pos(sensor_id, nullptr, nullptr, &count);
  ///           // 2. 获取实际坐标
  ///           // 2. Get actual coordinates
  ///           std::vector<float> x(count), y(count);
  ///           get_finger_sensor_pos(sensor_id, x.data(), y.data(), &count);
  int get_finger_sensor_pos(int sensor_id, float* x_values, float* y_values,
                            int* io_count);

  /// @brief 获取指定位置的触觉传感器压力值
  ///        Get tactile sensor pressure value at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param pressure_list 返回的数组指针,力的范围是[0.0-1.0]
  ///        pressure_list: Returned array pointer, force range [0.0-1.0]
  /// @param io_count 返回的数组长度
  ///        io_count: Returned array length
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///            // 1. 获取count大小
  ///            // 1. Get count size
  ///            int count = 0;
  ///            sdk_handle->get_finger_pressure(sensor_id, nullptr, &count);
  ///            // 2. 获取实际压力数组
  ///            // 2. Get actual pressure array
  ///            std::vector<float> pressures(count);
  ///            sdk_handle->get_finger_pressure(sensor_id, pressures.data(),
  ///            &count);
  int get_finger_pressure(int sensor_id, float* pressure_list, int* io_count);

  /// @brief 设置基准的触觉传感器当前值,即重置所有传感器当前状态为0状态
  ///        Set reference tactile sensor current value, i.e., reset all sensor current states to 0 state
  ///       get_finger_pressure返回的传感器差值以这个为基准
  ///       The sensor difference returned by get_finger_pressure is based on this
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->set_finger_pressure_reset();
  int set_finger_pressure_reset();

  /// @brief 获取指定位置的触觉传感器法向力
  ///        Get tactile sensor normal force at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param normal_force 返回的法向力,力的范围是[0.0-1.0]
  ///        normal_force: Returned normal force, force range [0.0-1.0]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float normal_force;
  ///           sdk_handle->get_finger_normal_force(LSS_FINGER_1_1, &normal_force);
  int get_finger_normal_force(int sensor_id, float* normal_force);

  /// @brief 获取指定位置的触觉传感器法向力数组
  ///        Get tactile sensor normal force array at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param normal_force_list 返回的法向力数组指针,力的范围是[0.0-1.0]
  ///        normal_force_list: Returned normal force array pointer, force range [0.0-1.0]
  /// @param io_count 返回的数组长度
  ///        io_count: Returned array length
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 1. 获取count大小
  ///           // 1. Get count size
  ///           int count = 0;
  ///           get_finger_normal_force_ex(sensor_id, nullptr, &count);
  ///           // 2. 获取实际法向力数组
  ///           // 2. Get actual normal force array
  ///           std::vector<float> normal_force(count);
  ///           get_finger_normal_force_ex(sensor_id, normal_force.data(),
  ///           &count);
  int get_finger_normal_force_ex(int sensor_id, float* normal_force_list,
                                 int* io_count);

  /// @brief 获取指定位置的触觉传感器切向力
  ///        Get tactile sensor tangential force at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param tangential_force 返回的切向力,力的范围是[0.0-1.0]
  ///        tangential_force: Returned tangential force, force range [0.0-1.0]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float tangential_force;
  ///           sdk_handle->get_finger_tangential_force(LSS_FINGER_1_1,
  ///           &tangential_force);
  int get_finger_tangential_force(int sensor_id, float* tangential_force);

  /// @brief 获取指定位置的触觉传感器切向力数组
  ///        Get tactile sensor tangential force array at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param tangential_force_list 返回的切向力数组指针,力的范围是[0.0-1.0]
  ///        tangential_force_list: Returned tangential force array pointer, force range [0.0-1.0]
  /// @param io_count 返回的数组长度
  ///        io_count: Returned array length
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 1. 获取count大小
  ///           // 1. Get count size
  ///           int count = 0;
  ///           get_finger_tangential_force_ex(sensor_id, nullptr, &count);
  ///           // 2. 获取实际切向力数组
  ///           // 2. Get actual tangential force array
  ///           std::vector<float> tangential_force(count);
  ///           get_finger_tangential_force_ex(sensor_id,
  ///           tangential_force.data(), &count);
  int get_finger_tangential_force_ex(int sensor_id,
                                     float* tangential_force_list,
                                     int* io_count);

  /// @brief 获取指定位置的触觉传感器的受力方向
  ///        Get tactile sensor force direction at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param force_direction 返回的受力方向,方向的范围是[0-360]
  ///        force_direction: Returned force direction, direction range [0-360]
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float force_direction;
  ///           sdk_handle->get_finger_force_direction(LSS_FINGER_1_1,
  ///           &force_direction);
  int get_finger_force_direction(int sensor_id, float* force_direction);

  /// @brief 获取指定位置的触觉传感器受力方向数组
  ///        Get tactile sensor force direction array at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param force_direction_list 返回的受力方向数组指针,方向的范围是[0-360]
  ///        force_direction_list: Returned force direction array pointer, direction range [0-360]
  /// @param io_count 返回的数组长度
  ///        io_count: Returned array length
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 1. 获取count大小
  ///           // 1. Get count size
  ///           int count = 0;
  ///           get_finger_force_direction_ex(sensor_id, nullptr, &count);
  ///           // 2. 获取实际受力方向数组
  ///           // 2. Get actual force direction array
  ///           std::vector<float> force_direction(count);
  ///           get_finger_force_direction_ex(sensor_id, force_direction.data(),
  ///           &count);
  int get_finger_force_direction_ex(int sensor_id, float* force_direction_list,
                                    int* io_count);

  /// @brief 获取指定位置的触觉传感器的接近程度
  ///        Get tactile sensor proximity at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param proximity 返回的接近程度
  ///        proximity: Returned proximity
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           float proximity;
  ///           sdk_handle->get_finger_proximity(LSS_FINGER_1_1, &proximity);
  int get_finger_proximity(int sensor_id, float* proximity);

  /// @brief 获取指定位置的触觉传感器接近程度数组
  ///        Get tactile sensor proximity array at specified location
  /// @param sensor_id 传感器的id,通过文件顶部的枚举输入
  ///        sensor_id: Sensor ID, input through enum at the top of the file
  /// @param proximity_list 返回的接近程度数组指针
  ///        proximity_list: Returned proximity array pointer
  /// @param io_count 返回的数组长度
  ///        io_count: Returned array length
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 1. 获取count大小
  ///           // 1. Get count size
  ///           int count = 0;
  ///           get_finger_proximity_ex(sensor_id, nullptr, &count);
  ///           // 2. 获取实际接近程度数组
  ///           // 2. Get actual proximity array
  ///           std::vector<float> proximity(count);
  ///           get_finger_proximity_ex(sensor_id, proximity.data(), &count);
  int get_finger_proximity_ex(int sensor_id, float* proximity_list,
                              int* io_count);

  /// @brief SDO设置驱动参数
  ///        Set SDO drive parameters
  /// @param index 参数索引
  ///        index: Parameter index
  /// @param subindex 参数子索引
  ///        subindex: Parameter subindex
  /// @param value 参数值
  ///        value: Parameter value
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  int set_sdo_drive_param(unsigned int index, unsigned char subindex,
                          unsigned int value);

  /// @brief SDO获取驱动参数
  ///        Get SDO drive parameters
  /// @param index 参数索引
  ///        index: Parameter index
  /// @param subindex 参数子索引
  ///        subindex: Parameter subindex
  /// @param value 返回的参数值指针
  ///        value: Returned parameter value pointer
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  int get_sdo_drive_param(unsigned int index, unsigned char subindex,
                          unsigned int* value);

  /// @brief SDO保存驱动参数
  ///        Save SDO drive parameters
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  int save_sdo_drive_param();

  /// @brief 发送 OTA 固件升级数据
  ///        Send OTA firmware upgrade data
  /// @param data 待发送的数据指针
  ///        data: Pointer to the data to be sent
  /// @param size 数据长度（字节数）
  ///        size: Data length in bytes
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  int set_ota_send_data(const unsigned char* data, unsigned int size);

  /// @brief 获取 OTA 固件升级应答数据
  ///        Get OTA firmware upgrade response data
  /// @param data 接收缓冲区指针，传入 null 时仅通过 io_count 返回所需长度
  ///        data: Receive buffer pointer; pass null to query required length via io_count
  /// @param io_count 传入时为缓冲区容量，返回时为实际写入字节数
  ///        io_count: Input as buffer capacity, output as actual bytes written
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  int get_ota_recv_data(unsigned char* data, int* io_count);

  /// @brief 开启/关闭日志打印
  ///        Enable/disable log printing
  /// @param on 是否开启日志打印, true开启, false关闭(默认)
  ///        on: Whether to enable log printing, true enable, false disable(default)
  /// @param maxsize 最大保存日志数量, 防止内存无限增长
  ///        maxsize: Maximum number of logs to save, prevent infinite memory growth
  /// @example
  ///           // 开启日志打印，最大保存10000条日志
  ///           // Enable log printing, maximum 10000 logs saved
  ///           sdk_handle->log_on(true, 10000);
  void log_on(bool on = false, int maxsize = 10000);

  /// @brief 设置需要打印的发送数据地址数组
  ///        Set send data address array to be printed
  /// @param cmd 需要打印的地址数组, 空表示全打印
  ///        cmd: Address array to be printed, empty means print all
  /// @param count 指令数组长度
  ///        count: Command array length
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 设置需要打印的发送数据地址
  ///           // Set send data address to be printed
  ///           int send_cmds[] = {0x00, 0x01, 0x0B};
  ///           sdk_handle->log_send(send_cmds, 3);
  int log_send(int* cmd, int count);

  /// @brief 设置需要打印的接收数据地址数组
  ///        Set receive data address array to be printed
  /// @param cmd 需要打印的地址数组, 空表示全打印
  ///        cmd: Address array to be printed, empty means print all
  /// @param count 指令数组长度
  ///        count: Command array length
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           // 设置需要打印的接收数据地址
  ///           // Set receive data address to be printed
  ///           int recv_cmds[] = {0x00, 0x40};
  ///           sdk_handle->log_recv(recv_cmds, 2);
  int log_recv(int* cmd, int count);

  /// @brief 清空日志地址数组
  ///        Clear log address array
  /// @param send 是否清空发送日志地址数组
  ///        send: Whether to clear send log address array
  /// @param recv 是否清空接收日志地址数组
  ///        recv: Whether to clear receive log address array
  /// @example
  ///           // 清空发送和接收日志地址数组
  ///           // Clear send and receive log address arrays
  ///           sdk_handle->log_reset();
  ///           // 仅清空发送日志地址数组
  ///           // Only clear send log address array
  ///           sdk_handle->log_reset(true, false);
  void log_reset(bool send = true, bool recv = true);

  /// @brief 将日志保存在文件中
  ///        Save log to file
  /// @param file_name - 文件地址
  ///        file_name - File address
  /// @return 0 - 执行成功, 其他 - 参考错误码
  ///        0 - Success, others - Refer to error codes
  /// @example
  ///           sdk_handle->log_save("hand_log.txt");
  int log_save(const char* file_name);

  /// @brief 清空当前日志记录
  ///        Clear current log records
  /// @example
  ///           sdk_handle->log_clear();
  void log_clear();

  /// @brief 日志回调函数
  ///        Log callback function
  /// @example
  ///           // 定义日志回调函数
  ///           // Define log callback function
  ///           void log_callback(const char* log) {
  ///               std::cout << log << std::endl;
  ///           }
  ///           // 设置日志回调
  ///           // Set log callback
  ///           sdk_handle->set_log_callback(log_callback);
  void set_log_callback(LogAddCallback callback);
};
}  // namespace lhplib

#endif  // LHANDPROLIB_HPP
