#ifndef LHANDPROLIB_LV_H
#define LHANDPROLIB_LV_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
    #ifdef SDK_LV_EXPORTS
        #define SDK_LV_API __declspec(dllexport)
    #else
        #define SDK_LV_API __declspec(dllimport)
    #endif
#else
    #define SDK_LV_API __attribute__((visibility("default")))
#endif

// ========== 通讯模式枚举 ==========
#define LV_COMM_ETHERCAT   0
#define LV_COMM_CANFD      1
#define LV_COMM_RS485      2

// ========== 通信生命周期 ==========

/// @brief 初始化指定通讯模式并连接设备
/// @param mode 通讯模式: LV_COMM_CANFD / LV_COMM_ETHERCAT / LV_COMM_RS485
/// @param device_index 设备索引号（CANFD: HCanbus设备号, EtherCAT: 网卡索引, RS485: 串口索引）
/// @param node_id 节点ID
/// @return 0-成功, 负数-错误
SDK_LV_API int lv_init(int mode, int device_index, int node_id);

/// @brief 关闭当前通讯连接并释放资源
SDK_LV_API void lv_close();

/// @brief 获取当前通讯连接状态
/// @return 1-已连接, 0-未连接
SDK_LV_API int lv_is_connected();

/// @brief 获取丢失帧数
/// @return 丢失的帧数量
SDK_LV_API int lv_get_lost_frames();

// ========== 设备扫描 (按通讯模式分组) ==========

/// @brief 扫描CANFD设备数量
SDK_LV_API int lv_canfd_scan_device_count();

/// @brief 获取CANFD设备描述字符串
SDK_LV_API int lv_canfd_get_device_string(int device_index,
                                                char* buffer, int buffer_size);

/// @brief 扫描EtherCAT网卡数量
SDK_LV_API int lv_ethercat_scan_device_count();

/// @brief 获取EtherCAT网卡描述字符串
SDK_LV_API int lv_ethercat_get_device_string(int device_index,
                                                   char* buffer, int buffer_size);

/// @brief 扫描RS485串口数量
SDK_LV_API int lv_rs485_scan_device_count();

/// @brief 获取RS485串口描述字符串
SDK_LV_API int lv_rs485_get_device_string(int device_index,
                                                char* buffer, int buffer_size);

// ========== 监控线程控制 ==========

/// @brief 启动后台监控线程
SDK_LV_API void lv_start_monitor();

/// @brief 停止后台监控线程
SDK_LV_API void lv_stop_monitor();

// ========== 配置 ==========
SDK_LV_API int lv_set_hand_type(int hand_type);
SDK_LV_API int lv_get_hand_type(int* hand_type);
SDK_LV_API int lv_set_hand_direction(int dir);
SDK_LV_API int lv_get_hand_direction(int* dir);
SDK_LV_API int lv_set_control_mode(int id, int mode);
SDK_LV_API int lv_get_control_mode(int id, int* mode);
SDK_LV_API int lv_set_safe_current_enable(int enable);
SDK_LV_API int lv_get_safe_current_enable(int* enable);
SDK_LV_API int lv_set_home_current(int current);
SDK_LV_API int lv_get_home_current(int* current);
SDK_LV_API int lv_set_enable(int id, int enable);
SDK_LV_API int lv_get_enable(int id, int* enable);
SDK_LV_API int lv_set_move_no_home(int enable);

// ========== 运动控制 ==========
SDK_LV_API int lv_set_target_angle(int id, float angle);
SDK_LV_API int lv_get_target_angle(int id, float* angle);
SDK_LV_API int lv_set_target_position(int id, int position);
SDK_LV_API int lv_get_target_position(int id, int* position);
SDK_LV_API int lv_set_velocity(int id, float velocity);
SDK_LV_API int lv_get_velocity(int id, float* velocity);
SDK_LV_API int lv_set_angular_velocity(int id, float velocity);
SDK_LV_API int lv_get_angular_velocity(int id, float* velocity);
SDK_LV_API int lv_set_position_velocity(int id, int velocity);
SDK_LV_API int lv_get_position_velocity(int id, int* velocity);
SDK_LV_API int lv_set_max_current(int id, int current);
SDK_LV_API int lv_get_max_current(int id, int* current);
SDK_LV_API int lv_move_motors(int id);
SDK_LV_API int lv_stop_motors(int id);
SDK_LV_API int lv_home_motors(int id);
SDK_LV_API int lv_sync_position_to_target(int id);
SDK_LV_API int lv_play_gesture(int gesture_id, int velocity, int current);
SDK_LV_API int lv_set_clear_alarm(int id);
SDK_LV_API int lv_get_limit_target_angle(int id, float* min_angle,
                                                float* max_angle);

// ========== 状态读取 ==========
SDK_LV_API int lv_get_dof(int* total, int* active);
SDK_LV_API int lv_get_now_status(int id, int* status);
SDK_LV_API int lv_get_now_angle(int id, float* angle);
SDK_LV_API int lv_get_now_position(int id, int* position);
SDK_LV_API int lv_get_now_velocity(int id, float* velocity);
SDK_LV_API int lv_get_now_angular_velocity(int id, float* velocity);
SDK_LV_API int lv_get_now_position_velocity(int id, int* velocity);
SDK_LV_API int lv_get_now_current(int id, int* current);
SDK_LV_API int lv_get_now_alarm(int id, int* alarm);
SDK_LV_API int lv_get_position_reached(int id, int* reached);
SDK_LV_API int lv_get_torque_reached(int id, int* reached);
SDK_LV_API int lv_get_firmware_version(float* version);

// ========== 传感器 ==========
SDK_LV_API int lv_set_sensor_enable(int enable);
SDK_LV_API int lv_set_sensor_data_format(int format);
SDK_LV_API int lv_set_sensor_order(const int* order, int size);
SDK_LV_API int lv_get_finger_normal_force(int sensor_id, float* force);
SDK_LV_API int lv_get_finger_normal_force_ex(int sensor_id,
                                                    float** force_list,
                                                    int* count);
SDK_LV_API int lv_get_finger_tangential_force(int sensor_id, float* force);
SDK_LV_API int lv_get_finger_tangential_force_ex(int sensor_id,
                                                        float** force_list,
                                                        int* count);
SDK_LV_API int lv_get_finger_force_direction(int sensor_id,
                                                    float* direction);
SDK_LV_API int lv_get_finger_force_direction_ex(int sensor_id,
                                                       float** dir_list,
                                                       int* count);
SDK_LV_API int lv_get_finger_proximity(int sensor_id, float* proximity);
SDK_LV_API int lv_get_finger_proximity_ex(int sensor_id,
                                                 float** prox_list, int* count);
SDK_LV_API int lv_get_finger_sensor_pos(int sensor_id, float** x,
                                               float** y, int* count);
SDK_LV_API int lv_get_finger_pressure(int sensor_id, float** pressure,
                                             int* count);
SDK_LV_API int lv_set_finger_pressure_reset();

// ========== 日志 ==========
SDK_LV_API void lv_log_on(int on, int maxsize);
SDK_LV_API int lv_log_send(int* cmd, int count);
SDK_LV_API int lv_log_recv(int* cmd, int count);
SDK_LV_API void lv_log_reset(int send, int recv);
SDK_LV_API int lv_log_save(const char* filename);
SDK_LV_API void lv_log_clear();

#ifdef __cplusplus
}
#endif

#endif
