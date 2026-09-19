"""
LHandProLib的Python面向对象封装
"""

from typing import Optional, List, Tuple, Callable
from ctypes import c_int, c_uint, c_float, c_bool, c_char, c_char_p, POINTER, byref

from .lhandprolib_loader import (
    get_global_lhandprolib_lib,
    LER_NONE, LER_PARAMETER, LER_KEY_FUNC_UNINIT, LER_GET_CONFIGURATION,
    LER_DATA_ANOMALY, LER_COMM_CONNECT, LER_COMM_SEND, LER_COMM_RECV,
    LER_COMM_DATA_FORMAT, LER_INVALID_PATH, LER_LOG_SAVE_FAIL, LER_NOT_HOME, LER_UNKNOWN,
    LAC_DOF_6, LAC_DOF_6_S, LAC_DOF_16,
    LCN_ECAT, LCN_CANFD, LCN_RS485,
    LCM_POSITION, LCM_VELOCITY, LCM_TORQUE, LCM_VEL_TOR, LCM_POS_TOR, LCM_HOME,
    LST_STOPPED, LST_RUNNING, LST_ALARM, LST_POS_LIMIT, LST_NEG_LIMIT,
    LST_BOTH_LIMIT, LST_EMG_STOP, LST_HOMING,
    LSS_FINGER_1_1, LSS_FINGER_1_2, LSS_FINGER_2_1, LSS_FINGER_2_2,
    LSS_FINGER_3_1, LSS_FINGER_3_2, LSS_FINGER_4_1, LSS_FINGER_4_2,
    LSS_FINGER_5_1, LSS_FINGER_5_2, LSS_HAND_PALM, LSS_MAX_COUNT,
    LDR_HAND_RIGHT, LDR_HAND_LEFT,
    LogAddCallbackWrapper, ECSendDataCallbackWrapper, CANFDSendDataCallbackWrapper,
    RS485SendDataCallbackWrapper
)


class LHandProLibError(Exception):
    """LHandProLib操作异常"""

    def __init__(self, error_code: int, message: str):
        self.error_code = error_code
        self.message = message
        super().__init__(f"Error {error_code}: {message}")


class PyRS485SharedBus:
    """SDK-owned scheduler for one physical RS485 serial bus."""

    def __init__(self, lib_path: Optional[str] = None):
        self._lib_loader = get_global_lhandprolib_lib(lib_path)
        self._lib = self._lib_loader.lib
        self._handle = self._lib.lhandprolib_rs485_shared_bus_create()
        if not self._handle:
            raise RuntimeError("无法创建RS485SharedBus实例")
        self._callbacks = {}

    def __del__(self):
        if hasattr(self, "_handle") and self._handle:
            self._lib.lhandprolib_rs485_shared_bus_destroy(self._handle)
            self._handle = None

    def set_send_callback(self, callback: Callable[[bytes], bool]) -> None:
        """Set the opaque physical serial write callback."""

        def wrapper(data_ptr, length: int) -> bool:
            return callback(bytes(data_ptr[:length]))

        wrapped_callback = RS485SendDataCallbackWrapper(wrapper)
        self._callbacks["send"] = wrapped_callback
        self._lib.lhandprolib_rs485_shared_bus_set_send_callback(
            self._handle, wrapped_callback
        )

    def set_receive_data(self, data: bytes) -> int:
        """Feed an arbitrary raw chunk from the shared serial receive stream."""
        payload = data if isinstance(data, bytes) else bytes(data)
        result = self._lib.lhandprolib_rs485_shared_bus_set_receive_data(
            self._handle, payload, len(payload)
        )
        if result != LER_NONE:
            raise LHandProLibError(result, "RS485共享总线接收数据失败")
        return result

    def close(self) -> None:
        if self._handle:
            self._lib.lhandprolib_rs485_shared_bus_close(self._handle)


class PyLHandProLib:
    """LHandProLib的Python封装类"""

    def __init__(self, lib_path: Optional[str] = None):
        """
        初始化LHandPro实例

        Args:
            lib_path: 可选的库文件路径
        """
        self._lib_loader = get_global_lhandprolib_lib(lib_path)
        self._lib = self._lib_loader.lib
        self._handle = self._lib.lhandprolib_create()

        if not self._handle:
            raise RuntimeError("无法创建LHandProLib实例")

        # 存储回调引用以避免垃圾回收
        self._callbacks = {}
        self._rs485_shared_bus = None

    def __del__(self):
        """析构函数，确保资源释放"""
        if hasattr(self, '_handle') and self._handle:
            self._lib.lhandprolib_destroy(self._handle)

    def _check_error(self, result: int, operation: str) -> None:
        """检查错误码并抛出异常"""
        if result != LER_NONE:
            error_messages = {
                LER_PARAMETER: "参数错误",
                LER_KEY_FUNC_UNINIT: "关键函数未初始化",
                LER_GET_CONFIGURATION: "读取配置失败",
                LER_DATA_ANOMALY: "数据异常",
                LER_COMM_CONNECT: "通讯连接错误",
                LER_COMM_SEND: "通讯发送错误",
                LER_COMM_RECV: "通讯接收错误",
                LER_COMM_DATA_FORMAT: "通讯数据格式错误",
                LER_INVALID_PATH: "无效的文件路径",
                LER_LOG_SAVE_FAIL: "日志文件保存失败",
                LER_NOT_HOME: "没回零错误",
                LER_UNKNOWN: "未知错误",
            }
            message = error_messages.get(result, f"未知错误码: {result}")
            raise LHandProLibError(result, f"{operation}: {message}")

    def _free_buffer(self, ptr) -> None:
        if ptr:
            self._lib.lhandprolib_free_buffer(ptr)

    @staticmethod
    def _decode_payload(data: bytes) -> bytes:
        return data if isinstance(data, bytes) else bytes(data)

    def _cfunc(self, suffix: str):
        return getattr(self._lib, f"lhandprolib_{suffix}")

    def _call_checked(self, suffix: str, operation: str, *args) -> None:
        result = self._cfunc(suffix)(self._handle, *args)
        self._check_error(result, operation)

    def _get_scalar(self, suffix: str, c_type, operation: str, *args):
        value = c_type()
        result = self._cfunc(suffix)(self._handle, *args, byref(value))
        self._check_error(result, operation)
        return value.value

    def _get_pre_send_data(
        self, suffix: str, operation: str, buffer_size: int = 1024
    ) -> Tuple[bytes, int]:
        data_buffer = (c_char * buffer_size)()
        io_size = c_int(buffer_size)
        result = self._cfunc(suffix)(self._handle, data_buffer, byref(io_size))
        self._check_error(result, operation)
        return bytes(data_buffer[:io_size.value]), io_size.value

    def _get_float_array(self, suffix: str, sensor_id: int, operation: str) -> List[float]:
        values_ptr = POINTER(c_float)()
        count = c_int()
        try:
            result = self._cfunc(suffix)(
                self._handle, sensor_id, byref(values_ptr), byref(count)
            )
            self._check_error(result, operation)
            return [values_ptr[i] for i in range(count.value)] if values_ptr else []
        finally:
            self._free_buffer(values_ptr)

    def _get_two_float_arrays(
        self, suffix: str, sensor_id: int, operation: str
    ) -> Tuple[List[float], List[float]]:
        first_ptr = POINTER(c_float)()
        second_ptr = POINTER(c_float)()
        count = c_int()
        try:
            result = self._cfunc(suffix)(
                self._handle, sensor_id, byref(first_ptr), byref(second_ptr), byref(count)
            )
            self._check_error(result, operation)
            first = [first_ptr[i] for i in range(count.value)] if first_ptr else []
            second = [second_ptr[i] for i in range(count.value)] if second_ptr else []
            return first, second
        finally:
            self._free_buffer(first_ptr)
            self._free_buffer(second_ptr)

    # 初始化和关闭
    def initial(self, mode: int) -> None:
        """初始化库"""
        self._call_checked("initial", "初始化", mode)

    def initial_ex(self, mode: int, node_id: int) -> None:
        """初始化库（扩展版本，包含节点ID）"""
        self._call_checked("initial_ex", "initial_ex", mode, node_id)

    def close(self) -> None:
        """关闭库"""
        self._lib.lhandprolib_close(self._handle)

    def start_monitor(self) -> None:
        """启动后台监控线程"""
        self._lib.lhandprolib_start_monitor(self._handle)

    def stop_monitor(self) -> None:
        """停止后台监控线程"""
        self._lib.lhandprolib_stop_monitor(self._handle)

    def set_dry_run_mode(self, enable: bool) -> None:
        """设置 dry-run 模式，开启后命令数据正常生成但不通过回调发送"""
        self._lib.lhandprolib_set_dry_run_mode(self._handle, enable)

    def get_dry_run_mode(self) -> bool:
        """获取当前 dry-run 模式状态"""
        return self._lib.lhandprolib_get_dry_run_mode(self._handle)

    # 回调设置
    def set_send_rpdo_callback(self, callback: Callable[[bytes], bool]) -> None:
        """设置发送RPDO回调"""

        def wrapper(data_ptr, length: int) -> bool:
            data = bytes(data_ptr[:length])
            return callback(data)

        wrapped_callback = ECSendDataCallbackWrapper(wrapper)
        self._callbacks['send_rpdo'] = wrapped_callback
        self._lib.lhandprolib_set_send_rpdo_callback(self._handle, wrapped_callback)

    def set_send_canfd_callback(self, callback: Callable[[int, bytes, int], bool]) -> None:
        """设置发送CANFD回调"""

        def wrapper(msg_id: int, data_ptr, length: int, is_extended: int) -> bool:
            data = bytes(data_ptr[:length])
            return callback(msg_id, data, is_extended)

        wrapped_callback = CANFDSendDataCallbackWrapper(wrapper)
        self._callbacks['send_canfd'] = wrapped_callback
        self._lib.lhandprolib_set_send_canfd_callback(self._handle, wrapped_callback)

    def set_send_rs485_callback(self, callback: Callable[[bytes], bool]) -> None:
        """设置发送RS485回调"""

        def wrapper(data_ptr, length: int) -> bool:
            data = bytes(data_ptr[:length])
            return callback(data)

        wrapped_callback = RS485SendDataCallbackWrapper(wrapper)
        self._callbacks['send_rs485'] = wrapped_callback
        self._lib.lhandprolib_set_send_rs485_callback(self._handle, wrapped_callback)

    def set_rs485_shared_bus(self, bus: Optional[PyRS485SharedBus]) -> None:
        """Bind this hand instance to a shared RS485 bus before initial_ex."""
        bus_handle = bus._handle if bus is not None else None
        self._lib.lhandprolib_set_rs485_shared_bus(self._handle, bus_handle)
        self._rs485_shared_bus = bus

    def set_log_callback(self, callback: Callable[[str], None]) -> None:
        """设置日志回调"""

        def wrapper(message: c_char_p) -> None:
            callback(message.decode('utf-8'))

        wrapped_callback = LogAddCallbackWrapper(wrapper)
        self._callbacks['log'] = wrapped_callback
        self._lib.lhandprolib_set_log_callback(self._handle, wrapped_callback)

    # 数据接收处理
    def set_tpdo_data_decode(self, data: bytes) -> int:
        """设置TPDO数据解码"""
        payload = self._decode_payload(data)
        return self._lib.lhandprolib_set_tpdo_data_decode(self._handle, payload, len(payload))

    def set_canfd_data_decode(self, msg_id: int, data: bytes) -> int:
        """设置CANFD数据解码"""
        payload = self._decode_payload(data)
        return self._lib.lhandprolib_set_canfd_data_decode(
            self._handle, c_uint(msg_id), payload, len(payload)
        )

    def set_rs485_data_decode(self, data: bytes) -> int:
        """设置RS485数据解码"""
        payload = self._decode_payload(data)
        return self._lib.lhandprolib_set_rs485_data_decode(self._handle, payload, len(payload))

    # RPDO数据处理
    def get_pre_send_rpdo_data(self) -> Tuple[bytes, int]:
        """获取预发送RPDO数据"""
        return self._get_pre_send_data("get_pre_send_rpdo_data", "获取RPDO数据")

    def get_pre_send_canfd_data(self) -> Tuple[bytes, int]:
        """获取预发送CANFD数据"""
        return self._get_pre_send_data("get_pre_send_canfd_data", "获取CANFD数据")

    def get_pre_send_rs485_data(self) -> Tuple[bytes, int]:
        """获取预发送RS485数据"""
        return self._get_pre_send_data("get_pre_send_rs485_data", "获取RS485数据")

    def get_pre_send_can_data(self) -> Tuple[bytes, int]:
        """获取预发送标准CAN数据（dry-run捕获），每帧11字节: CAN_ID(4B) + data(7B)"""
        return self._get_pre_send_data("get_pre_send_can_data", "获取CAN数据", 4096)

    # 配置相关
    def set_hand_type(self, hand_type: int) -> None:
        """设置手类型"""
        self._call_checked("set_hand_type", "设置手类型", hand_type)

    def get_dof(self) -> Tuple[int, int]:
        """获取自由度信息"""
        total = c_int()
        active = c_int()
        result = self._lib.lhandprolib_get_dof(self._handle, byref(total), byref(active))
        self._check_error(result, "获取自由度信息")
        return total.value, active.value

    def set_hand_direction(self, direction: int) -> None:
        """设置手部方向"""
        self._call_checked("set_hand_direction", "设置手部方向", direction)

    def get_hand_direction(self) -> int:
        """获取手部方向"""
        return self._get_scalar("get_hand_direction", c_int, "获取手部方向")

    def get_hand_type(self) -> int:
        """获取手类型"""
        return self._get_scalar("get_hand_type", c_int, "获取手类型")

    def set_move_no_home(self, move_no_home: int) -> None:
        """设置是否不回零"""
        self._call_checked("set_move_no_home", "设置是否不回零", move_no_home)

    # 电机控制
    def set_control_mode(self, motor_id: int, mode: int) -> None:
        """设置控制模式"""
        self._call_checked("set_control_mode", "设置控制模式", motor_id, mode)

    def get_control_mode(self, motor_id: int) -> int:
        """获取控制模式"""
        return self._get_scalar("get_control_mode", c_int, "获取控制模式", motor_id)

    def set_safe_current_enable(self, enable: bool) -> None:
        """设置安全电流使能状态"""
        self._call_checked("set_safe_current_enable", "设置安全电流使能状态", int(enable))

    def get_safe_current_enable(self) -> bool:
        """获取安全电流使能状态"""
        return bool(self._get_scalar("get_safe_current_enable", c_int, "获取安全电流使能状态"))

    def set_home_current(self, current: int) -> None:
        """设置回零电流，单位 %(百分比)，例如 100 表示 100%"""
        self._call_checked("set_home_current", "设置回零电流", current)

    def get_home_current(self) -> int:
        """获取回零电流，单位 %(百分比)"""
        return self._get_scalar("get_home_current", c_int, "获取回零电流")

    def set_enable(self, motor_id: int, enable: bool) -> None:
        """设置使能状态"""
        self._call_checked("set_enable", "设置使能状态", motor_id, int(enable))

    def get_enable(self, motor_id: int) -> bool:
        """获取使能状态"""
        return bool(self._get_scalar("get_enable", c_int, "获取使能状态", motor_id))

    def get_position_reached(self, motor_id: int) -> bool:
        """获取位置到达状态"""
        return bool(self._get_scalar("get_position_reached", c_int, "获取位置到达状态", motor_id))

    def get_torque_reached(self, motor_id: int) -> bool:
        """获取力矩到达状态"""
        return bool(self._get_scalar("get_torque_reached", c_int, "获取力矩到达状态", motor_id))

    def set_clear_alarm(self, motor_id: int) -> None:
        """清除报警"""
        self._call_checked("set_clear_alarm", "清除报警", motor_id)

    def get_now_alarm(self, motor_id: int) -> int:
        """获取当前报警"""
        return self._get_scalar("get_now_alarm", c_int, "获取当前报警", motor_id)

    def home_motors(self, motor_id: int) -> None:
        """回零电机"""
        self._call_checked("home_motors", "电机回零", motor_id)

    # 目标设置
    def set_target_angle(self, motor_id: int, angle: float) -> None:
        """设置目标角度"""
        self._call_checked("set_target_angle", "设置目标角度", motor_id, angle)

    def get_target_angle(self, motor_id: int) -> float:
        """获取目标角度"""
        return self._get_scalar("get_target_angle", c_float, "获取目标角度", motor_id)

    def set_target_position(self, motor_id: int, position: int) -> None:
        """设置目标位置"""
        self._call_checked("set_target_position", "设置目标位置", motor_id, position)

    def get_target_position(self, motor_id: int) -> int:
        """获取目标位置"""
        return self._get_scalar("get_target_position", c_int, "获取目标位置", motor_id)

    def set_angular_velocity(self, motor_id: int, velocity: float) -> None:
        """设置角速度"""
        self._call_checked("set_angular_velocity", "设置角速度", motor_id, velocity)

    def get_angular_velocity(self, motor_id: int) -> float:
        """获取角速度"""
        return self._get_scalar("get_angular_velocity", c_float, "获取角速度", motor_id)

    def set_position_velocity(self, motor_id: int, velocity: int) -> None:
        """设置位置速度"""
        self._call_checked("set_position_velocity", "设置位置速度", motor_id, velocity)

    def get_position_velocity(self, motor_id: int) -> int:
        """获取位置速度"""
        return self._get_scalar("get_position_velocity", c_int, "获取位置速度", motor_id)

    def set_max_current(self, motor_id: int, current: int) -> None:
        """设置最大电流"""
        self._call_checked("set_max_current", "设置最大电流", motor_id, current)

    def get_max_current(self, motor_id: int) -> int:
        """获取最大电流"""
        return self._get_scalar("get_max_current", c_int, "获取最大电流", motor_id)

    # 运动控制
    def move_motors(self, motor_id: int) -> None:
        """启动电机运动"""
        self._call_checked("move_motors", "启动电机运动", motor_id)

    def stop_motors(self, motor_id: int) -> None:
        """停止电机运动"""
        self._call_checked("stop_motors", "停止电机运动", motor_id)

    # 状态获取
    def get_now_status(self, motor_id: int) -> int:
        """获取当前状态"""
        return self._get_scalar("get_now_status", c_int, "获取当前状态", motor_id)

    def get_now_angle(self, motor_id: int) -> float:
        """获取当前角度"""
        return self._get_scalar("get_now_angle", c_float, "获取当前角度", motor_id)

    def get_now_position(self, motor_id: int) -> int:
        """获取当前位置"""
        return self._get_scalar("get_now_position", c_int, "获取当前位置", motor_id)

    def get_now_angular_velocity(self, motor_id: int) -> float:
        """获取当前角速度"""
        return self._get_scalar("get_now_angular_velocity", c_float, "获取当前角速度", motor_id)

    def get_now_position_velocity(self, motor_id: int) -> int:
        """获取当前位置速度"""
        return self._get_scalar("get_now_position_velocity", c_int, "获取当前位置速度", motor_id)

    def get_now_current(self, motor_id: int) -> int:
        """获取当前电流"""
        return self._get_scalar("get_now_current", c_int, "获取当前电流", motor_id)

    # 触觉传感器
    def set_sensor_enable(self, enable: bool) -> None:
        """设置传感器启用状态"""
        self._call_checked("set_sensor_enable", "设置传感器启用状态", int(enable))

    def set_sensor_data_format(self, format: int) -> None:
        """设置传感器数据格式"""
        self._call_checked("set_sensor_data_format", "设置传感器数据格式", format)

    def set_sensor_order(self, order: List[int]) -> None:
        """设置传感器顺序"""
        order_values = list(order)
        if not 1 <= len(order_values) <= LSS_MAX_COUNT:
            raise ValueError(f"传感器顺序数组长度必须在1到{LSS_MAX_COUNT}之间")
        order_array = (c_int * len(order_values))(*order_values)
        result = self._lib.lhandprolib_set_sensor_order(
            self._handle, order_array, len(order_values)
        )
        self._check_error(result, "设置传感器顺序")

    def get_finger_sensor_pos(self, sensor_id: int) -> Tuple[List[float], List[float]]:
        """获取手指传感器位置数据"""
        return self._get_two_float_arrays(
            "get_finger_sensor_pos", sensor_id, "获取手指传感器位置"
        )

    def get_finger_pressure(self, sensor_id: int) -> List[float]:
        """获取手指压力数据"""
        return self._get_float_array("get_finger_pressure", sensor_id, "获取手指压力")

    def set_finger_pressure_reset(self) -> None:
        """重置手指压力"""
        self._call_checked("set_finger_pressure_reset", "重置手指压力")

    def get_finger_normal_force_ex(self, sensor_id: int) -> List[float]:
        """获取手指法向力数组"""
        return self._get_float_array(
            "get_finger_normal_force_ex", sensor_id, "获取手指法向力数组"
        )

    def get_finger_tangential_force_ex(self, sensor_id: int) -> List[float]:
        """获取手指切向力数组"""
        return self._get_float_array(
            "get_finger_tangential_force_ex", sensor_id, "获取手指切向力数组"
        )

    def get_finger_force_direction_ex(self, sensor_id: int) -> List[float]:
        """获取手指力方向数组"""
        return self._get_float_array(
            "get_finger_force_direction_ex", sensor_id, "获取手指力方向数组"
        )

    def get_finger_proximity_ex(self, sensor_id: int) -> List[float]:
        """获取手指接近度数组"""
        return self._get_float_array(
            "get_finger_proximity_ex", sensor_id, "获取手指接近度数组"
        )

    def get_finger_normal_force(self, sensor_id: int) -> float:
        """获取手指法向力"""
        return self._get_scalar("get_finger_normal_force", c_float, "获取手指法向力", sensor_id)

    def get_finger_tangential_force(self, sensor_id: int) -> float:
        """获取手指切向力"""
        return self._get_scalar(
            "get_finger_tangential_force", c_float, "获取手指切向力", sensor_id
        )

    def get_finger_force_direction(self, sensor_id: int) -> float:
        """获取手指力方向"""
        return self._get_scalar(
            "get_finger_force_direction", c_float, "获取手指力方向", sensor_id
        )

    def get_finger_proximity(self, sensor_id: int) -> float:
        """获取手指接近度"""
        return self._get_scalar("get_finger_proximity", c_float, "获取手指接近度", sensor_id)

    # 日志管理
    def log_on(self, enable: bool, max_size: int = 1024) -> None:
        """启用/禁用日志"""
        self._lib.lhandprolib_log_on(self._handle, c_bool(enable), max_size)

    def log_save(self, file_name: str) -> None:
        """保存日志到文件"""
        self._call_checked("log_save", "保存日志", file_name.encode('utf-8'))

    def log_clear(self) -> None:
        """清除日志"""
        self._lib.lhandprolib_log_clear(self._handle)
