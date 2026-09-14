"""
LHandProLib库加载器和函数原型定义
"""

import ctypes
import sys
from pathlib import Path
from typing import Optional
from ctypes import c_void_p, c_int, c_float, c_bool, c_uint, c_ubyte, c_char, c_char_p, POINTER

from .defaults import DEFAULT_C_API_PREFIX, DEFAULT_LIBRARY_BASE_NAME

# 错误码定义
LER_NONE = 0
LER_PARAMETER = 1
LER_KEY_FUNC_UNINIT = 2
LER_GET_CONFIGURATION = 3
LER_DATA_ANOMALY = 4
LER_COMM_CONNECT = 5
LER_COMM_SEND = 6
LER_COMM_RECV = 7
LER_COMM_DATA_FORMAT = 8
LER_INVALID_PATH = 9
LER_LOG_SAVE_FAIL = 10
LER_NOT_HOME = 11
LER_UNKNOWN = 999

# 型号枚举
LAC_DOF_6 = 0
LAC_DOF_6_S = 1
LAC_DOF_16 = 2

# 通讯类型枚举
LCN_ECAT = 0
LCN_CANFD = 1
LCN_RS485 = 2

# 控制模式枚举
LCM_POSITION = 0
LCM_VELOCITY = 1
LCM_TORQUE = 2
LCM_VEL_TOR = 3
LCM_POS_TOR = 4
LCM_HOME = 5

# 运行状态枚举
LST_STOPPED = 0
LST_RUNNING = 1
LST_ALARM = 2
LST_POS_LIMIT = 3
LST_NEG_LIMIT = 4
LST_BOTH_LIMIT = 5
LST_EMG_STOP = 6
LST_HOMING = 7

# 传感器ID枚举
LSS_FINGER_1_1 = 1
LSS_FINGER_1_2 = 2
LSS_FINGER_2_1 = 3
LSS_FINGER_2_2 = 4
LSS_FINGER_3_1 = 5
LSS_FINGER_3_2 = 6
LSS_FINGER_4_1 = 7
LSS_FINGER_4_2 = 8
LSS_FINGER_5_1 = 9
LSS_FINGER_5_2 = 10
LSS_HAND_PALM = 11
LSS_MAX_COUNT = 12

# 左右手枚举
LDR_HAND_RIGHT = 0
LDR_HAND_LEFT = 1

# 回调函数类型定义
LogAddCallbackWrapper = ctypes.CFUNCTYPE(None, c_char_p)
ECSendDataCallbackWrapper = ctypes.CFUNCTYPE(c_bool, POINTER(c_char), c_uint)
CANFDSendDataCallbackWrapper = ctypes.CFUNCTYPE(c_bool, c_uint, POINTER(c_char), c_uint, c_int)
RS485SendDataCallbackWrapper = ctypes.CFUNCTYPE(c_bool, POINTER(c_char), c_uint)
CANSendDataCallbackWrapper = ctypes.CFUNCTYPE(c_bool, c_uint, POINTER(c_char), c_uint)

P_CHAR = POINTER(c_char)
P_INT = POINTER(c_int)
P_UINT = POINTER(c_uint)
P_FLOAT = POINTER(c_float)
P_FLOAT_PTR = POINTER(P_FLOAT)

C_API_PROTOTYPES = (
    ("rs485_shared_bus_create", c_void_p, []),
    ("rs485_shared_bus_destroy", None, [c_void_p]),
    (
        "rs485_shared_bus_set_send_callback",
        None,
        [c_void_p, RS485SendDataCallbackWrapper],
    ),
    ("rs485_shared_bus_set_receive_data", c_int, [c_void_p, c_char_p, c_int]),
    ("rs485_shared_bus_close", None, [c_void_p]),
    ("create", c_void_p, []),
    ("destroy", None, [c_void_p]),
    ("free_buffer", None, [c_void_p]),
    ("initial", c_int, [c_void_p, c_int]),
    ("initial_ex", c_int, [c_void_p, c_int, c_int]),
    ("close", None, [c_void_p]),
    ("start_monitor", None, [c_void_p]),
    ("stop_monitor", None, [c_void_p]),
    ("set_dry_run_mode", None, [c_void_p, c_bool]),
    ("get_dry_run_mode", c_bool, [c_void_p]),
    ("set_send_rpdo_callback", None, [c_void_p, ECSendDataCallbackWrapper]),
    ("set_send_canfd_callback", None, [c_void_p, CANFDSendDataCallbackWrapper]),
    ("set_send_rs485_callback", None, [c_void_p, RS485SendDataCallbackWrapper]),
    ("set_rs485_shared_bus", None, [c_void_p, c_void_p]),
    ("set_send_can_callback", None, [c_void_p, CANSendDataCallbackWrapper]),
    ("set_log_callback", None, [c_void_p, LogAddCallbackWrapper]),
    ("set_tpdo_data_decode", c_int, [c_void_p, c_char_p, c_int]),
    ("set_canfd_data_decode", c_int, [c_void_p, c_uint, c_char_p, c_int]),
    ("set_rs485_data_decode", c_int, [c_void_p, c_char_p, c_int]),
    ("set_can_data_decode", c_int, [c_void_p, c_uint, c_char_p, c_int]),
    ("get_pre_send_rpdo_data", c_int, [c_void_p, P_CHAR, P_INT]),
    ("get_pre_send_canfd_data", c_int, [c_void_p, P_CHAR, P_INT]),
    ("get_pre_send_rs485_data", c_int, [c_void_p, P_CHAR, P_INT]),
    ("get_pre_send_can_data", c_int, [c_void_p, P_CHAR, P_INT]),
    ("get_firmware_version", c_int, [c_void_p, P_FLOAT]),
    ("get_serial_number", c_int, [c_void_p, P_CHAR, c_int]),
    ("set_sdo_drive_param", c_int, [c_void_p, c_uint, c_ubyte, c_uint]),
    ("get_sdo_drive_param", c_int, [c_void_p, c_uint, c_ubyte, P_UINT]),
    ("save_sdo_drive_param", c_int, [c_void_p]),
    ("get_dof", c_int, [c_void_p, P_INT, P_INT]),
    ("set_hand_type", c_int, [c_void_p, c_int]),
    ("get_hand_type", c_int, [c_void_p, P_INT]),
    ("set_hand_direction", c_int, [c_void_p, c_int]),
    ("get_hand_direction", c_int, [c_void_p, P_INT]),
    ("set_move_no_home", c_int, [c_void_p, c_int]),
    ("set_angle_conversion_enable", c_int, [c_void_p, c_int]),
    ("get_angle_conversion_enable", c_int, [c_void_p, P_INT]),
    ("set_control_mode", c_int, [c_void_p, c_int, c_int]),
    ("get_control_mode", c_int, [c_void_p, c_int, P_INT]),
    ("set_safe_current_enable", c_int, [c_void_p, c_int]),
    ("get_safe_current_enable", c_int, [c_void_p, P_INT]),
    ("set_home_current", c_int, [c_void_p, c_int]),
    ("get_home_current", c_int, [c_void_p, P_INT]),
    ("set_can_node_id", c_int, [c_void_p, c_int]),
    ("get_can_node_id", c_int, [c_void_p, P_INT]),
    ("set_canfd_arb_baudrate", c_int, [c_void_p, c_int]),
    ("get_canfd_arb_baudrate", c_int, [c_void_p, P_INT]),
    ("set_canfd_data_baudrate", c_int, [c_void_p, c_int]),
    ("get_canfd_data_baudrate", c_int, [c_void_p, P_INT]),
    ("set_rs485_node_id", c_int, [c_void_p, c_int]),
    ("get_rs485_node_id", c_int, [c_void_p, P_INT]),
    ("set_rs485_baudrate", c_int, [c_void_p, c_int]),
    ("get_rs485_baudrate", c_int, [c_void_p, P_INT]),
    ("set_enable", c_int, [c_void_p, c_int, c_int]),
    ("get_enable", c_int, [c_void_p, c_int, P_INT]),
    ("get_position_reached", c_int, [c_void_p, c_int, P_INT]),
    ("get_torque_reached", c_int, [c_void_p, c_int, P_INT]),
    ("set_clear_alarm", c_int, [c_void_p, c_int]),
    ("get_now_alarm", c_int, [c_void_p, c_int, P_INT]),
    ("home_motors", c_int, [c_void_p, c_int]),
    ("get_limit_target_angle", c_int, [c_void_p, c_int, P_FLOAT, P_FLOAT]),
    ("set_target_angle", c_int, [c_void_p, c_int, c_float]),
    ("get_target_angle", c_int, [c_void_p, c_int, P_FLOAT]),
    ("set_target_position", c_int, [c_void_p, c_int, c_int]),
    ("get_target_position", c_int, [c_void_p, c_int, P_INT]),
    ("set_velocity", c_int, [c_void_p, c_int, c_float]),
    ("get_velocity", c_int, [c_void_p, c_int, P_FLOAT]),
    ("set_angular_velocity", c_int, [c_void_p, c_int, c_float]),
    ("get_angular_velocity", c_int, [c_void_p, c_int, P_FLOAT]),
    ("set_position_velocity", c_int, [c_void_p, c_int, c_int]),
    ("get_position_velocity", c_int, [c_void_p, c_int, P_INT]),
    ("set_max_current", c_int, [c_void_p, c_int, c_int]),
    ("get_max_current", c_int, [c_void_p, c_int, P_INT]),
    ("move_motors", c_int, [c_void_p, c_int]),
    ("stop_motors", c_int, [c_void_p, c_int]),
    ("play_gesture", c_int, [c_void_p, c_int, c_int, c_int]),
    ("get_now_status", c_int, [c_void_p, c_int, P_INT]),
    ("get_now_angle", c_int, [c_void_p, c_int, P_FLOAT]),
    ("get_now_position", c_int, [c_void_p, c_int, P_INT]),
    ("get_now_velocity", c_int, [c_void_p, c_int, P_FLOAT]),
    ("get_now_angular_velocity", c_int, [c_void_p, c_int, P_FLOAT]),
    ("get_now_position_velocity", c_int, [c_void_p, c_int, P_INT]),
    ("get_now_current", c_int, [c_void_p, c_int, P_INT]),
    ("sync_position_to_target", c_int, [c_void_p, c_int]),
    ("set_sensor_enable", c_int, [c_void_p, c_int]),
    ("set_sensor_data_format", c_int, [c_void_p, c_int]),
    ("set_sensor_order", c_int, [c_void_p, P_INT, c_int]),
    ("get_finger_sensor_pos", c_int, [c_void_p, c_int, P_FLOAT_PTR, P_FLOAT_PTR, P_INT]),
    ("get_finger_pressure", c_int, [c_void_p, c_int, P_FLOAT_PTR, P_INT]),
    ("set_finger_pressure_reset", c_int, [c_void_p]),
    ("get_finger_normal_force", c_int, [c_void_p, c_int, P_FLOAT]),
    ("get_finger_normal_force_ex", c_int, [c_void_p, c_int, P_FLOAT_PTR, P_INT]),
    ("get_finger_tangential_force", c_int, [c_void_p, c_int, P_FLOAT]),
    ("get_finger_tangential_force_ex", c_int, [c_void_p, c_int, P_FLOAT_PTR, P_INT]),
    ("get_finger_force_direction", c_int, [c_void_p, c_int, P_FLOAT]),
    ("get_finger_force_direction_ex", c_int, [c_void_p, c_int, P_FLOAT_PTR, P_INT]),
    ("get_finger_proximity", c_int, [c_void_p, c_int, P_FLOAT]),
    ("get_finger_proximity_ex", c_int, [c_void_p, c_int, P_FLOAT_PTR, P_INT]),
    ("log_on", None, [c_void_p, c_bool, c_int]),
    ("log_send", c_int, [c_void_p, P_INT, c_int]),
    ("log_recv", c_int, [c_void_p, P_INT, c_int]),
    ("log_reset", None, [c_void_p, c_bool, c_bool]),
    ("log_save", c_int, [c_void_p, c_char_p]),
    ("log_clear", None, [c_void_p]),
)

C_API_SUFFIXES = tuple(suffix for suffix, _, _ in C_API_PROTOTYPES)


class LHandProLibLoader:
    """LHandProLib库加载器"""

    def __init__(self, lib_path: Optional[str] = None):
        """
        初始化库加载器

        Args:
            lib_path: 可选的库文件路径，如果为None则自动查找
        """
        self._lib = None
        self._api_prefix = DEFAULT_C_API_PREFIX
        self._load_library(lib_path)
        self._define_function_prototypes()

    def _find_library(self) -> Path:
        """查找库文件"""
        package_dir = Path(__file__).resolve().parent

        # 根据平台确定库文件名
        if sys.platform == "win32":
            lib_name = f"{DEFAULT_LIBRARY_BASE_NAME}.dll"
        elif sys.platform == "darwin":
            lib_name = f"lib{DEFAULT_LIBRARY_BASE_NAME}.dylib"
        else:
            lib_name = f"lib{DEFAULT_LIBRARY_BASE_NAME}.so"

        # 尝试在常见位置查找库文件
        lib_paths = [
            package_dir / lib_name,
            package_dir / "lib" / lib_name,
            package_dir / "thirdParty/bin" / lib_name,
            package_dir / "thirdParty/lib" / lib_name,
            package_dir.parent / lib_name,
            package_dir.parent / "bin" / lib_name,
            package_dir.parent / "lib" / lib_name,
            package_dir.parent / "build/bin" / lib_name,
            package_dir.parent / "build/lib" / lib_name,
            package_dir.parent / "install/bin" / lib_name,
            package_dir.parent / "install/lib" / lib_name,
            package_dir / "../../../../bin" / lib_name,
            package_dir / "../../../../lib" / lib_name,
            package_dir / "../../../../../bin" / lib_name,
            Path("/usr/local/lib") / lib_name,
            Path("/usr/lib") / lib_name,
        ]

        for path in lib_paths:
            if path.exists():
                return path

        raise FileNotFoundError(f"无法找到库文件 {lib_name}")

    def _load_library(self, lib_path: Optional[str] = None):
        """加载动态库"""
        if lib_path is None:
            lib_path = self._find_library()
        else:
            lib_path = Path(lib_path)

        if not lib_path.exists():
            raise FileNotFoundError(f"指定的库文件不存在: {lib_path}")

        try:
            self._lib = ctypes.CDLL(str(lib_path))
        except Exception as e:
            raise RuntimeError(f"加载库失败: {e}") from e

    def _bind_api_functions(self):
        """Bind configured public API names to historical ctypes attributes."""
        for suffix in C_API_SUFFIXES:
            canonical_name = f"lhandprolib_{suffix}"
            public_name = f"{self._api_prefix}_{suffix}"
            try:
                func = getattr(self._lib, public_name)
            except AttributeError as exc:
                raise RuntimeError(
                    f"Library is missing C API symbol '{public_name}'. "
                    f"Check SDK_C_API_PREFIX/build_config.py."
                ) from exc
            setattr(self._lib, canonical_name, func)

    def _define_function_prototypes(self):
        """定义C函数的原型"""
        self._bind_api_functions()
        for suffix, restype, argtypes in C_API_PROTOTYPES:
            func = getattr(self._lib, f"lhandprolib_{suffix}")
            func.restype = restype
            func.argtypes = argtypes

    @property
    def lib(self):
        """获取底层的ctypes库对象"""
        return self._lib


# 全局单例实例
_global_lhandprolib_lib = None


def get_global_lhandprolib_lib(lib_path: Optional[str] = None) -> LHandProLibLoader:
    """
    获取全局单例LHandProLib实例

    Args:
        lib_path: 可选的库文件路径，仅在第一次调用时有效

    Returns:
        全局LHandProLibLoader实例
    """
    global _global_lhandprolib_lib
    if _global_lhandprolib_lib is None:
        _global_lhandprolib_lib = LHandProLibLoader(lib_path)
    return _global_lhandprolib_lib
