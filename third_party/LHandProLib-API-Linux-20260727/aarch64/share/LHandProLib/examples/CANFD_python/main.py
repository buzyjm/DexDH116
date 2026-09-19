#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LHandPro CANFD motion / sensor example."""

import sys
import time
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
ROOT_DIR = CURRENT_DIR.parents[0]
for candidate in (CURRENT_DIR, ROOT_DIR):
    candidate_str = str(candidate)
    if candidate_str not in sys.path:
        sys.path.insert(0, candidate_str)

from lhandprolib_python_sdk.controller import LHandProController
from lhandprolib_python_sdk.lhandprolib_wrapper import (
    LSS_FINGER_1_1, LSS_FINGER_2_1, LSS_FINGER_3_1,
    LSS_FINGER_4_1, LSS_FINGER_5_1, LSS_HAND_PALM,
)

ALL_SENSOR_LIST = [
    LSS_FINGER_1_1, LSS_FINGER_2_1, LSS_FINGER_3_1,
    LSS_FINGER_4_1, LSS_FINGER_5_1, LSS_HAND_PALM,
]

POSITION_VELOCITY = 20000
POSITION_MAX_CURRENT = 1000
SENSOR_LOOP_DELAY_S = 0.05
MOTION_STEP_DELAY_S = 1.0

# C++ 版本的运动位置表
POSITIONS = [
    [10000, 0, 0, 0, 0, 0],
    [0, 10000, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 10000, 0, 0, 0],
    [0, 0, 0, 10000, 0, 0],
    [0, 0, 0, 0, 10000, 0],
    [0, 0, 0, 0, 0, 10000],
]


def _get_node_id() -> int:
    print("\nPlease enter CANFD node id (default 1) [1 - 255]")
    raw = input(">>> ").strip()
    if raw == "":
        return 1
    try:
        val = int(raw)
        return val if 1 <= val <= 255 else 1
    except ValueError:
        return 1


def _get_sensor_mode() -> int:
    print(
        "\nSelect test mode\n"
        "  0 = Motor motion test\n"
        "  1 = Sensor data test\n"
        "(default 0) [0 - 1]"
    )
    raw = input(">>> ").strip()
    if raw == "":
        return 0
    try:
        val = int(raw)
        return val if val in (0, 1) else 0
    except ValueError:
        return 0


def _print_sensor_data(controller, sensor_id: int, pressure_only: bool) -> None:
    """打印单个传感器数据 (参照 C++ EtherCAT Sensor 格式)"""
    pressure = controller.sdk_handle.get_finger_pressure(sensor_id)

    line = f"  [Sensor {sensor_id:2d}] pressure["
    for i, v in enumerate(pressure):
        if i > 0:
            line += ","
        line += f"{v:.1f}"
    line += "]"

    if not pressure_only:
        nf = controller.sdk_handle.get_finger_normal_force_ex(sensor_id)
        tf = controller.sdk_handle.get_finger_tangential_force_ex(sensor_id)
        dr = controller.sdk_handle.get_finger_force_direction_ex(sensor_id)

        line += " NF["
        for i, v in enumerate(nf):
            if i > 0:
                line += ","
            line += f"{v:.2f}"
        line += "]"

        line += " TF["
        for i, v in enumerate(tf):
            if i > 0:
                line += ","
            line += f"{v:.2f}"
        line += "]"

        line += " DIR["
        for i, v in enumerate(dr):
            if i > 0:
                line += ","
            line += f"{v:.1f}"
        line += "]"

    # 用 ANSI 控制码实现原地刷新
    sys.stdout.write(f"\033[2K{line}\n")


def _run_motor_test(controller, dof_active: int) -> None:
    """电机运动测试 (与 C++ 版本相同的运动队列)"""
    actual_positions = [[p[i] for i in range(dof_active)] for p in POSITIONS]

    print("Starting cyclic motion test...")
    print("Press Ctrl+C to exit.")

    cycle_count = 0
    while True:
        cycle_count += 1
        for i, pos_list in enumerate(actual_positions):
            try:
                for axis, pos in enumerate(pos_list):
                    joint_id = axis + 1
                    controller.sdk_handle.set_target_position(joint_id, pos)
                    controller.sdk_handle.set_position_velocity(
                        joint_id, POSITION_VELOCITY
                    )
                    controller.sdk_handle.set_max_current(
                        joint_id, POSITION_MAX_CURRENT
                    )

                controller.sdk_handle.move_motors(0)

                print(
                    f"line: {i}  positions: "
                    + " ".join(str(p) for p in pos_list)
                    + "  retn: 0"
                )
            except Exception as exc:
                print(
                    f"line: {i}  positions: "
                    + " ".join(str(p) for p in pos_list)
                    + f"  retn: {exc}"
                )

            time.sleep(MOTION_STEP_DELAY_S)


def _run_sensor_test(controller) -> None:
    """传感器数据测试 (参照 C++ EtherCAT Sensor 原地刷新风格)"""
    controller.sdk_handle.set_sensor_enable(True)

    hand_type = controller.sdk_handle.get_hand_type()
    pressure_only = (hand_type == 1)  # LAC_DOF_6_S

    active_count = len(ALL_SENSOR_LIST)
    total_lines = 1 + active_count

    print("Starting sensor data test...")
    print("Press Ctrl+C to exit.")

    first_frame = True
    try:
        while True:
            if not first_frame:
                sys.stdout.write(f"\033[{total_lines}A")
            first_frame = False

            sys.stdout.write(
                "\033[2K----------------------------------------\n"
            )
            for sid in ALL_SENSOR_LIST:
                _print_sensor_data(controller, sid, pressure_only)
            sys.stdout.flush()

            time.sleep(SENSOR_LOOP_DELAY_S)
    finally:
        controller.sdk_handle.set_sensor_enable(False)


def main() -> int:
    print("LHandPro CANFD 运动/传感器示例")
    print("=" * 50)

    node_id = _get_node_id()
    sensor_mode = _get_sensor_mode()

    enable_motors = (sensor_mode == 0)
    home_motors = (sensor_mode == 0)

    with LHandProController(
        communication_mode="CANFD",
        canfd_node_id=node_id,
    ) as controller:
        try:
            print("\n正在连接 CANFD 设备...")
            connected = controller.connect(
                enable_motors=enable_motors,
                home_motors=home_motors,
                home_wait_time=5.0,
                canfd_nom_baudrate=1000000,
                canfd_dat_baudrate=5000000,
            )
            if not connected:
                print("设备连接失败，请检查硬件连接和 CANFD 配置")
                return 1

            print("设备连接成功")
            dof_total, dof_active = controller.get_dof()
            print(f"DOF: total {dof_total}, active {dof_active}")

            if sensor_mode == 0:
                _run_motor_test(controller, dof_active)
            else:
                _run_sensor_test(controller)

        except KeyboardInterrupt:
            print("\n程序被用户中断")
        except Exception as exc:
            print(f"\n程序运行出错: {exc}")
            return 1

    print("\n" + "=" * 50)
    print("程序执行完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
