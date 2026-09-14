#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LHandPro RS485 motion loop example."""

import sys
from pathlib import Path

CURRENT_DIR = Path(__file__).resolve().parent
ROOT_DIR = CURRENT_DIR.parents[0]
for candidate in (CURRENT_DIR, ROOT_DIR):
    candidate_str = str(candidate)
    if candidate_str not in sys.path:
        sys.path.insert(0, candidate_str)

from lhandprolib_python_sdk.controller import LHandProController


def main():
    print("LHandPro RS485 循环运动示例")
    print("=" * 50)

    with LHandProController(communication_mode="RS485") as controller:
        try:
            print("\n正在连接 RS485 设备...")
            connected = controller.connect(
                enable_motors=True,
                home_motors=True,
                home_wait_time=5.0,
            )
            if not connected:
                print("设备连接失败，请检查硬件连接和 RS485 配置")
                return 1

            print("设备连接成功")
            dof_total, dof_active = controller.get_dof()
            print("\n设备信息:")
            print(f"  - 总自由度: {dof_total}")
            print(f"  - 主动自由度: {dof_active}")

            velocity = 20000
            max_current = 1000
            wait_time = 1.0
            positions = [
                [10000, 10000, 0, 0, 0, 0][:dof_active],
                [0, 0, 0, 0, 0, 0][:dof_active],
                [0, 0, 10000, 10000, 10000, 10000][:dof_active],
                [0, 0, 0, 0, 0, 0][:dof_active],
            ]

            print("\n开始执行循环运动")
            print(f"运动速度: {velocity}, 最大电流: {max_current}")
            print("按 Ctrl+C 退出程序")

            cycle_count = 0
            while True:
                cycle_count += 1
                print(f"\n=== 循环 {cycle_count} ===")
                for index, pos_list in enumerate(positions, start=1):
                    print(f"  移动到位置 {index}: {pos_list}")
                    success = controller.move_to_positions(
                        positions=pos_list,
                        velocity=velocity,
                        max_current=max_current,
                        wait_time=wait_time,
                    )
                    print(f"  {'成功' if success else '失败'}")
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        except Exception as exc:
            print(f"\n程序运行出错: {exc}")
            return 1
        finally:
            print("\n停止所有电机...")
            controller.stop_motors()

    print("\n" + "=" * 50)
    print("程序执行完成")
    return 0


if __name__ == "__main__":
    sys.exit(main())
