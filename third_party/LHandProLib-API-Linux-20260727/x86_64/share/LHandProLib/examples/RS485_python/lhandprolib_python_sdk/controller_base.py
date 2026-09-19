"""Shared controller behavior for all communication modes."""

import time
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple

from .defaults import (
    DEFAULT_CANFD_NODE_ID,
    DEFAULT_ENABLE_HOME_CHECK,
    DEFAULT_HAND_TYPE,
    DEFAULT_RS485_BAUD_RATE,
    DEFAULT_RS485_NODE_ID,
)
from .lhandprolib_wrapper import PyLHandProLib, LHandProLibError


class BaseLHandProController(ABC):
    """Common controller behavior shared by transport-specific controllers."""

    def __init__(
        self,
        communication_mode: str,
        hand_type: int = DEFAULT_HAND_TYPE,
        enable_home_check: bool = DEFAULT_ENABLE_HOME_CHECK,
        canfd_node_id: int = DEFAULT_CANFD_NODE_ID,
        canfd_driver: Optional[str] = None,
        rs485_baud_rate: int = DEFAULT_RS485_BAUD_RATE,
        rs485_node_id: int = DEFAULT_RS485_NODE_ID,
    ):
        self.communication_mode = communication_mode.upper()
        self.hand_type = hand_type
        self.enable_home_check = enable_home_check
        self.canfd_node_id = canfd_node_id
        self.canfd_driver = canfd_driver
        self.rs485_baud_rate = rs485_baud_rate
        self.rs485_node_id = rs485_node_id

        self.sdk_handle: Optional[PyLHandProLib] = None
        self.is_connected = False
        self.dof_total = 0
        self.dof_active = 0

    def connect(
        self,
        enable_motors: bool = True,
        home_motors: bool = True,
        home_wait_time: float = 5.0,
        device_index: Optional[int] = None,
        slave_id: Optional[int] = None,
        auto_select: bool = True,
        canfd_nom_baudrate: int = 1000000,
        canfd_dat_baudrate: int = 5000000,
        rs485_port_name: Optional[str] = None,
        rs485_baud_rate: Optional[int] = None,
        rs485_node_id: Optional[int] = None,
        hand_type: Optional[int] = None,
        enable_home_check: Optional[bool] = None,
        canfd_node_id: Optional[int] = None,
        canfd_driver: Optional[str] = None,
    ) -> bool:
        """Create the native SDK object, connect transport, then run common init."""

        if self.is_connected or self.sdk_handle is not None:
            print("Controller already owns connection resources; disconnect first")
            return False

        try:
            self.sdk_handle = PyLHandProLib()

            if hand_type is not None:
                self.hand_type = hand_type
            if enable_home_check is not None:
                self.enable_home_check = enable_home_check
            if canfd_node_id is not None:
                self.canfd_node_id = canfd_node_id
            if canfd_driver is not None:
                self.canfd_driver = canfd_driver
            if rs485_baud_rate is not None:
                self.rs485_baud_rate = rs485_baud_rate
            if rs485_node_id is not None:
                self.rs485_node_id = rs485_node_id

            connected = self._connect_transport(
                enable_motors=enable_motors,
                home_motors=home_motors,
                home_wait_time=home_wait_time,
                device_index=device_index,
                slave_id=slave_id,
                auto_select=auto_select,
                canfd_nom_baudrate=canfd_nom_baudrate,
                canfd_dat_baudrate=canfd_dat_baudrate,
                rs485_port_name=rs485_port_name,
                canfd_driver=self.canfd_driver,
            )
            if not connected or not self.is_connected:
                self._teardown_resources()
                return False

            self.sdk_handle.set_hand_type(self.hand_type)
            self.sdk_handle.set_move_no_home(0 if self.enable_home_check else 1)
            if not self.is_connected:
                self._teardown_resources()
                return False
            return True
        except (LHandProLibError, Exception) as exc:
            print(f"Operation failed: {exc}")
            self._teardown_resources()
            return False

    @abstractmethod
    def _connect_transport(
        self,
        enable_motors: bool,
        home_motors: bool,
        home_wait_time: float,
        device_index: Optional[int],
        slave_id: Optional[int],
        auto_select: bool,
        canfd_nom_baudrate: int,
        canfd_dat_baudrate: int,
        rs485_port_name: Optional[str],
        canfd_driver: Optional[str] = None,
    ) -> bool:
        """Connect the transport and call _common_initialization on success."""

    @abstractmethod
    def _cleanup_communication_resources(self) -> None:
        """Release transport-specific resources."""

    def _reset_sdk(self) -> None:
        if self.sdk_handle:
            try:
                self.sdk_handle.close()
            except Exception:
                pass
            self.sdk_handle = None

    def _teardown_resources(self) -> None:
        """Stop transport receive paths before destroying the native SDK."""
        self.is_connected = False
        try:
            self._cleanup_communication_resources()
        except Exception as exc:
            print(f"Communication cleanup failed: {exc}")
        finally:
            self._reset_sdk()

    def _common_initialization(
        self, enable_motors: bool, home_motors: bool, home_wait_time: float
    ) -> bool:
        self.dof_total, self.dof_active = self.sdk_handle.get_dof()
        print(f"DOF: total {self.dof_total}, active {self.dof_active}")

        if enable_motors:
            self.sdk_handle.set_enable(0, True)
            print("Waiting for motors to enable")
            time.sleep(1.0)

        if home_motors:
            print("Homing motors")
            self.sdk_handle.home_motors(0)
            time.sleep(home_wait_time)

        return True

    def disconnect(self) -> None:
        # A transport monitor may clear is_connected after detecting a link
        # failure while the native SDK and transport objects still need cleanup.
        if not self.is_connected and not self.sdk_handle:
            return

        print("Disconnecting...")
        self._teardown_resources()
        print("Disconnected")

    def move_to_positions(
        self,
        positions: List[int],
        velocity: int = 20000,
        max_current: int = 1000,
        wait_time: float = 1.0,
    ) -> bool:
        if not self.is_connected or not self.sdk_handle:
            print("Device is not connected")
            return False
        if len(positions) != self.dof_active:
            print(
                f"Position count mismatch: expected {self.dof_active}, got {len(positions)}"
            )
            return False

        try:
            for index, position in enumerate(positions, start=1):
                self.sdk_handle.set_target_position(index, position)
                self.sdk_handle.set_position_velocity(index, velocity)
                self.sdk_handle.set_max_current(index, max_current)

            self.sdk_handle.move_motors(0)
            print(f"Move command sent successfully: positions={positions}")
            if wait_time > 0:
                time.sleep(wait_time)
            return True
        except Exception as exc:
            print(f"Position move failed: {exc}")
            return False

    def move_to_angles(
        self,
        angles: List[float],
        angular_velocity: float = 200.0,
        max_current: int = 1000,
        wait_time: float = 1.0,
    ) -> bool:
        if not self.is_connected or not self.sdk_handle:
            print("Device is not connected")
            return False
        if len(angles) != self.dof_active:
            print(f"Angle count mismatch: expected {self.dof_active}, got {len(angles)}")
            return False

        try:
            for index, angle in enumerate(angles, start=1):
                self.sdk_handle.set_target_angle(index, angle)
                self.sdk_handle.set_angular_velocity(index, angular_velocity)
                self.sdk_handle.set_max_current(index, max_current)

            self.sdk_handle.move_motors(0)
            print(f"Move command sent successfully: angles={angles}")
            if wait_time > 0:
                time.sleep(wait_time)
            return True
        except Exception as exc:
            print(f"Angle move failed: {exc}")
            return False

    def move_sequence(
        self,
        positions_list: List[List[int]],
        velocity: int = 20000,
        max_current: int = 1000,
        wait_time: float = 1.0,
    ) -> bool:
        if not self.is_connected or not self.sdk_handle:
            print("Device is not connected")
            return False

        try:
            for index, positions in enumerate(positions_list):
                if not self.move_to_positions(positions, velocity, max_current, wait_time):
                    print(f"Sequence step {index} failed")
                    return False
                print(f"line: {index} positions: {positions} ok")
            return True
        except Exception as exc:
            print(f"Sequence move failed: {exc}")
            return False

    def enable_motors(self, enable: bool = True) -> None:
        if not self.is_connected or not self.sdk_handle:
            print("Device is not connected")
            return

        try:
            self.sdk_handle.set_enable(0, enable)
            print(f"Motors {'enabled' if enable else 'disabled'} successfully")
        except Exception as exc:
            print(f"Set motor enable failed: {exc}")

    def home(self, wait_time: float = 5.0) -> None:
        if not self.is_connected or not self.sdk_handle:
            print("Device is not connected")
            return

        try:
            print("Homing motors")
            self.sdk_handle.home_motors(0)
            time.sleep(wait_time)
            print("Homing complete")
        except Exception as exc:
            print(f"Homing failed: {exc}")

    def stop_motors(self) -> None:
        if not self.is_connected or not self.sdk_handle:
            print("Device is not connected")
            return

        try:
            self.sdk_handle.stop_motors(0)
            print("All motors stopped")
        except Exception as exc:
            print(f"Stop motors failed: {exc}")

    def move_to_zero(
        self,
        velocity: int = 20000,
        max_current: int = 1000,
        wait_time: float = 1.0,
    ) -> bool:
        return self.move_to_positions([0] * self.dof_active, velocity, max_current, wait_time)

    def get_dof(self) -> Tuple[int, int]:
        return self.dof_total, self.dof_active

    def clear_alarm(self) -> None:
        if not self.is_connected or not self.sdk_handle:
            print("Device is not connected")
            return

        try:
            self.sdk_handle.set_clear_alarm(0)
            print("All motor alarms cleared")
        except Exception as exc:
            print(f"Clear alarm failed: {exc}")

    def get_alarm(self) -> bool:
        if not self.is_connected or not self.sdk_handle:
            print("Device is not connected")
            return False

        try:
            for motor_id in range(1, self.dof_active + 1):
                if self.sdk_handle.get_now_alarm(motor_id) == 1:
                    print(f"Motor {motor_id} alarm")
                    return True
            return False
        except Exception as exc:
            print(f"Get alarm failed: {exc}")
            return False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False
