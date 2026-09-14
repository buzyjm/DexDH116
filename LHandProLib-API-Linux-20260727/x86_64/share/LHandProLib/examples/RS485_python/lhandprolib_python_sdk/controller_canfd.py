"""CANFD controller implementation."""

from typing import Optional

from .controller_base import BaseLHandProController
from .lhandprolib_wrapper import LCN_CANFD


class CANFDController(BaseLHandProController):
    def __init__(self, **kwargs):
        kwargs.pop("communication_mode", None)
        super().__init__(communication_mode="CANFD", **kwargs)
        self.canfd = None

    def _canfd_send_callback(self, msg_id: int, data: bytes, is_extended: int = 0) -> bool:
        if self.canfd and self.is_connected:
            try:
                return self.canfd.send(msg_id, data)
            except Exception as exc:
                print(f"CANFD send failed: {exc}")
        return False

    def _canfd_receive_callback(self, msg) -> None:
        if self.sdk_handle and self.is_connected:
            self.sdk_handle.set_canfd_data_decode(msg["id"], msg["data"])

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
        from canfd_lib import CANFD

        self.canfd = CANFD(driver=self.canfd_driver)
        print("Using CANFD communication")

        device_count = self.canfd.scan()
        print(f"Found CANFD devices: {device_count}")
        if device_count == 0:
            print("No CANFD device found")
            self.canfd = None
            return False

        if device_index is None:
            if device_count == 1 or auto_select:
                device_index = 0
                print(f"Auto selected device index: {device_index}")
            else:
                print(f"Select device [0 - {device_count - 1}]")
                while True:
                    try:
                        user_input = input(">>> ").strip()
                        device_index = 0 if user_input == "" else int(user_input)
                        if 0 <= device_index < device_count:
                            break
                    except ValueError:
                        pass
                    print(f"Please enter a number in [0 - {device_count - 1}]")
        elif not 0 <= device_index < device_count:
            print(f"Invalid device index: {device_index}")
            self.canfd = None
            return False

        print(
            "Connecting CANFD device "
            f"(nom={canfd_nom_baudrate}bps, data={canfd_dat_baudrate}bps)"
        )
        if not self.canfd.connect(
            device_index=device_index,
            channel_index=0,
            nom_baudrate=canfd_nom_baudrate,
            dat_baudrate=canfd_dat_baudrate,
        ):
            print("CANFD device connection failed")
            self.canfd = None
            return False

        self.is_connected = True
        self.canfd.set_receive_callback(self._canfd_receive_callback)
        self.sdk_handle.set_send_canfd_callback(self._canfd_send_callback)
        self.sdk_handle.initial_ex(LCN_CANFD, self.canfd_node_id)
        return self._common_initialization(enable_motors, home_motors, home_wait_time)

    def _cleanup_communication_resources(self) -> None:
        if self.canfd:
            try:
                self.canfd.disconnect()
            finally:
                self.canfd = None
