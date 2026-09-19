"""RS485 controller implementation."""

from typing import Optional

from .controller_base import BaseLHandProController
from .lhandprolib_wrapper import LCN_RS485


class RS485Controller(BaseLHandProController):
    def __init__(self, **kwargs):
        kwargs.pop("communication_mode", None)
        super().__init__(communication_mode="RS485", **kwargs)
        self.serial_port = None

    def _rs485_send_callback(self, data: bytes) -> bool:
        if self.serial_port and self.is_connected:
            try:
                return self.serial_port.write(data) > 0
            except Exception as exc:
                print(f"RS485 send failed: {exc}")
        return False

    def _rs485_receive_callback(self, data: bytes) -> None:
        if self.sdk_handle and self.is_connected:
            self.sdk_handle.set_rs485_data_decode(data)

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
        from serial_port import SerialPort

        self.serial_port = SerialPort()
        print("Using RS485 communication")

        port_name = rs485_port_name
        if port_name is None:
            available_ports = self.serial_port.scan_available_ports()
            if not available_ports:
                print("No serial port found")
                self.serial_port = None
                return False

            print("Available serial ports:")
            for index, port in enumerate(available_ports):
                print(f"  [{index}] {port}")

            if device_index is not None:
                if device_index < len(available_ports):
                    port_name = available_ports[device_index]
                    print(f"Auto selected serial port by device index: {port_name}")
                else:
                    print(
                        f"Device index {device_index} exceeds port count {len(available_ports)}"
                    )
                    self.serial_port = None
                    return False
            elif len(available_ports) == 1 or auto_select:
                port_name = available_ports[0]
                print(f"Auto selected serial port: {port_name}")
            else:
                while True:
                    try:
                        user_input = input(
                            f"Select serial port [0-{len(available_ports) - 1}]: "
                        ).strip()
                        port_index = 0 if user_input == "" else int(user_input)
                        if 0 <= port_index < len(available_ports):
                            port_name = available_ports[port_index]
                            break
                    except ValueError:
                        pass
                    print("Please enter a valid serial port index")

        print(f"Opening serial port {port_name} at {self.rs485_baud_rate}bps")
        if not self.serial_port.open(port_name, self.rs485_baud_rate):
            print(f"Open serial port failed: {port_name}")
            self.serial_port = None
            return False

        self.is_connected = True
        self.sdk_handle.set_send_rs485_callback(self._rs485_send_callback)
        self.serial_port.set_read_callback(self._rs485_receive_callback)
        self.sdk_handle.initial_ex(LCN_RS485, self.rs485_node_id)
        return self._common_initialization(enable_motors, home_motors, home_wait_time)

    def _cleanup_communication_resources(self) -> None:
        if self.serial_port:
            try:
                self.serial_port.close()
            finally:
                self.serial_port = None
