"""EtherCAT controller implementation."""

import threading
import time
from typing import Optional

from .controller_base import BaseLHandProController
from .lhandprolib_wrapper import LCN_ECAT


class EtherCATController(BaseLHandProController):
    def __init__(self, **kwargs):
        kwargs.pop("communication_mode", None)
        super().__init__(communication_mode="ECAT", **kwargs)
        self.slave_id = None
        self.ec_master = None
        self.stop_flag = None
        self.monitor_thread = None

    def _ec_send_callback(self, data: bytes) -> bool:
        if self.ec_master and self.is_connected:
            return self.ec_master.setSlaveOutputs(self.slave_id, data, len(data))
        print("EC master not initialized")
        return False

    def _monitor_thread_func(self, stop_flag: threading.Event) -> None:
        try:
            while not stop_flag.is_set() and self.is_connected:
                ec_master = self.ec_master
                sdk_handle = self.sdk_handle
                if ec_master:
                    if not ec_master.isConnected():
                        print(
                            "EtherCAT connection lost: "
                            f"{ec_master.getLastError() or 'process-data exchange stopped'}"
                        )
                        self.is_connected = False
                        break
                    input_size = ec_master.getSlaveInputSize(self.slave_id)
                    inputs = ec_master.getSlaveInputs(self.slave_id, input_size)
                    if inputs is not None and sdk_handle:
                        sdk_handle.set_tpdo_data_decode(inputs)
                time.sleep(0.01)
        except Exception as exc:
            if not stop_flag.is_set():
                print(f"EtherCAT monitor failed: {exc}")
                self.is_connected = False

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
        from ethercat_master import EthercatMaster

        self.ec_master = EthercatMaster()
        print("Using EtherCAT communication (100M)")

        names = self.ec_master.scanNetworkInterfaces()
        print(f"Found network interfaces: {len(names)}")
        if len(names) == 0:
            print("No network interface found")
            self.ec_master = None
            return False

        channel_index = device_index
        if channel_index is None:
            if len(names) == 1 or auto_select:
                channel_index = 0
                print(f"Auto selected network interface: {names[channel_index]}")
            else:
                print(f"Select interface [0 - {len(names) - 1}]")
                for index, name in enumerate(names):
                    print(f"  [{index}] {name}")
                while True:
                    try:
                        user_input = input(">>> ").strip()
                        channel_index = 0 if user_input == "" else int(user_input)
                        if 0 <= channel_index < len(names):
                            break
                    except ValueError:
                        pass
                    print(f"Please enter a number in [0 - {len(names) - 1}]")
        elif not 0 <= channel_index < len(names):
            print(f"Invalid network interface index: {channel_index}")
            self.ec_master = None
            return False

        if not self.ec_master.init(channel_index):
            print(
                "EtherCAT initialization failed: "
                f"{self.ec_master.getLastError() or 'unknown error'}"
            )
            self.ec_master = None
            return False

        if not self.ec_master.start():
            print(
                "EtherCAT start failed: "
                f"{self.ec_master.getLastError() or 'unknown error'}"
            )
            self._cleanup_communication_resources()
            return False

        slave_info_list = self.ec_master.getSlaveInfoList()
        slave_count = len(slave_info_list)
        print(f"Found EtherCAT slaves: {slave_count}")
        if slave_count == 0:
            print("No EtherCAT slave found")
            self._cleanup_communication_resources()
            return False

        selected_slave_id = slave_id
        if selected_slave_id is None:
            if slave_count == 1 or auto_select:
                selected_slave_id = 1
                print(
                    "Auto selected EtherCAT slave: "
                    f"1 ({slave_info_list[0].name})"
                )
            else:
                print(f"Select EtherCAT slave [1 - {slave_count}]")
                for slave_info in slave_info_list:
                    print(f"  [{slave_info.index}] {slave_info.name}")
                while True:
                    try:
                        user_input = input(">>> ").strip()
                        selected_slave_id = 1 if user_input == "" else int(user_input)
                        if 1 <= selected_slave_id <= slave_count:
                            break
                    except ValueError:
                        pass
                    print(f"Please enter a number in [1 - {slave_count}]")
        elif not 1 <= selected_slave_id <= slave_count:
            print(
                f"Invalid EtherCAT slave ID {selected_slave_id}; "
                f"detected slave count: {slave_count}"
            )
            self._cleanup_communication_resources()
            return False

        self.slave_id = selected_slave_id
        selected_info = slave_info_list[self.slave_id - 1]
        print(f"Using EtherCAT slave {self.slave_id}: {selected_info.name}")

        self.ec_master.run()
        if not self.ec_master.isConnected():
            print(
                "EtherCAT process-data thread failed to start: "
                f"{self.ec_master.getLastError() or 'unknown error'}"
            )
            self._cleanup_communication_resources()
            return False

        self.is_connected = True
        self.sdk_handle.set_send_rpdo_callback(self._ec_send_callback)

        self.stop_flag = threading.Event()
        self.monitor_thread = threading.Thread(
            target=self._monitor_thread_func,
            args=(self.stop_flag,),
            daemon=True,
        )
        self.monitor_thread.start()

        self.sdk_handle.initial(LCN_ECAT)
        return self._common_initialization(enable_motors, home_motors, home_wait_time)

    def _cleanup_communication_resources(self) -> None:
        stop_flag = self.stop_flag
        monitor_thread = self.monitor_thread
        if stop_flag:
            stop_flag.set()
        if (
            monitor_thread
            and monitor_thread.is_alive()
            and monitor_thread is not threading.current_thread()
        ):
            monitor_thread.join()
        self.monitor_thread = None
        self.stop_flag = None
        if self.ec_master:
            try:
                self.ec_master.stop()
                time.sleep(0.1)
            finally:
                self.ec_master = None
        self.slave_id = None
