"""Thread-safe PySOEM wrapper matching the C++ EthercatMaster contract."""

import atexit
import logging
import sys
import threading
import time
import weakref
from enum import IntEnum
from typing import List, Optional

import pysoem


_LOGGER = logging.getLogger(__name__)
ETHERCAT_DEBUG = False

_PROCESS_CYCLE_SECONDS = 0.001
_PROCESS_DATA_TIMEOUT_US = 20_000
_SAFE_OP_TIMEOUT_US = 8_000_000
_STATE_CHECK_TIMEOUT_US = 2_000
_SDO_TIMEOUT_US = 2_100_000
_MAX_OP_ATTEMPTS = 2_000
_MAX_CONSECUTIVE_LOST_FRAMES = 10
_MAX_NONE_STATE_COUNT = 3
_PYSOEM_IOMAP_CAPACITY = 4_096

_LSLQ_DH116_PRODUCT_ID = 0x00660000
_LSLQ_DH116_REVISION = 0x00000001
_LSLQ_DH116_OUTPUT_BYTES = 64
_LSLQ_DH116_INPUT_BYTES = 192
_SM2_REGISTER = 0x0810
_SM3_REGISTER = 0x0818
_SM_READ_FAILED = object()


def _as_text(value) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="ignore")
    return str(value)


def _state_to_string(state: int) -> str:
    base_state = int(state) & 0x0F
    return {
        pysoem.INIT_STATE: "INIT",
        pysoem.PREOP_STATE: "PRE_OP",
        pysoem.SAFEOP_STATE: "SAFE_OP",
        pysoem.OP_STATE: "OP",
    }.get(base_state, "UNKNOWN")


def _al_status_to_string(status: int) -> str:
    try:
        return _as_text(pysoem.al_status_code_to_string(int(status)))
    except Exception:
        return "Unknown AL status"


def _stop_master_at_exit(master_ref) -> None:
    master = master_ref()
    if master is not None:
        try:
            master.stop()
        except Exception:
            pass


class SlaveInfo:
    """Snapshot of the public state of one EtherCAT slave."""

    def __init__(
        self,
        index: int,
        name: str,
        state: int = 0,
        al_status: int = 0,
        al_status_str: Optional[str] = None,
        is_lost: bool = False,
    ):
        self.index = int(index)
        self.name = _as_text(name)
        self.state = int(state)
        self.al_status = int(al_status)
        self.al_status_str = (
            _al_status_to_string(self.al_status)
            if al_status_str is None
            else _as_text(al_status_str)
        )
        self.is_lost = bool(is_lost)

    def toString(self) -> str:
        lost_suffix = " [LOST]" if self.is_lost else ""
        return (
            f"Slave {self.index:2d} [{self.name[:16]:<16}] "
            f"State: {_state_to_string(self.state):>8} (0x{self.state:02X}) "
            f"AL: 0x{self.al_status:04X} ({self.al_status_str}){lost_suffix}"
        )

    def __str__(self) -> str:
        return self.toString()

    def __repr__(self) -> str:
        return f"SlaveInfo({self.toString()})"


class EthercatState(IntEnum):
    Disconnected = 0
    Initializing = 1
    SafeOperational = 2
    Operational = 3
    Error = 4


class EthercatMaster:
    """EtherCAT master wrapper with the same lifecycle as the C++ example.

    The lifecycle is ``scanNetworkInterfaces`` -> ``init`` -> ``start`` ->
    ``run`` -> ``stop``. User-facing PDO writes are staged and are committed by
    the process-data thread at a cycle boundary.
    """

    EthercatState = EthercatState

    def __init__(self):
        self.master = pysoem.Master()
        self.slaves = []
        self._input_size = 0
        self._output_size = 0
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.ifname: Optional[str] = None

        self._interfaces: List[str] = []
        self._interface_descriptions: List[str] = []
        self._slave_info_list: List[SlaveInfo] = []
        self._slave_output_sizes: List[int] = []
        self._slave_input_sizes: List[int] = []

        self._initialized = False
        self._started = False
        self._master_open = False
        self._connected_index = -1
        self._connection_lost = False
        self._none_state_count = 0
        self._roundtrip_time_us = 0
        self._lost_frames = 0
        self._dh116_mapping_error = ""

        self._pending_outputs: List[bytearray] = []
        self._committed_outputs: List[bytes] = []
        self._output_dirty: List[bool] = []

        self._lifecycle_lock = threading.RLock()
        self._soem_lock = threading.RLock()
        self._io_lock = threading.RLock()
        self._output_lock = threading.RLock()
        self._error_lock = threading.RLock()
        self._state_lock = threading.RLock()
        self._stats_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._last_error = ""

        self._configure_master_timeouts()
        atexit.register(_stop_master_at_exit, weakref.ref(self))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
        return False

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass

    def _configure_master_timeouts(self) -> None:
        try:
            # PySOEM 1.1.12 otherwise holds the GIL during blocking SOEM calls.
            # The wrapper serializes access explicitly with _soem_lock.
            self.master.always_release_gil = True
            self.master.sdo_read_timeout = _SDO_TIMEOUT_US
            self.master.sdo_write_timeout = _SDO_TIMEOUT_US
        except (AttributeError, TypeError):
            # Test doubles and older PySOEM builds may not expose these fields.
            pass

    def getLastError(self) -> str:
        with self._error_lock:
            return self._last_error if hasattr(self, "_last_error") else ""

    def _clear_last_error(self) -> None:
        with self._error_lock:
            self._last_error = ""

    def _set_last_error(self, phase: str, error: Optional[BaseException] = None) -> None:
        message = phase if error is None else f"{phase}: {error}"
        with self._error_lock:
            self._last_error = message
        _LOGGER.error(message)

    def scanNetworkInterfaces(self) -> List[str]:
        """Scan physical adapters, retaining device names for a later init()."""
        self._interfaces = []
        self._interface_descriptions = []

        try:
            adapters = pysoem.find_adapters() or []
        except Exception as exc:
            self._set_last_error("Scan network interfaces", exc)
            return []

        if sys.platform.startswith("win"):
            exclude_keywords = (
                "wan miniport",
                "wi-fi",
                "wifi",
                "wireless",
                "bluetooth",
                "vmware",
                "virtual",
                "loopback",
                "tap-",
                "vpn",
                "wintun",
                "teredo",
                "isatap",
            )
        else:
            exclude_keywords = (
                "lo",
                "docker",
                "veth",
                "br-",
                "virbr",
                "vmnet",
                "tap",
                "tun",
                "wlan",
                "wlp",
                "wlx",
                "wwan",
                "vboxnet",
                "p2p",
                "teredo",
                "isatap",
            )

        for adapter in adapters:
            name = _as_text(adapter.name)
            description = _as_text(adapter.desc)
            name_lower = name.lower()
            description_lower = description.lower()

            should_exclude = any(keyword in name_lower for keyword in exclude_keywords)
            if sys.platform.startswith("win"):
                should_exclude = should_exclude or any(
                    keyword in description_lower for keyword in exclude_keywords
                )
            if should_exclude:
                continue

            self._interfaces.append(name)
            self._interface_descriptions.append(description or name)

        self._clear_last_error()
        return list(self._interface_descriptions)

    def _resolve_interface(self, channel_index: int) -> bool:
        if channel_index < 0 or channel_index >= len(self._interfaces):
            self._set_last_error("Network interface selection")
            return False

        self._connected_index = int(channel_index)
        self.ifname = self._interfaces[channel_index]
        return True

    def probeSlaves(self, channel_index: int, slaves: List[SlaveInfo]) -> bool:
        """Probe an adapter through an independent temporary master."""
        self._clear_last_error()
        slaves.clear()

        with self._lifecycle_lock:
            if self.running or self._initialized:
                self._set_last_error("Already connected")
                return False
            if channel_index < 0 or channel_index >= len(self._interfaces):
                self._set_last_error("Network interface selection")
                return False

            probe_master = pysoem.Master()
            try:
                probe_master.always_release_gil = True
            except (AttributeError, TypeError):
                pass
            discovered: List[SlaveInfo] = []
            opened = False
            try:
                probe_master.open(self._interfaces[channel_index])
                opened = True
                if probe_master.config_init(release_gil=True) <= 0:
                    self._set_last_error("Scan EtherCAT slaves")
                else:
                    probe_master.read_state()
                    discovered = self._build_slave_info_list(probe_master.slaves)
                    if not discovered:
                        self._set_last_error("Scan EtherCAT slaves")
            except Exception as exc:
                phase = "Open network interface" if not opened else "Scan EtherCAT slaves"
                self._set_last_error(phase, exc)
            finally:
                if opened:
                    try:
                        probe_master.close()
                    except Exception:
                        pass

        if discovered:
            self._clear_last_error()
        slaves.extend(discovered)
        return bool(discovered)

    def init(self, channel_index: int) -> bool:
        """Open the adapter, enumerate slaves, map PDOs, and configure DC."""
        self._clear_last_error()

        with self._lifecycle_lock:
            if self.running or self._initialized:
                self._set_last_error("Already connected")
                return False
            if not self._resolve_interface(channel_index):
                return False

            phase = "Open network interface"
            try:
                with self._soem_lock:
                    self._configure_master_timeouts()
                    self.master.open(self.ifname)
                    self._master_open = True

                    phase = "Scan EtherCAT slaves"
                    if self.master.config_init(release_gil=True) <= 0:
                        return self._fail_initialization("Scan EtherCAT slaves")

                    self.slaves = list(self.master.slaves)
                    for slave in self.slaves:
                        try:
                            slave.is_lost = False
                        except (AttributeError, TypeError):
                            pass

                    known_mapping_bytes = sum(
                        _LSLQ_DH116_OUTPUT_BYTES + _LSLQ_DH116_INPUT_BYTES
                        for slave in self.slaves
                        if self._is_lslq_dh116_slave(slave)
                    )
                    if known_mapping_bytes > _PYSOEM_IOMAP_CAPACITY:
                        return self._fail_initialization(
                            "I/O mapping exceeds PySOEM's 4096-byte IO map"
                        )

                    phase = "I/O mapping"
                    mapped_bytes = int(self.master.config_map())
                    self._slave_output_sizes = [
                        len(slave.output) for slave in self.slaves
                    ]
                    self._slave_input_sizes = [len(slave.input) for slave in self.slaves]
                    self._output_size = sum(self._slave_output_sizes)
                    self._input_size = sum(self._slave_input_sizes)

                    if (
                        mapped_bytes <= 0
                        or mapped_bytes > _PYSOEM_IOMAP_CAPACITY
                        or self._output_size <= 0
                        or self._input_size <= 0
                    ):
                        return self._fail_initialization("I/O mapping")

                    if not self._validate_lslq_dh116_mapping():
                        return self._fail_initialization(
                            f"I/O mapping: {self._dh116_mapping_error}"
                        )

                    self._initialize_output_buffers()
                    self._verify_mapped_esi_configuration()
                    if ETHERCAT_DEBUG:
                        self._verify_esi_configuration()

                    phase = "Configure distributed clock"
                    self.master.config_dc()

                self._slave_info_list = self._build_slave_info_list(self.slaves)
                self._initialized = True
                self._started = False
                self._connection_lost = False
                self._none_state_count = 0
                self._clear_last_error()
                return True
            except Exception as exc:
                return self._fail_initialization(phase, exc)

    def _fail_initialization(
        self, phase: str, error: Optional[BaseException] = None
    ) -> bool:
        self._set_last_error(phase, error)
        if self._master_open:
            try:
                self.master.close()
            except Exception:
                pass
        self._master_open = False
        self._initialized = False
        self._started = False
        self._connection_lost = False
        self._connected_index = -1
        self.slaves = []
        self._slave_info_list = []
        self._slave_output_sizes = []
        self._slave_input_sizes = []
        self._input_size = 0
        self._output_size = 0
        self._clear_output_buffers()
        return False

    def _initialize_output_buffers(self) -> None:
        with self._output_lock:
            self._pending_outputs = []
            self._committed_outputs = []
            self._output_dirty = []
            for index, slave in enumerate(self.slaves):
                output = bytes(self._slave_output_sizes[index])
                slave.output = output
                self._pending_outputs.append(bytearray(output))
                self._committed_outputs.append(output)
                self._output_dirty.append(False)

    def _clear_output_buffers(self) -> None:
        with self._output_lock:
            self._pending_outputs = []
            self._committed_outputs = []
            self._output_dirty = []

    def _validate_lslq_dh116_mapping(self) -> bool:
        self._dh116_mapping_error = ""
        if not self.slaves:
            return True
        slave = self.slaves[0]
        if not self._is_lslq_dh116_slave(slave):
            return True
        sizes_ok = (
            self._slave_output_sizes[0] == _LSLQ_DH116_OUTPUT_BYTES
            and self._slave_input_sizes[0] == _LSLQ_DH116_INPUT_BYTES
        )
        if not sizes_ok:
            self._dh116_mapping_error = (
                "LSLQ DH116 SM2/SM3 mapping requires 64 output and "
                "192 input bytes"
            )
            return False
        if int(self.master.expected_wkc) <= 0:
            self._dh116_mapping_error = (
                "LSLQ DH116 SM2/SM3 mapping produced an invalid expected WKC"
            )
            return False

        sm2 = self._read_sync_manager(slave, _SM2_REGISTER)
        sm3 = self._read_sync_manager(slave, _SM3_REGISTER)
        if sm2 is _SM_READ_FAILED or sm3 is _SM_READ_FAILED:
            self._dh116_mapping_error = (
                "unable to verify LSLQ DH116 SM2/SM3 registers"
            )
            return False
        if sm2 is not None and (sm2[0] != 0x1100 or sm2[1] != 64 or sm2[2] == 0):
            self._dh116_mapping_error = (
                "LSLQ DH116 SM2/SM3 verification: SM2 does not match "
                "start=0x1100, length=64"
            )
            return False
        if sm3 is not None and (sm3[0] != 0x1400 or sm3[1] != 192 or sm3[2] == 0):
            self._dh116_mapping_error = (
                "LSLQ DH116 SM2/SM3 verification: SM3 does not match "
                "start=0x1400, length=192"
            )
            return False
        return True

    @staticmethod
    def _read_sync_manager(slave, register: int):
        """Read an SM register for validation when PySOEM exposes _fprd()."""
        reader = getattr(slave, "_fprd", None)
        if not callable(reader):
            return None
        data = b""
        for attempt in range(3):
            try:
                data = bytes(reader(register, 8, 4_000))
                if len(data) >= 8:
                    break
            except Exception as exc:
                _LOGGER.debug(
                    "Unable to read SM register 0x%04X (attempt %s): %s",
                    register,
                    attempt + 1,
                    exc,
                )
            if attempt < 2:
                time.sleep(_PROCESS_CYCLE_SECONDS)
        if len(data) < 8:
            return _SM_READ_FAILED
        return (
            int.from_bytes(data[0:2], "little"),
            int.from_bytes(data[2:4], "little"),
            int.from_bytes(data[4:8], "little"),
        )

    @staticmethod
    def _is_lslq_dh116_slave(slave) -> bool:
        return (
            int(getattr(slave, "id", 0)) == _LSLQ_DH116_PRODUCT_ID
            and int(getattr(slave, "rev", 0)) == _LSLQ_DH116_REVISION
        )

    def start(self) -> bool:
        """Drive all mapped slaves from SAFE_OP to OPERATIONAL."""
        self._clear_last_error()

        with self._lifecycle_lock:
            if not self._initialized:
                self._set_last_error("SOEM init")
                return False
            if not self.slaves:
                self._set_last_error("Scan EtherCAT slaves")
                return False

            try:
                with self._soem_lock:
                    safe_state = self._wait_for_state(
                        pysoem.SAFEOP_STATE, _SAFE_OP_TIMEOUT_US
                    )
                    self.master.read_state()
                    if (
                        safe_state != pysoem.SAFEOP_STATE
                        or self.master.state != pysoem.SAFEOP_STATE
                    ):
                        self._set_last_error("SAFE_OP")
                        self._log_slave_diagnostics("SAFE_OP")
                        return False

                    started_at = time.perf_counter_ns()
                    self.master.send_processdata()
                    first_wkc = self.master.receive_processdata(
                        _PROCESS_DATA_TIMEOUT_US
                    )
                    self._roundtrip_time_us = int(
                        (time.perf_counter_ns() - started_at) / 1_000
                    )
                    expected_wkc = int(self.master.expected_wkc)
                    if first_wkc < expected_wkc:
                        _LOGGER.warning(
                            "First EtherCAT WKC is %s, expected %s",
                            first_wkc,
                            expected_wkc,
                        )

                    self.master.state = pysoem.OP_STATE
                    self.master.write_state()

                    confirmed_state = int(self.master.state)
                    for attempt in range(_MAX_OP_ATTEMPTS):
                        started_at = time.perf_counter_ns()
                        self.master.send_processdata()
                        self.master.receive_processdata(_PROCESS_DATA_TIMEOUT_US)
                        self._roundtrip_time_us = int(
                            (time.perf_counter_ns() - started_at) / 1_000
                        )

                        if attempt % 10 == 0:
                            confirmed_state = int(
                                self.master.state_check(
                                    pysoem.OP_STATE, _STATE_CHECK_TIMEOUT_US
                                )
                            )
                        if (
                            confirmed_state == pysoem.OP_STATE
                            and self.master.state == pysoem.OP_STATE
                        ):
                            self._started = True
                            self._connection_lost = False
                            self.resetLostFrames()
                            self._clear_last_error()
                            return True
                        time.sleep(_PROCESS_CYCLE_SECONDS)

                    self.master.read_state()
                    self._log_slave_diagnostics("OP")
                    self._set_last_error("OPERATIONAL")
                    return False
            except Exception as exc:
                self._set_last_error("OPERATIONAL", exc)
                return False

    def _wait_for_state(self, expected_state: int, timeout_us: int) -> int:
        """Poll state in short calls so PySOEM 1.1.12 cannot hold the GIL for seconds."""
        deadline = time.perf_counter() + timeout_us / 1_000_000.0
        state = int(getattr(self.master, "state", pysoem.NONE_STATE))
        while True:
            remaining_us = int((deadline - time.perf_counter()) * 1_000_000)
            if remaining_us <= 0:
                return state
            state = int(
                self.master.state_check(
                    expected_state, min(_STATE_CHECK_TIMEOUT_US, remaining_us)
                )
            )
            if state == expected_state:
                return state
            time.sleep(_PROCESS_CYCLE_SECONDS)

    def run(self) -> None:
        """Start the non-blocking PDO exchange thread."""
        with self._lifecycle_lock:
            if not self._started or self.running:
                return

            self._stop_event.clear()
            self.running = True
            self.thread = threading.Thread(
                target=EthercatMaster._process_io_worker,
                args=(weakref.ref(self),),
                name="EthercatMasterProcessData",
                daemon=True,
            )
            try:
                self.thread.start()
            except Exception as exc:
                self.running = False
                self._stop_event.set()
                self.thread = None
                self._set_last_error("Process data", exc)

    @staticmethod
    def _process_io_worker(master_ref) -> None:
        consecutive_lost_frames = 0
        try:
            while True:
                master = master_ref()
                if master is None:
                    return
                if not master.running or master._stop_event.is_set():
                    return

                keep_running, consecutive_lost_frames = master._process_io_cycle(
                    consecutive_lost_frames
                )
                stop_event = master._stop_event
                if not keep_running:
                    return

                # Do not retain the master between cycles. This lets __del__ stop
                # a running instance when user code drops its last reference.
                del master
                if stop_event.wait(_PROCESS_CYCLE_SECONDS):
                    return
        finally:
            master = master_ref()
            if master is not None:
                master.running = False
                master._stop_event.set()

    def _process_io_cycle(self, consecutive_lost_frames: int):
        committed_outputs = self._consume_outputs()

        try:
            with self._soem_lock:
                with self._io_lock:
                    if (
                        not self._started
                        or self._connection_lost
                        or not self.slaves
                        or self._output_size <= 0
                        or self._input_size <= 0
                    ):
                        self._mark_connection_lost("Process data")
                        return False, consecutive_lost_frames

                    for index, output in committed_outputs:
                        self.slaves[index].output = output

                    started_at = time.perf_counter_ns()
                    self.master.send_processdata()
                    wkc = int(
                        self.master.receive_processdata(_PROCESS_DATA_TIMEOUT_US)
                    )
                    self._roundtrip_time_us = int(
                        (time.perf_counter_ns() - started_at) / 1_000
                    )
                    expected_wkc = int(self.master.expected_wkc)
                    dc_time = int(getattr(self.master, "dc_time", 0))
        except Exception as exc:
            self._mark_connection_lost("Process data", exc)
            return False, consecutive_lost_frames

        if expected_wkc <= 0:
            self._mark_connection_lost("Process data")
            return False, consecutive_lost_frames

        if wkc < expected_wkc:
            if consecutive_lost_frames == 0:
                self._read_and_log_slave_states(wkc, expected_wkc)
            with self._stats_lock:
                self._lost_frames += 1
            consecutive_lost_frames += 1

            if consecutive_lost_frames >= _MAX_CONSECUTIVE_LOST_FRAMES:
                self._mark_connection_lost("Process data")
                return False, consecutive_lost_frames
        else:
            consecutive_lost_frames = 0
            _LOGGER.debug(
                "EtherCAT cycle %sus WKC=%s DC=%s",
                self._roundtrip_time_us,
                wkc,
                dc_time,
            )

        return True, consecutive_lost_frames

    def _consume_outputs(self):
        committed = []
        with self._output_lock:
            for index, dirty in enumerate(self._output_dirty):
                if dirty:
                    committed.append((index, self._committed_outputs[index]))
                    self._output_dirty[index] = False
        return committed

    def _commit_output_locked(self, index: int) -> None:
        self._committed_outputs[index] = bytes(self._pending_outputs[index])
        self._output_dirty[index] = True

    def _mark_connection_lost(
        self, phase: str, error: Optional[BaseException] = None
    ) -> None:
        self._set_last_error(phase, error)
        self._connection_lost = True
        self.running = False
        self._stop_event.set()

    def _read_and_log_slave_states(self, wkc: int, expected_wkc: int) -> None:
        try:
            with self._soem_lock:
                self.master.read_state()
                states = " | ".join(
                    self._describe_slave_state(index, slave)
                    for index, slave in enumerate(self.slaves, start=1)
                )
            _LOGGER.warning(
                "EtherCAT WKC abnormal: %s/%s %s", wkc, expected_wkc, states
            )
        except Exception as exc:
            _LOGGER.warning("Unable to read EtherCAT slave states: %s", exc)

    def getState(self) -> EthercatState:
        with self._lifecycle_lock:
            if not self.running or not self._initialized or self._connection_lost:
                return EthercatState.Disconnected

            try:
                with self._soem_lock:
                    self.master.read_state()
                    states = [int(slave.state) for slave in self.slaves]
            except Exception as exc:
                self._set_last_error("Read slave state", exc)
                return EthercatState.Disconnected

        if not states:
            return EthercatState.Disconnected

        def convert(state: int) -> EthercatState:
            if state & pysoem.STATE_ERROR:
                return EthercatState.Error
            if state == pysoem.NONE_STATE:
                return EthercatState.Disconnected
            if state in (pysoem.INIT_STATE, pysoem.PREOP_STATE):
                return EthercatState.Initializing
            if state == pysoem.SAFEOP_STATE:
                return EthercatState.SafeOperational
            if state == pysoem.OP_STATE:
                return EthercatState.Operational
            return EthercatState.Initializing

        priority = {
            EthercatState.Error: 0,
            EthercatState.Disconnected: 1,
            EthercatState.Initializing: 2,
            EthercatState.SafeOperational: 3,
            EthercatState.Operational: 4,
        }
        worst = min((convert(state) for state in states), key=priority.get)

        with self._lifecycle_lock:
            if not self.running or not self._initialized or self._connection_lost:
                return EthercatState.Disconnected
            with self._state_lock:
                if worst == EthercatState.Disconnected:
                    self._none_state_count += 1
                    if self._none_state_count >= _MAX_NONE_STATE_COUNT:
                        self._none_state_count = 0
                        return EthercatState.Disconnected
                    return EthercatState.Initializing
                self._none_state_count = 0
            return worst

    def isConnected(self) -> bool:
        with self._lifecycle_lock:
            return (
                self.running
                and self._initialized
                and self._started
                and not self._connection_lost
            )

    def getSlaveInfoList(self) -> List[SlaveInfo]:
        with self._lifecycle_lock:
            if not self._initialized:
                return []
            try:
                with self._soem_lock:
                    self._slave_info_list = self._build_slave_info_list(self.slaves)
            except Exception as exc:
                self._set_last_error("Read slave state", exc)
            return list(self._slave_info_list)

    def _build_slave_info_list(self, slave_list) -> List[SlaveInfo]:
        result = []
        for index, slave in enumerate(slave_list, start=1):
            al_status = int(getattr(slave, "al_status", 0))
            result.append(
                SlaveInfo(
                    index=index,
                    name=getattr(slave, "name", ""),
                    state=int(getattr(slave, "state", 0)),
                    al_status=al_status,
                    al_status_str=_al_status_to_string(al_status),
                    is_lost=bool(getattr(slave, "is_lost", False)),
                )
            )
        return result

    def _describe_slave_state(self, index: int, slave) -> str:
        state = int(getattr(slave, "state", 0))
        result = f"S{index}={_state_to_string(state)}(0x{state:04x})"
        al_status = int(getattr(slave, "al_status", 0))
        if al_status:
            result += f" AL=0x{al_status:04x} {_al_status_to_string(al_status)}"
        return result

    def _log_slave_diagnostics(self, phase: str) -> None:
        diagnostics = " | ".join(
            self._describe_slave_state(index, slave)
            for index, slave in enumerate(self.slaves, start=1)
        )
        _LOGGER.error("[%s] %s", phase, diagnostics)

    def _print_slave_states(self) -> None:
        for info in self.getSlaveInfoList():
            print(info.toString())

    def getLostFrames(self) -> int:
        with self._stats_lock:
            return self._lost_frames

    def resetLostFrames(self) -> None:
        with self._stats_lock:
            self._lost_frames = 0

    def stop(self) -> None:
        """Stop PDO exchange, request INIT, close the socket, and clear state."""
        with self._lifecycle_lock:
            self.running = False
            self._stop_event.set()
            worker = self.thread

        if (
            worker is not None
            and worker.is_alive()
            and worker is not threading.current_thread()
        ):
            worker.join()

        with self._lifecycle_lock:
            self.thread = None
            with self._soem_lock:
                if self._started:
                    if not self._connection_lost and self.slaves:
                        try:
                            self.master.state = pysoem.INIT_STATE
                            self.master.write_state()
                        except Exception as exc:
                            self._set_last_error("Request INIT", exc)
                    self._started = False

                if self._master_open:
                    try:
                        self.master.close()
                    except Exception as exc:
                        self._set_last_error("Close socket", exc)
                    self._master_open = False

            self._initialized = False
            self.slaves = []
            self._slave_info_list = []
            self._slave_output_sizes = []
            self._slave_input_sizes = []
            self._input_size = 0
            self._output_size = 0
            self.ifname = None
            self._connected_index = -1
            self._none_state_count = 0
            self._connection_lost = False
            self._roundtrip_time_us = 0
            self._clear_output_buffers()
            self.resetLostFrames()

    close = stop

    def getSlaveCount(self) -> int:
        with self._lifecycle_lock:
            return len(self.slaves)

    def getSlaveOutputSize(self, slave_id: int) -> int:
        with self._lifecycle_lock:
            if slave_id < 1 or slave_id > len(self._slave_output_sizes):
                return 0
            return self._slave_output_sizes[slave_id - 1]

    def getSlaveInputSize(self, slave_id: int) -> int:
        with self._lifecycle_lock:
            if slave_id < 1 or slave_id > len(self._slave_input_sizes):
                return 0
            return self._slave_input_sizes[slave_id - 1]

    def _find_slave(self, slave_id: int):
        if slave_id < 1 or slave_id > len(self.slaves):
            return None
        return self.slaves[slave_id - 1]

    def setOutput(self, slave_id: int, index: int, value: int) -> None:
        with self._lifecycle_lock:
            if not self._pdo_available():
                return
            slave = self._find_slave(slave_id)
            output_size = self.getSlaveOutputSize(slave_id)
            if slave is None or index < 0 or index >= output_size:
                return
            if value < 0 or value > 0xFF:
                return
            with self._output_lock:
                self._pending_outputs[slave_id - 1][index] = value
                self._commit_output_locked(slave_id - 1)

    def getInput(self, slave_id: int, index: int) -> int:
        with self._lifecycle_lock:
            if not self._pdo_available():
                return 0
            slave = self._find_slave(slave_id)
            input_size = self.getSlaveInputSize(slave_id)
            if slave is None or index < 0 or index >= input_size:
                return 0
            with self._soem_lock:
                with self._io_lock:
                    data = bytes(slave.input)
            return data[index]

    def setSlaveOutputs(self, slave_id: int, data: bytes, size: int) -> bool:
        with self._lifecycle_lock:
            if not self._pdo_available() or data is None or size <= 0:
                return False
            slave = self._find_slave(slave_id)
            output_size = self.getSlaveOutputSize(slave_id)
            if slave is None or output_size == 0:
                return False
            try:
                payload = bytes(data)
            except (TypeError, ValueError):
                return False
            if size > len(payload) or size > output_size:
                return False

            with self._output_lock:
                self._pending_outputs[slave_id - 1][:size] = payload[:size]
                self._commit_output_locked(slave_id - 1)
            return True

    def getSlaveInputs(self, slave_id: int, size: int) -> Optional[bytes]:
        with self._lifecycle_lock:
            if not self._pdo_available() or size <= 0:
                return None
            slave = self._find_slave(slave_id)
            if slave is None or size > self.getSlaveInputSize(slave_id):
                return None
            with self._soem_lock:
                with self._io_lock:
                    return bytes(slave.input[:size])

    def getSlaveOutputs(self, slave_id: int, size: int) -> Optional[bytes]:
        with self._lifecycle_lock:
            if not self._pdo_available() or size <= 0:
                return None
            slave = self._find_slave(slave_id)
            if slave is None or size > self.getSlaveOutputSize(slave_id):
                return None
            with self._output_lock:
                return bytes(self._pending_outputs[slave_id - 1][:size])

    def _pdo_available(self) -> bool:
        return self._started and not self._connection_lost

    def sdoRead(
        self, index: int, subindex: int, slave_id: int = 1
    ) -> Optional[int]:
        with self._lifecycle_lock:
            if not self._initialized or not self._valid_sdo_address(index, subindex):
                return None
            slave = self._find_slave(slave_id)
            if slave is None:
                return None

            try:
                with self._soem_lock:
                    data = bytes(slave.sdo_read(index, subindex, size=4))
                if len(data) < 4:
                    self._set_last_error(
                        f"SDO data too short (expected 4, got {len(data)})"
                    )
                    return None
                return int.from_bytes(data[:4], byteorder="little", signed=False)
            except Exception as exc:
                self._set_last_error(
                    f"SDO Read [Slave {slave_id} 0x{index:04X}:{subindex:02X}]",
                    exc,
                )
                return None

    def sdoWrite(
        self, index: int, subindex: int, value: int, slave_id: int = 1
    ) -> bool:
        with self._lifecycle_lock:
            if (
                not self._initialized
                or not self._valid_sdo_address(index, subindex)
                or value < 0
                or value > 0xFFFFFFFF
            ):
                return False
            slave = self._find_slave(slave_id)
            if slave is None:
                return False

            try:
                data = int(value).to_bytes(4, byteorder="little", signed=False)
                with self._soem_lock:
                    slave.sdo_write(index, subindex, data)
                return True
            except Exception as exc:
                self._set_last_error(
                    f"SDO Write [Slave {slave_id} 0x{index:04X}:{subindex:02X}]",
                    exc,
                )
                return False

    @staticmethod
    def _valid_sdo_address(index: int, subindex: int) -> bool:
        return 0 <= index <= 0xFFFF and 0 <= subindex <= 0xFF

    def _read_sdo_value(
        self, slave_id: int, index: int, subindex: int, max_size: int
    ) -> Optional[int]:
        slave = self._find_slave(slave_id)
        if slave is None or max_size <= 0 or max_size > 4:
            return None
        try:
            data = bytes(slave.sdo_read(index, subindex, size=max_size))
        except Exception as exc:
            _LOGGER.debug(
                "ESI verify: CoE read 0x%04X:%02X unavailable: %s",
                index,
                subindex,
                exc,
            )
            return None
        if not data:
            return None
        return int.from_bytes(data[:max_size], byteorder="little", signed=False)

    def _verify_sdo_value(
        self,
        name: str,
        index: int,
        subindex: int,
        expected: int,
        max_size: int,
    ) -> int:
        actual = self._read_sdo_value(1, index, subindex, max_size)
        if actual is None:
            return 0
        if actual != expected:
            _LOGGER.debug(
                "ESI mismatch: %s 0x%04X:%02X actual=0x%X expected=0x%X",
                name,
                index,
                subindex,
                actual,
                expected,
            )
            return 1
        return 0

    def _verify_pdo_mapping(
        self,
        name: str,
        pdo_index: int,
        object_index: int,
        expected_entries: int,
    ) -> int:
        actual_count = self._read_sdo_value(1, pdo_index, 0, 1)
        if actual_count is None:
            return 0
        mismatches = int(actual_count != expected_entries)
        for subindex in range(1, min(actual_count, expected_entries) + 1):
            actual = self._read_sdo_value(1, pdo_index, subindex, 4)
            expected = (object_index << 16) | (subindex << 8) | 0x10
            if actual != expected:
                mismatches += 1
                _LOGGER.debug(
                    "ESI mismatch: %s 0x%04X:%02X actual=%r expected=0x%08X",
                    name,
                    pdo_index,
                    subindex,
                    actual,
                    expected,
                )
        if actual_count > expected_entries:
            mismatches += actual_count - expected_entries
        return mismatches

    def _verify_esi_configuration(self) -> None:
        if not self.slaves or not self._is_lslq_dh116_slave(self.slaves[0]):
            return
        mismatches = 0
        mismatches += self._verify_sdo_value("SM2 RxPDO count", 0x1C12, 0, 1, 1)
        mismatches += self._verify_sdo_value(
            "SM2 RxPDO assignment", 0x1C12, 1, 0x1600, 2
        )
        mismatches += self._verify_sdo_value("SM3 TxPDO count", 0x1C13, 0, 1, 1)
        mismatches += self._verify_sdo_value(
            "SM3 TxPDO assignment", 0x1C13, 1, 0x1A00, 2
        )
        mismatches += self._verify_pdo_mapping("RxPDO", 0x1600, 0x9000, 32)
        mismatches += self._verify_pdo_mapping("TxPDO", 0x1A00, 0x9001, 96)
        _LOGGER.debug("ESI verify complete: mismatches=%s", mismatches)

    def _verify_mapped_esi_configuration(self) -> None:
        if not self.slaves or not self._is_lslq_dh116_slave(self.slaves[0]):
            return
        output_bytes = self._slave_output_sizes[0]
        input_bytes = self._slave_input_sizes[0]
        mismatches = int(output_bytes != _LSLQ_DH116_OUTPUT_BYTES) + int(
            input_bytes != _LSLQ_DH116_INPUT_BYTES
        )
        _LOGGER.debug(
            "ESI mapped verify: actual=%sO+%sI expected=%sO+%sI mismatches=%s",
            output_bytes,
            input_bytes,
            _LSLQ_DH116_OUTPUT_BYTES,
            _LSLQ_DH116_INPUT_BYTES,
            mismatches,
        )
