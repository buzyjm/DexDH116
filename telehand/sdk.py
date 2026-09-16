"""Locate and load the vendor LHandProLib SDK.

The shipped Python example expects to be run from inside its own directory and
its loader cannot find the .so in this layout, so we put the example package on
sys.path and hand the loader an explicit library path.
"""

from __future__ import annotations

import functools
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SDK_ROOT = PROJECT_ROOT / "LHandProLib-API-Linux-20260727" / "x86_64"
EXAMPLE_DIR = SDK_ROOT / "share" / "LHandProLib" / "examples" / "RS485_python"
LIBRARY = SDK_ROOT / "lib" / "libLHandProLib.so"

_loaded = False


def load():
    """Import the vendor SDK, returning (RS485Controller, wrapper_module)."""
    global _loaded

    if not LIBRARY.exists():
        raise FileNotFoundError(f"LHandProLib shared library not found: {LIBRARY}")
    if not EXAMPLE_DIR.exists():
        raise FileNotFoundError(f"RS485 example package not found: {EXAMPLE_DIR}")

    # controller_rs485 does a bare `from serial_port import SerialPort`, so the
    # example directory itself has to be importable, not just its sub-package.
    path = str(EXAMPLE_DIR)
    if path not in sys.path:
        sys.path.insert(0, path)

    from lhandprolib_python_sdk import controller_base
    from lhandprolib_python_sdk import lhandprolib_wrapper
    from lhandprolib_python_sdk.controller_rs485 import RS485Controller

    if not _loaded:
        controller_base.PyLHandProLib = functools.partial(
            lhandprolib_wrapper.PyLHandProLib, lib_path=str(LIBRARY)
        )
        _loaded = True

    return RS485Controller, lhandprolib_wrapper


# --------------------------------------------------------------- flush bypass

_flush_patched = False
_original_write = None


def disable_serial_flush() -> bool:
    """Stop the vendor serial layer calling ``flush()`` after every write.

    ``SerialPort.write`` (vendor, examples/RS485_python/serial_port.py) does::

        with self._write_lock:
            written = self.serial.write(data)
            self.serial.flush()

    ``flush()`` is ``tcdrain``: it blocks until the UART has physically
    transmitted. One ``write_angles`` issues seven of these -- six
    ``set_target_angle`` plus ``move_motors`` -- and measured 12 ms per frame,
    about 1.7 ms each, which is USB round-trip granularity rather than the
    500 kbaud line rate.

    Removing the drain does not reorder anything. ``_write_lock`` still
    serializes callers, so frames are handed to the kernel in call order, and a
    tty output buffer is FIFO, so they reach the wire in that same order --
    ``move_motors`` still arrives after the six target writes. What is given up
    is only the *caller's* knowledge of when the bytes left, which nothing in
    this path uses: the send callback checks ``written > 0``, which counts bytes
    accepted by the kernel and is unaffected.

    If the host ever outran the line, ``serial.write`` would block on a full
    kernel buffer rather than lose data, so the failure mode is the latency we
    started with, not corruption.

    This patches the class object at runtime. Nothing under the vendor tree is
    modified -- and it must not be, since that directory is a symlink to the
    original project.

    Returns True if the patch was applied.
    """
    global _flush_patched, _original_write
    if _flush_patched:
        return True

    import serial_port    # importable because load() put EXAMPLE_DIR on sys.path

    _original_write = serial_port.SerialPort.write

    def write_no_drain(self, data: bytes) -> int:
        """Vendor write() minus the tcdrain. Same locking, same return value."""
        if not self.is_open or not self.serial:
            return 0
        try:
            with self._write_lock:
                return int(self.serial.write(data))
        except Exception as exc:
            print(f"RS485 send failed: {exc}")
            return 0

    serial_port.SerialPort.write = write_no_drain
    _flush_patched = True
    return True


def restore_serial_flush() -> None:
    """Undo :func:`disable_serial_flush`."""
    global _flush_patched, _original_write
    if not _flush_patched:
        return
    import serial_port
    serial_port.SerialPort.write = _original_write
    _flush_patched = False
