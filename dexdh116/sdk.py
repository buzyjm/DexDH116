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
