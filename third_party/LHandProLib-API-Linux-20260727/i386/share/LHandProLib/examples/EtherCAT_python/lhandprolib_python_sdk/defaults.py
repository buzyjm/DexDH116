"""Built-in defaults for the Python example SDK."""

import os

DEFAULT_LIBRARY_BASE_NAME = os.environ.get("SDK_LIBRARY_NAME", "LHandProLib")
DEFAULT_C_API_PREFIX = os.environ.get("SDK_C_API_PREFIX", "lhandprolib")

try:
    from .build_config import C_API_PREFIX, LIBRARY_BASE_NAME

    DEFAULT_LIBRARY_BASE_NAME = os.environ.get(
        "SDK_LIBRARY_NAME", LIBRARY_BASE_NAME
    )
    DEFAULT_C_API_PREFIX = os.environ.get("SDK_C_API_PREFIX", C_API_PREFIX)
except ImportError:
    pass

DEFAULT_HAND_TYPE = 0

DEFAULT_CANFD_NODE_ID = 1
DEFAULT_RS485_NODE_ID = 1
DEFAULT_RS485_BAUD_RATE = 500000

DEFAULT_ENABLE_HOME_CHECK = True
