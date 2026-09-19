"""Camera-driven teleoperation for the LHandPro DH116 dexterous hand."""

from __future__ import annotations

import importlib

__version__ = "0.1.0"

# Public names, resolved on first use so that importing the package does not
# pull in numpy or the vendor SDK before ``telehand.bootstrap`` has run.
_EXPORTS = {
    "Hand": "telehand.hand",
    "JOINT_NAMES": "telehand.hand",
    "HandTracker": "telehand.tracker",
    "SIGNAL_NAMES": "telehand.tracker",
    "PoseLibrary": "telehand.poses",
    "PoseAnchor": "telehand.poses",
    "PoseRetargeter": "telehand.poses",
}

__all__ = ["__version__", *_EXPORTS]


def __getattr__(name: str):
    module = _EXPORTS.get(name)
    if module is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    return getattr(importlib.import_module(module), name)
