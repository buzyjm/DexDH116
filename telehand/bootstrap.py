"""Make the project-local dependencies importable.

This machine has no pip, venv or usable sudo, so every third-party package
lives under ``vendor/`` (installed with pip's zipapp; see the README). Call
:func:`bootstrap` before importing anything from that directory. ``run.py``
does it for the CLI; standalone scripts import this module and call it.

Only the standard library may be imported here, for the obvious reason.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from .paths import PROJECT_ROOT, VENDOR_DIR

# pin (Pinocchio) wheels use the cmeel layout: the importable package sits one
# level below vendor/, so that directory has to be on the path as well.
CMEEL_SITE_PACKAGES = (
    VENDOR_DIR / "cmeel.prefix" / "lib"
    / f"python{sys.version_info.major}.{sys.version_info.minor}" / "site-packages"
)


def _quiet_native_logging() -> None:
    """Silence third-party console noise before cv2 or mediapipe are imported."""
    os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
    os.environ.setdefault("GLOG_minloglevel", "2")


def _ensure_qt_fonts() -> None:
    """Give the vendored OpenCV's Qt build the fonts it expects.

    cv2/config-3.py *assigns* QT_QPA_FONTDIR to a bundled directory that the
    wheel ships empty, so setting that variable ourselves has no effect: the
    import overwrites it. Populating the directory it insists on is what
    actually silences the warning, and it self-heals if vendor/ is reinstalled.
    """
    fontdir = VENDOR_DIR / "cv2" / "qt" / "fonts"
    if fontdir.is_dir() and any(fontdir.iterdir()):
        return
    system = Path("/usr/share/fonts/truetype/dejavu")
    if not system.is_dir():
        return
    try:
        fontdir.mkdir(parents=True, exist_ok=True)
        for ttf in system.glob("*.ttf"):
            link = fontdir / ttf.name
            if not link.exists():
                link.symlink_to(ttf)
    except OSError:
        pass  # cosmetic only; a warning on stderr is not worth failing over


def bootstrap() -> None:
    """Quiet the native libraries and put ``vendor/`` on ``sys.path``."""
    _quiet_native_logging()
    _ensure_qt_fonts()
    for extra in (CMEEL_SITE_PACKAGES, VENDOR_DIR, PROJECT_ROOT):
        p = str(extra)
        if p not in sys.path:
            sys.path.insert(0, p)
