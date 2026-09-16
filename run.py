#!/usr/bin/env python3
"""Entry point: puts the project-local vendor/ dir on sys.path, then runs the CLI."""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

# Quiet the third-party console noise before anything imports cv2 or mediapipe.
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "3")
os.environ.setdefault("GLOG_minloglevel", "2")


def _ensure_qt_fonts() -> None:
    """Give the vendored OpenCV's Qt build the fonts it expects.

    cv2/config-3.py *assigns* QT_QPA_FONTDIR to a bundled directory that the
    wheel ships empty, so setting that variable ourselves has no effect -- the
    import overwrites it. Populating the directory it insists on is what
    actually silences the warning, and it self-heals if vendor/ is reinstalled.
    """
    fontdir = ROOT / "vendor" / "cv2" / "qt" / "fonts"
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


_ensure_qt_fonts()
# pin (Pinocchio) wheels use the cmeel layout: the importable package sits one
# level below vendor/, so that directory has to be on the path as well.
_CMEEL = ROOT / "vendor" / "cmeel.prefix" / "lib" / f"python{sys.version_info.major}.{sys.version_info.minor}" / "site-packages"
for extra in (_CMEEL, ROOT / "vendor", ROOT):
    p = str(extra)
    if p not in sys.path:
        sys.path.insert(0, p)

from telehand.teleop import main

if __name__ == "__main__":
    sys.exit(main())
