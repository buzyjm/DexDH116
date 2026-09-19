"""Every filesystem location the project uses, in one place.

Nothing here touches the disk at import time. The runtime state directory is
created on demand by :func:`ensure_state_dir`; calibration and data files are
created by the code that writes them.
"""

from __future__ import annotations

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

# Third-party material shipped with the project rather than installed.
VENDOR_DIR = PROJECT_ROOT / "vendor"                       # Python dependencies
THIRD_PARTY_DIR = PROJECT_ROOT / "third_party"             # vendor SDK and robot models
SDK_ROOT = THIRD_PARTY_DIR / "LHandProLib-API-Linux-20260727" / "x86_64"
URDF_ROOT = THIRD_PARTY_DIR / "DH116_URDF_MJCF_Files"
MODELS_DIR = PROJECT_ROOT / "models"
HAND_LANDMARKER_MODEL = MODELS_DIR / "hand_landmarker.task"

# Calibration written by the CLI and read back on later runs.
CALIBRATION_DIR = PROJECT_ROOT / "calibration"
POSES_FILE = CALIBRATION_DIR / "poses.json"
PROFILE_FILE = CALIBRATION_DIR / "handprofile.json"
DIRECT_CALIBRATION_FILE = CALIBRATION_DIR / "direct_calibration.json"

# Measurements and recordings.
DATA_DIR = PROJECT_ROOT / "data"
RECORDINGS_DIR = DATA_DIR / "recordings"
TOUCHSCAN_FILE = DATA_DIR / "touchscan.json"

# Per-machine runtime state. Nothing in here is worth keeping.
STATE_DIR = PROJECT_ROOT / ".telehand"
CAMERA_FILE = STATE_DIR / "camera.json"
VIEW_FILE = STATE_DIR / "view.json"
CACHE_DIR = STATE_DIR / "cache"
LOCK_FILE = STATE_DIR / "session.lock"


def ensure_state_dir() -> Path:
    """Create the runtime state directory if needed and return it."""
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    return STATE_DIR
