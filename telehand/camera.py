"""Camera discovery and low-latency V4L2 capture.

Device numbers shuffle when cameras are plugged in a different order, and two
identical models share a serial, so a bare index is not a reliable name for a
camera. ``by-path`` links are keyed on the USB port and stay unique; the
``cameras --use`` command remembers one in the state directory.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

from .paths import CAMERA_FILE, ensure_state_dir

CAMERA_BY_PATH = Path("/dev/v4l/by-path")


def saved_camera():
    """The camera chosen with `run.py cameras --use`, if it is still present."""
    try:
        spec = json.loads(CAMERA_FILE.read_text())["camera"]
    except Exception:
        return None
    if str(spec).startswith("/dev/") and not Path(spec).exists():
        print(f"saved camera {spec} is not connected; falling back")
        return None
    return spec


def save_camera(spec) -> None:
    """Remember ``spec`` as the default camera for later runs."""
    ensure_state_dir()
    CAMERA_FILE.write_text(json.dumps({"camera": str(spec)}, indent=2) + "\n")


def list_cameras():
    """Every capture device, with the path that survives a reboot or replug.

    Device numbers shuffle when cameras are plugged in a different order, and
    two identical models share a serial, so `by-id` cannot tell them apart.
    `by-path` is keyed on the USB port and stays unique.
    """
    out = []
    for node in sorted(Path("/dev").glob("video*"), key=lambda p: int(p.name[5:])):
        sysdir = Path(f"/sys/class/video4linux/{node.name}")
        # Each camera exposes several nodes; index 0 is the one that captures.
        # Reading /sys instead of probing with OpenCV keeps this quiet and
        # instant -- opening a metadata node logs a warning and wastes a second.
        try:
            if (sysdir / "index").read_text().strip() != "0":
                continue
        except OSError:
            continue
        stable = None
        if CAMERA_BY_PATH.is_dir():
            for link in sorted(CAMERA_BY_PATH.iterdir()):
                if link.resolve() == node.resolve() and "index0" in link.name:
                    stable = str(link)
                    break
        name = ""
        info = sysdir / "name"
        if info.exists():
            name = info.read_text().strip()
        out.append({"node": str(node), "index": int(node.name[5:]),
                    "name": name, "by_path": stable})
    return out


def resolve_camera(spec):
    """Accept an index, a /dev node, a by-path link, or a name fragment."""
    if spec in (None, "", "auto"):
        spec = saved_camera()
        if spec is None:
            cams = list_cameras()
            if not cams:
                raise RuntimeError("no capture devices found")
            spec = cams[0]["node"]
    if isinstance(spec, int):
        return spec
    text = str(spec)
    if text.isdigit():
        return int(text)
    if text.startswith("/dev/"):
        return text
    matches = [c for c in list_cameras() if text.lower() in c["name"].lower()]
    if len(matches) == 1:
        return matches[0]["node"]
    if len(matches) > 1:
        raise RuntimeError(f"{text!r} matches {len(matches)} cameras; use a by-path from "
                           "`run.py cameras`")
    raise RuntimeError(f"no camera matches {text!r}; see `run.py cameras`")


class Camera:
    """V4L2 capture that always hands back the newest frame.

    Two things matter for teleop latency and neither is the default:

    * ``CAP_PROP_BUFFERSIZE = 1`` looks like the way to avoid stale frames, but
      with the V4L2 backend it halves the delivered rate (measured 15 fps on
      both cameras here, against 30 without it). So we leave the queue alone.
    * With a normal queue, a slow consumer falls behind and you end up
      teleoperating from frames that are several hundred ms old. A reader
      thread that keeps only the most recent frame fixes that instead.
    """

    def __init__(self, index, width: int = 640, height: int = 480, fps: int = 30,
                 settle: float = 1.5):
        import cv2

        self._cv2 = cv2
        target = resolve_camera(index)
        self.target = target
        cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera {index!r} (resolved to {target!r})")
        # FOURCC must be set before the frame size to take effect.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)

        self._cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0
        self._last_seq = 0
        self._running = True
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
        # Auto-exposure needs a moment: the first frames off a freshly opened
        # camera come back blown out or black, which reads as a broken device.
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            with self._lock:
                if self._seq > 8:
                    break
            time.sleep(0.02)
        self._last_seq = self._seq

    def _pump(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame = frame
                self._seq += 1

    def read(self, timeout: float = 1.0):
        """Block until a frame newer than the last one returned is available."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._frame is not None and self._seq != self._last_seq:
                    self._last_seq = self._seq
                    return True, self._frame.copy()
            time.sleep(0.002)
        return False, None

    def release(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()
