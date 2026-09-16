"""Vendor-style direct retargeting: per-joint bend angles, linearly mapped.

This replicates the pipeline in Leadtron's own `hand_detector` (recovered from
the shipped PyInstaller bundle): smooth the 21 landmarks first (EMA with a
deadband, then a per-landmark Kalman filter), compute each finger's bend angles
from *2D image-plane* triplets, and drive every actuator proportionally. It
knows nothing about fingertip geometry, which is exactly why it feels 1:1 and
responsive -- and why it cannot promise a pinch closes. Kept side by side with
the pose library so both can be compared on the same hand.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIRECT_FILE = PROJECT_ROOT / "direct_calibration.json"

# Landmark triplets (a, b, c): bend measured at b. Same table as the vendor's.
FINGER_JOINTS: Dict[str, List[tuple]] = {
    "thumb": [(0, 1, 2), (1, 2, 3), (2, 3, 4)],
    "index": [(0, 5, 6), (5, 6, 7), (6, 7, 8)],
    "middle": [(0, 9, 10), (9, 10, 11), (10, 11, 12)],
    "ring": [(0, 13, 14), (13, 14, 15), (14, 15, 16)],
    "pinky": [(0, 17, 18), (17, 18, 19), (18, 19, 20)],
}

SIGNAL_NAMES = ["thumb_abduction", "thumb_flexion", "index_flexion",
                "middle_flexion", "ring_flexion", "pinky_flexion"]

# Raw bend degrees for a typical hand until calibrated: (open, closed).
# The thumb CMC bend reads ~40 deg on a spread, relaxed hand, so its range
# starts there; anything lower parks the thumb half-swung with the hand open.
# These are placeholders -- press 1 / 2 in teleop to capture your own.
DEFAULT_RANGES = [(40.0, 75.0), (5.0, 60.0), (5.0, 90.0),
                  (5.0, 90.0), (5.0, 90.0), (5.0, 90.0)]


def bend_2d(a, b, c) -> float:
    """180 minus the angle at b, from image-plane coordinates. Straight = 0."""
    r = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    ang = abs(np.degrees(r))
    ang = min(ang, 360.0 - ang)
    return 180.0 - ang


def finger_bends(pts: np.ndarray) -> Dict[str, List[float]]:
    """All 15 bends from 21 (x, y) points, three per finger, proximal first."""
    return {f: [bend_2d(pts[i], pts[j], pts[k]) for i, j, k in tri]
            for f, tri in FINGER_JOINTS.items()}


def bends_to_signals(bends: Dict[str, List[float]]) -> List[float]:
    """Six scalars in JOINT_NAMES order.

    The thumb's CMC bend tracks how far it is swung across the palm; its MCP
    and IP bends together are the curl. Each finger uses the mean of its MCP
    and PIP bends, which is steadier than either alone.
    """
    t = bends["thumb"]
    out = [t[0], 0.5 * (t[1] + t[2])]
    for f in ("index", "middle", "ring", "pinky"):
        b = bends[f]
        out.append(0.5 * (b[0] + b[1]))
    return out


class LandmarkSmoother:
    """EMA with a deadband, then a constant-velocity Kalman filter per axis.

    Both stages run on normalized landmark coordinates, before any angle is
    computed. The measurement-noise value is not recoverable from the vendor
    bytecode; 1.0 is a conventional choice and is exposed for tuning.
    """

    def __init__(self, alpha: float = 0.3, deadband: float = 0.02,
                 process_noise: float = 0.1, measurement_noise: float = 1.0,
                 use_kalman: bool = True) -> None:
        self.alpha = alpha
        self.deadband = deadband
        self.use_kalman = use_kalman
        self._prev: Optional[np.ndarray] = None
        # State per landmark per axis: [position, velocity].
        self._x = np.zeros((21, 2, 2))
        self._P = np.tile(np.eye(2), (21, 2, 1, 1))
        self._F = np.array([[1.0, 1.0], [0.0, 1.0]])
        self._Q = np.eye(2) * process_noise
        self._R = float(measurement_noise)
        self._init = False

    def reset(self) -> None:
        self._prev = None
        self._init = False

    def __call__(self, pts: np.ndarray) -> np.ndarray:
        pts = np.asarray(pts, dtype=np.float64)
        if self._prev is None:
            self._prev = pts.copy()
            sm = pts
        else:
            delta = np.abs(pts - self._prev)
            moved = delta > self.deadband
            sm = np.where(moved, self.alpha * pts + (1 - self.alpha) * self._prev, self._prev)
            self._prev = sm
        if not self.use_kalman:
            return sm
        if not self._init:
            self._x[:, :, 0] = sm
            self._x[:, :, 1] = 0.0
            self._init = True
            return sm
        out = np.empty_like(sm)
        for i in range(21):
            for a in range(2):
                x = self._F @ self._x[i, a]
                P = self._F @ self._P[i, a] @ self._F.T + self._Q
                z = sm[i, a]
                S = P[0, 0] + self._R
                K = P[:, 0] / S
                x = x + K * (z - x[0])
                P = P - np.outer(K, P[0, :])
                self._x[i, a] = x
                self._P[i, a] = P
                out[i, a] = x[0]
        return out


@dataclass
class DirectCalibration:
    ranges: List[List[float]] = field(default_factory=lambda: [list(r) for r in DEFAULT_RANGES])
    calibrated: bool = False

    def save(self, path: Path = DEFAULT_DIRECT_FILE) -> None:
        Path(path).write_text(json.dumps({"ranges": self.ranges, "calibrated": self.calibrated},
                                         indent=2) + "\n")

    @classmethod
    def load(cls, path: Path = DEFAULT_DIRECT_FILE) -> "DirectCalibration":
        p = Path(path)
        if not p.exists():
            return cls()
        d = json.loads(p.read_text())
        return cls([[float(a), float(b)] for a, b in d["ranges"]], bool(d.get("calibrated", True)))


class DirectRetargeter:
    """Bend signals -> joint angles by per-joint linear interpolation."""

    def __init__(self, limits, calibration: Optional[DirectCalibration] = None) -> None:
        self.limits = limits
        self.calibration = calibration or DirectCalibration()

    def __call__(self, signals: Sequence[float]) -> List[float]:
        out = []
        for i, s in enumerate(signals):
            lo, hi = self.calibration.ranges[i]
            span = hi - lo
            frac = 0.0 if abs(span) < 1e-6 else (s - lo) / span
            frac = max(0.0, min(1.0, frac))
            lim = self.limits[i]
            out.append(lim.min_angle + frac * (lim.max_angle - lim.min_angle))
        return out

    def reset(self) -> None:
        pass


class RangeCapture:
    """Average a few frames of signals to set one end of every range."""

    def __init__(self, frames: int = 20) -> None:
        self.frames = frames
        self._which: Optional[int] = None
        self._buf: List[List[float]] = []

    def start(self, which: int) -> None:      # 0 = open end, 1 = closed end
        self._which = which
        self._buf = []

    @property
    def active(self) -> bool:
        return self._which is not None

    def feed(self, signals: Sequence[float], calib: DirectCalibration) -> Optional[str]:
        if self._which is None:
            return None
        self._buf.append(list(signals))
        if len(self._buf) < self.frames:
            return None
        mean = np.mean(np.array(self._buf), axis=0)
        for i in range(len(mean)):
            calib.ranges[i][self._which] = float(mean[i])
        calib.calibrated = True
        label = "open" if self._which == 0 else "closed"
        self._which = None
        return label
