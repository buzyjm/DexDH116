"""Pose library: the bridge between a human hand shape and a robot hand shape.

Each anchor pairs one set of tracker signals with the joint angles that put the
robot into the matching pose. Between anchors the retargeter blends, so adding
an anchor is how you teach a pose the mapping could not otherwise express --
thumb-to-middle opposition, for instance, which is neither an open hand nor a
fist nor a blend of the two.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POSE_FILE = PROJECT_ROOT / "poses.json"

# The poses worth capturing, in the order calibration walks through them.
# Each opposition pose is what teaches the mapping to close the thumb onto one
# specific finger while the others stay out of the way.
POSE_PLAN = [
    ("open", "hold your hand FLAT and OPEN, fingers straight"),
    ("pinch_index", "thumb tip on INDEX tip, other fingers extended"),
    ("pinch_middle", "thumb tip on MIDDLE tip, other fingers extended"),
    ("pinch_ring", "thumb tip on RING tip, other fingers extended"),
    ("pinch_pinky", "thumb tip on PINKY tip, other fingers extended"),
    ("fist", "tight FIST, thumb across the fingers"),
]

# Robot-side joint angles for each pose.
#
# `open`, `fist` and `pinch_index` are known: the first two are the joint limits
# and the third was measured on the hardware with `run.py jog`. The other three
# are ESTIMATES -- the thumb has to swing further across the palm to reach each
# successive finger, and the target finger has to come up to meet it. Measure
# them with `run.py jog` and save over these; the estimates only exist so the
# system runs before you have.
DEFAULT_JOINTS: Dict[str, List[float]] = {
    "open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "pinch_index": [10.0, 15.0, 45.0, 0.0, 0.0, 0.0],
    "pinch_middle": [25.0, 18.0, 0.0, 50.0, 0.0, 0.0],
    "pinch_ring": [40.0, 20.0, 0.0, 0.0, 55.0, 0.0],
    "pinch_pinky": [60.0, 20.0, 0.0, 0.0, 0.0, 50.0],
    "fist": [60.0, 30.0, 80.0, 80.0, 80.0, 80.0],
}
MEASURED = {"open", "fist", "pinch_index", "pinch_middle", "pinch_ring", "pinch_pinky"}


@dataclass
class PoseAnchor:
    name: str
    joints: List[float]
    signals: Optional[List[float]] = None   # filled in by calibration
    measured: bool = False                  # joints came from the hardware

    @property
    def complete(self) -> bool:
        return self.signals is not None


@dataclass
class PoseLibrary:
    anchors: List[PoseAnchor] = field(default_factory=list)

    @classmethod
    def default(cls) -> "PoseLibrary":
        return cls([
            PoseAnchor(name, list(DEFAULT_JOINTS[name]), None, name in MEASURED)
            for name, _ in POSE_PLAN
        ])

    def get(self, name: str) -> Optional[PoseAnchor]:
        for a in self.anchors:
            if a.name == name:
                return a
        return None

    def set_joints(self, name: str, joints: List[float], measured: bool = True) -> None:
        anchor = self.get(name)
        if anchor is None:
            self.anchors.append(PoseAnchor(name, list(joints), None, measured))
        else:
            anchor.joints = list(joints)
            anchor.measured = measured

    @property
    def complete_anchors(self) -> List[PoseAnchor]:
        return [a for a in self.anchors if a.complete]

    def save(self, path: Path = DEFAULT_POSE_FILE) -> None:
        Path(path).write_text(json.dumps({
            "version": 2,
            "anchors": [
                {"name": a.name, "joints": a.joints,
                 "signals": a.signals, "measured": a.measured}
                for a in self.anchors
            ],
        }, indent=2) + "\n")

    @classmethod
    def load(cls, path: Path = DEFAULT_POSE_FILE) -> "PoseLibrary":
        path = Path(path)
        if not path.exists():
            return cls.default()
        data = json.loads(path.read_text())
        lib = cls([
            PoseAnchor(
                a["name"],
                [float(v) for v in a["joints"]],
                [float(v) for v in a["signals"]] if a.get("signals") else None,
                bool(a.get("measured", False)),
            )
            for a in data.get("anchors", [])
        ])
        # Poses added to the plan after the file was written show up as
        # unmeasured, uncalibrated anchors so jog and calibrate can fill them in.
        known = {a.name for a in lib.anchors}
        for name, _ in POSE_PLAN:
            if name not in known:
                lib.anchors.append(PoseAnchor(name, list(DEFAULT_JOINTS[name]), None,
                                              name in MEASURED))
        return lib


class PoseRetargeter:
    """Blend the pose library by how close the current hand is to each anchor.

    Inverse-distance weighting in normalized signal space. It reproduces every
    anchor exactly, which is the property that matters here: a pose you took the
    trouble to record is a pose the robot will actually reach.
    """

    def __init__(self, limits, library: PoseLibrary, *, smoothing: float = 0.35,
                 deadband_deg: float = 0.8, power: float = 2.5) -> None:
        self.limits = limits
        self.library = library
        self.smoothing = smoothing
        self.deadband_deg = deadband_deg
        self.power = power

        anchors = library.complete_anchors
        if len(anchors) < 2:
            raise ValueError("pose library needs at least two calibrated anchors")
        self._sig = np.array([a.signals for a in anchors], dtype=np.float64)
        self._joints = np.array([a.joints for a in anchors], dtype=np.float64)
        self.names = [a.name for a in anchors]

        # Normalize each signal by its spread across the anchors, so a channel
        # with a 140-degree range does not drown out one with a 20-degree range.
        self._lo = self._sig.min(axis=0)
        span = self._sig.max(axis=0) - self._lo
        self._span = np.where(span < 1e-6, 1.0, span)
        self._norm = (self._sig - self._lo) / self._span

        self._filtered: Optional[np.ndarray] = None
        self._held: Optional[np.ndarray] = None

    def weights(self, signals) -> np.ndarray:
        point = (np.asarray(signals, dtype=np.float64) - self._lo) / self._span
        dist = np.linalg.norm(self._norm - point, axis=1)
        exact = dist < 1e-6
        if exact.any():
            w = exact.astype(np.float64)
            return w / w.sum()
        w = 1.0 / np.power(dist, self.power)
        return w / w.sum()

    def __call__(self, signals) -> List[float]:
        angles = self.weights(signals) @ self._joints

        if self._filtered is None:
            self._filtered = angles.copy()
        else:
            a = self.smoothing
            self._filtered = a * angles + (1.0 - a) * self._filtered

        if self._held is None:
            self._held = self._filtered.copy()
        else:
            move = np.abs(self._filtered - self._held) > self.deadband_deg
            self._held = np.where(move, self._filtered, self._held)

        return [lim.clamp(float(v)) for lim, v in zip(self.limits, self._held)]

    def reset(self) -> None:
        self._filtered = None
        self._held = None
