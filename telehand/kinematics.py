"""Forward kinematics for the DH116 hand, read from the vendor URDF.

The URDF describes 11 revolute joints, matching the 11 the firmware reports.
Only 6 are driven; the other 5 (each finger's distal joint, plus the thumb's)
are moved by linkages. The URDF does not say how -- it carries no `mimic`
tags -- so the coupling is a parameter here rather than a fact read from file.
"""

from __future__ import annotations

import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Sequence, Tuple

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
# 2026-04-09 package: same geometry as the 2025-11-24 one, but joint limits
# now equal the firmware ranges (60/30/80) and the passive joints carry their
# coupling. Vendor MJCF states it as a quadratic in the driving angle (rad).
URDF_ROOT = PROJECT_ROOT / "DH116_URDF_MJCF_Files"

# distal = c0 + c1*q + c2*q^2, q = driving joint angle in radians.
COUPLING = {
    "thumb":  (0.004592, 0.8808, 0.469825),
    "finger": (0.011072, 0.9206, 0.200535),
}
FIRMWARE_MAX_DEG = [60.0, 30.0, 80.0, 80.0, 80.0, 80.0]

# Firmware angle -> driving joint angle, per joint (degrees in, degrees out).
#
# The vendor's sim takes the firmware angle as the joint angle. On the real
# hand the firmware angle is a linear function of actuator stroke and the joint
# angle is not. The four fingers were measured from side-view photos at
# commands 0/20/40/60/80 -> 0/19.4/35.4/50.1/62.6 deg (proximal phalanx vs the
# 0-deg frame); a quadratic fits those within 1 deg, and its 62.6 deg full
# travel matches the 0..60 deg sweep of the vendor's own coupling study.
# The thumb curves could not be read from the photos and are parametrised
# instead: same shape as the fingers, scaled to a full travel found by
# tactile contact (see touch calibration); abduction as a plain gain.
def _finger_curve(fw: float) -> float:
    return max(0.0, 0.987 * fw - 0.00256 * fw * fw)


# Fitted to 10 sensor-confirmed thumb contacts and 16 confirmed non-contacts
# (touch_grid.json). Loosely determined: F 50..60 and G 0.6..0.7 fit almost
# as well, so expect a few degrees of thumb error away from the pinch region.
THUMB_FLEX_FULL_DEG = 57.5     # joint angle at firmware 30
THUMB_ABD_GAIN = 0.6           # joint deg per firmware deg
THUMB_ABD_OFFSET = 5.0         # joint deg at firmware 0

# Mechanical envelope, also measured: the further the thumb is swung across the
# palm, the less it can flex before it binds. Firmware degrees in and out.
_FLEX_ENVELOPE = [(0.0, 30.0), (30.0, 30.0), (40.0, 27.0), (50.0, 21.0), (60.0, 18.0)]


def thumb_flex_max(abd_fw: float) -> float:
    xs = [p[0] for p in _FLEX_ENVELOPE]; ys = [p[1] for p in _FLEX_ENVELOPE]
    return float(np.interp(abd_fw, xs, ys))


def _thumb_flex_curve(fw: float) -> float:
    return THUMB_FLEX_FULL_DEG * _finger_curve(80.0 * fw / 30.0) / 62.6


def _thumb_abd_curve(fw: float) -> float:
    return max(0.0, THUMB_ABD_GAIN * fw + THUMB_ABD_OFFSET)


DRIVE_MAP = {
    0: _thumb_abd_curve,
    1: _thumb_flex_curve,
    2: _finger_curve, 3: _finger_curve, 4: _finger_curve, 5: _finger_curve,
}

# Actuated joint -> the URDF chain it drives, tip mesh, and the passive joint.
FINGER_CHAINS = {
    "thumb": (("finger11", "finger12", "finger13"), "finger14"),
    "index": (("finger21", "finger22"), "finger23"),
    "middle": (("finger31", "finger32"), "finger33"),
    "ring": (("finger41", "finger42"), "finger43"),
    "pinky": (("finger51", "finger52"), "finger53"),
}
FINGER_ORDER = ["index", "middle", "ring", "pinky"]


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def _stl_vertices(path: Path) -> np.ndarray:
    raw = path.read_bytes()
    if raw[:5] == b"solid" and b"facet" in raw[:512]:
        return np.array([[float(v) for v in line.split()[1:4]]
                         for line in raw.decode("utf8", "ignore").splitlines()
                         if line.strip().startswith("vertex")])
    count = struct.unpack("<I", raw[80:84])[0]
    vals = []
    for i in range(count):
        off = 84 + i * 50 + 12
        vals += list(struct.unpack("<9f", raw[off:off + 36]))
    return np.array(vals).reshape(-1, 3)


@dataclass
class Joint:
    xyz: np.ndarray
    rotation: np.ndarray
    axis: np.ndarray | None
    lower: float
    upper: float


class HandModel:
    """Kinematic model of one DH116 hand."""

    def __init__(self, direction: str = "R") -> None:
        pkg = URDF_ROOT / f"DH116-{direction}000-A1"
        urdf = pkg / "urdf" / f"DH116-{direction}000-A1.urdf"
        if not urdf.exists():
            raise FileNotFoundError(f"URDF not found: {urdf}")
        self.direction = direction
        self.joints: Dict[str, Joint] = {}
        root = ET.parse(urdf).getroot()
        for j in root.findall("joint"):
            origin = j.find("origin")
            axis_el = j.find("axis")
            limit = j.find("limit")
            self.joints[j.get("name")] = Joint(
                xyz=np.array([float(v) for v in origin.get("xyz").split()]),
                rotation=_rpy_to_matrix(*[float(v) for v in origin.get("rpy").split()]),
                axis=(np.array([float(v) for v in axis_el.get("xyz").split()])
                      if axis_el is not None else None),
                lower=float(limit.get("lower")) if limit is not None else 0.0,
                upper=float(limit.get("upper")) if limit is not None else 0.0,
            )

        # The tip links are fixed at their parent's origin, so the contact point
        # lives in the mesh: take the vertex furthest from that origin.
        suffix = "link"
        self.tips: Dict[str, np.ndarray] = {}
        for finger, (_chain, tip_link) in FINGER_CHAINS.items():
            pts = _stl_vertices(pkg / "meshes" / f"{tip_link}_{suffix}.STL")
            self.tips[finger] = pts[np.argmax(np.linalg.norm(pts, axis=1))]

    def fingertip(self, finger: str, angles: Sequence[float]) -> np.ndarray:
        """Tip position in base_link, metres. `angles` are URDF joint radians."""
        chain, _ = FINGER_CHAINS[finger]
        pos = np.zeros(3)
        rot = np.eye(3)
        for name, q in zip(chain, angles):
            j = self.joints[name]
            pos = pos + rot @ j.xyz
            rot = rot @ j.rotation @ _axis_rotation(j.axis, q)
        return pos + rot @ self.tips[finger]

    def limits(self, finger: str) -> list[Tuple[float, float]]:
        chain, _ = FINGER_CHAINS[finger]
        return [(self.joints[n].lower, self.joints[n].upper) for n in chain]

    def all_fingertips(self, urdf_angles: Dict[str, Sequence[float]]) -> Dict[str, np.ndarray]:
        return {f: self.fingertip(f, q) for f, q in urdf_angles.items()}

    # ------------------------------------------------------------ firmware

    @staticmethod
    def coupled(kind: str, q: float) -> float:
        c0, c1, c2 = COUPLING[kind]
        return c0 + c1 * q + c2 * q * q

    @classmethod
    def joints_from_firmware(cls, fw: Sequence[float]) -> Dict[str, list]:
        """Firmware angles (deg, JOINT_NAMES order) -> per-finger URDF chain radians.

        The vendor's model treats each firmware angle as the driving joint's
        angle directly, and derives the passive joint from it.
        """
        def drv(i):
            return np.radians(DRIVE_MAP[i](float(fw[i])))
        q11, q12 = drv(0), drv(1)
        out = {"thumb": [q11, q12, cls.coupled("thumb", q12)]}
        for i, f in enumerate(FINGER_ORDER):
            q1 = drv(2 + i)
            out[f] = [q1, cls.coupled("finger", q1)]
        return out

    def fingertips_from_firmware(self, fw: Sequence[float]) -> Dict[str, np.ndarray]:
        return self.all_fingertips(self.joints_from_firmware(fw))

    def chain_points(self, finger: str, angles: Sequence[float]) -> list:
        """Joint origins along the chain plus the tip, in base_link (metres)."""
        chain, _ = FINGER_CHAINS[finger]
        pos = np.zeros(3); rot = np.eye(3); pts = []
        for name, q in zip(chain, angles):
            j = self.joints[name]
            pos = pos + rot @ j.xyz
            rot = rot @ j.rotation @ _axis_rotation(j.axis, q)
            pts.append(pos.copy())
        pts.append(pos + rot @ self.tips[finger])
        return pts
