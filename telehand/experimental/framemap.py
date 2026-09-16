"""Frame-based joint mapping for teleop (experimental, ``--pose-mode frames``).

Three DH116 joints are driven from :class:`handpose.HandPose3D`; the other
three stay with the FK solver, which also supplies the fallback whenever the
frame evidence is not trustworthy:

    thumb_abduction  <-  thumb_swing (default) or thumb_lift
    thumb_flexion    <-  thumb_curl
    index_flexion    <-  index_curl
    middle / ring / pinky  <-  FK solver, unchanged

Signals (all wrist-local, canonical right hand, degrees):

* ``index_curl``  = geodesic bend at index MCP + PIP + DIP. The scalar
  ``index_flexion`` is PIP + DIP only; the MCP term is what is new (measured
  +22 deg median on fk_session.npz, r = +0.98 with the scalar).
* ``thumb_curl``  = thumb MCP + IP bends (identical to the scalar
  ``thumb_flexion`` by construction, but taken from the frames).
* ``thumb_swing`` = angle of the thumb metacarpal (CMC frame z axis) in the
  palm plane, from +y (pointing to the thumb side) toward +z (distal).
  Measured r = +0.96 against the scalar ``thumb_abduction``.
* ``thumb_lift``  = elevation of the metacarpal out of the palm plane,
  palmar positive. Measured r = +0.65. Kept as the alternative candidate.

These are explicit geometric quantities; none of them is claimed to *be*
flexion or abduction. Which one maps best onto the DH116 thumb is exactly
what running both modes is meant to find out.

Each mapped joint is a calibrated linear map (open value -> joint minimum,
closed value -> joint maximum), then the same EMA + deadband the FK
retargeter applies, so the two modes differ only in the signal. Max-speed,
stall kick and firmware limits are applied downstream in ``Hand.write_angles``
for both.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from .. import handpose as hp

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIB_FILE = PROJECT_ROOT / "frames_calibration.json"

FRAME_SIGNALS = ["index_curl", "thumb_curl", "thumb_swing", "thumb_lift"]
# Open -> closed, from fk_session.npz percentiles p2..p98. Press 1 (open) and
# 2 (fist) in teleop to replace them with your own.
DEFAULT_RANGES: Dict[str, List[float]] = {
    "index_curl": [40.0, 220.0],
    "thumb_curl": [16.0, 71.0],
    "thumb_swing": [40.0, 78.0],
    "thumb_lift": [0.0, 30.0],
}
ABD_SIGNALS = {"swing": "thumb_swing", "lift": "thumb_lift"}
MAPPED_JOINTS = (0, 1, 2)            # thumb_abduction, thumb_flexion, index_flexion
INDEX_CHAIN = ("index_mcp", "index_pip", "index_dip")
THUMB_CHAIN = ("thumb_cmc", "thumb_mcp", "thumb_ip")
CAPTURE_FRAMES = 20


def frame_signals(pose: hp.HandPose3D) -> Dict[str, float]:
    z = pose.joints["thumb_cmc"].rotation[:, 2]
    return {
        "index_curl": (pose.bend_angle_deg("index_mcp") + pose.bend_angle_deg("index_pip")
                       + pose.bend_angle_deg("index_dip")),
        "thumb_curl": pose.chain_bend_deg("thumb"),
        "thumb_swing": float(np.degrees(np.arctan2(z[2], z[1]))),
        "thumb_lift": float(np.degrees(np.arcsin(np.clip(z[0], -1.0, 1.0)))),
    }


# ------------------------------------------------------------- calibration

@dataclass
class FrameCalibration:
    ranges: Dict[str, List[float]] = field(
        default_factory=lambda: {k: list(v) for k, v in DEFAULT_RANGES.items()})
    calibrated: bool = False

    def save(self, path: Path = DEFAULT_CALIB_FILE) -> None:
        Path(path).write_text(json.dumps({"ranges": self.ranges, "calibrated": self.calibrated},
                                         indent=2) + "\n")

    @classmethod
    def load(cls, path: Path = DEFAULT_CALIB_FILE) -> "FrameCalibration":
        p = Path(path)
        if not p.exists():
            return cls()
        d = json.loads(p.read_text())
        ranges = {k: list(v) for k, v in DEFAULT_RANGES.items()}
        ranges.update({k: [float(a), float(b)] for k, (a, b) in d.get("ranges", {}).items()})
        return cls(ranges, bool(d.get("calibrated", True)))


class FrameRangeCapture:
    """Average a few frames of frame signals to set one end of every range."""

    def __init__(self, frames: int = CAPTURE_FRAMES) -> None:
        self.frames = frames
        self._which: Optional[int] = None      # 0 open, 1 closed
        self._buf: List[Dict[str, float]] = []

    def start(self, which: int) -> None:
        self._which = which
        self._buf = []

    @property
    def active(self) -> bool:
        return self._which is not None

    def feed(self, signals: Dict[str, float], calib: FrameCalibration) -> Optional[str]:
        if self._which is None:
            return None
        self._buf.append(dict(signals))
        if len(self._buf) < self.frames:
            return None
        for k in FRAME_SIGNALS:
            calib.ranges[k][self._which] = float(np.mean([b[k] for b in self._buf]))
        calib.calibrated = True
        label = "open" if self._which == 0 else "closed"
        self._which = None
        return label


# ------------------------------------------------------------------ mapper

@dataclass
class FrameMapResult:
    angles: List[float]          # full 6-vector frames mode sends (3..5 are the FK values)
    frame_cmd: List[float]       # raw frame-derived targets for joints 0, 1, 2 (before smoothing)
    fk_cmd: List[float]          # what the FK solver produced for joints 0, 1, 2
    applied: List[float]         # joints 0, 1, 2 after source choice, EMA, deadband, clamp
    source: List[str]            # "F" frame-derived or "K" FK fallback, per mapped joint
    signals: Dict[str, float]
    conf_palm: float
    conf_index: float
    conf_thumb: float
    disp_mm: float               # largest wrist-local landmark move since the previous frame
    plausible: bool
    reason: str                  # why anything fell back; empty when nothing did


class FrameMapper:
    def __init__(self, limits, calib: FrameCalibration, *, smoothing: float = 0.35,
                 deadband_deg: float = 0.8, abd_signal: str = "swing",
                 min_conf: float = 0.05, max_disp_mm: float = 30.0) -> None:
        if abd_signal not in ABD_SIGNALS:
            raise ValueError(f"abd_signal must be one of {list(ABD_SIGNALS)}")
        self.limits = limits
        self.calib = calib
        self.smoothing = smoothing
        self.deadband = deadband_deg
        self.abd_key = ABD_SIGNALS[abd_signal]
        self.min_conf = min_conf
        self.max_disp_mm = max_disp_mm
        self._filtered: Optional[np.ndarray] = None
        self._held: Optional[np.ndarray] = None
        self._prev_points: Optional[np.ndarray] = None
        self.fallbacks = 0
        self.frames = 0

    def reset(self) -> None:
        self._filtered = None
        self._held = None
        self._prev_points = None

    def _map(self, joint: int, key: str, value: float) -> float:
        lo, hi = self.calib.ranges[key]
        span = hi - lo
        frac = 0.0 if abs(span) < 1e-6 else (value - lo) / span
        frac = float(np.clip(frac, 0.0, 1.0))
        lim = self.limits[joint]
        return lim.min_angle + frac * (lim.max_angle - lim.min_angle)

    def _smooth(self, chosen: Sequence[float]) -> List[float]:
        """FKRetargeter's EMA then deadband hold, on the three mapped joints."""
        q = np.asarray(chosen, dtype=np.float64)
        a = self.smoothing
        self._filtered = q.copy() if self._filtered is None else a * q + (1.0 - a) * self._filtered
        if self._held is None:
            self._held = self._filtered.copy()
        else:
            move = np.abs(self._filtered - self._held) > self.deadband
            self._held = np.where(move, self._filtered, self._held)
        return [self.limits[j].clamp(float(v)) for j, v in zip(MAPPED_JOINTS, self._held)]

    def map(self, pose: hp.HandPose3D, fk_out: Sequence[float],
            canon: Optional[hp.Canonicalizer] = None) -> FrameMapResult:
        self.frames += 1
        sig = frame_signals(pose)
        disp = 0.0
        if self._prev_points is not None:
            disp = float(np.linalg.norm(pose.points - self._prev_points, axis=1).max() * 1000.0)
        self._prev_points = pose.points

        conf_palm = pose.joints["palm"].confidence
        conf_i = min(pose.joints[n].confidence for n in INDEX_CHAIN)
        conf_t = min(pose.joints[n].confidence for n in THUMB_CHAIN)
        deg_i = any(pose.joints[n].degenerate for n in INDEX_CHAIN)
        deg_t = any(pose.joints[n].degenerate for n in THUMB_CHAIN)

        reasons: List[str] = []
        plausible = True
        if disp > self.max_disp_mm:
            plausible = False
            reasons.append(f"landmark jump {disp:.0f}mm")
        if pose.used_continuity:
            plausible = False
            reasons.append("palm sign by continuity")
        if conf_palm < 0.5:
            plausible = False
            reasons.append(f"palm conf {conf_palm:.2f}")
        use_index = plausible and conf_i >= self.min_conf and not deg_i
        use_thumb = plausible and conf_t >= self.min_conf and not deg_t
        if plausible and not use_index:
            reasons.append("index degenerate" if deg_i else f"index conf {conf_i:.2f}")
        if plausible and not use_thumb:
            reasons.append("thumb degenerate" if deg_t else f"thumb conf {conf_t:.2f}")
        if canon is not None and canon.last_source == "default":
            reasons.append("chirality assumed")      # logged, not a fallback

        frame_cmd = [self._map(0, self.abd_key, sig[self.abd_key]),
                     self._map(1, "thumb_curl", sig["thumb_curl"]),
                     self._map(2, "index_curl", sig["index_curl"])]
        fk_cmd = [float(fk_out[j]) for j in MAPPED_JOINTS]
        chosen = [frame_cmd[0] if use_thumb else fk_cmd[0],
                  frame_cmd[1] if use_thumb else fk_cmd[1],
                  frame_cmd[2] if use_index else fk_cmd[2]]
        source = ["F" if use_thumb else "K", "F" if use_thumb else "K", "F" if use_index else "K"]
        if "K" in source:
            self.fallbacks += 1
        applied = self._smooth(chosen)

        angles = [float(v) for v in fk_out]
        for k, j in enumerate(MAPPED_JOINTS):
            angles[j] = applied[k]
        return FrameMapResult(angles, frame_cmd, fk_cmd, applied, source, sig,
                              conf_palm, conf_i, conf_t, disp, plausible, "; ".join(reasons))


# --------------------------------------------------------------- logging

LOG_COLUMNS = (["t", "pose_mode", "engaged", "canon_source", "canon_vote", "chirality",
                "disp_mm", "plausible", "conf_palm", "conf_index", "conf_thumb"]
               + ["s_thumb_abd", "s_thumb_flex", "s_index_flex", "s_middle_flex", "s_ring_flex", "s_pinky_flex"]
               + [f"f_{k}" for k in FRAME_SIGNALS]
               + ["fk_abd", "fk_flex", "fk_index", "fr_abd", "fr_flex", "fr_index",
                  "src_abd", "src_flex", "src_index",
                  "cmd_abd", "cmd_flex", "cmd_index", "cmd_middle", "cmd_ring", "cmd_pinky", "reason"])


def log_header() -> str:
    return ",".join(LOG_COLUMNS) + "\n"


def log_row(t: float, pose_mode: str, engaged: bool, scalar_signals: Sequence[float],
            fm: FrameMapResult, canon: Optional[hp.Canonicalizer],
            commanded: Optional[Sequence[float]]) -> str:
    cmd = list(commanded) if commanded is not None else [float("nan")] * 6
    vals = ([f"{t:.3f}", pose_mode, int(engaged),
             canon.last_source if canon else "", f"{canon.last_vote:.0f}" if canon else "",
             (canon.chirality if canon and canon.chirality is not None else 0),
             f"{fm.disp_mm:.1f}", int(fm.plausible), f"{fm.conf_palm:.2f}", f"{fm.conf_index:.2f}",
             f"{fm.conf_thumb:.2f}"]
            + [f"{v:.2f}" for v in scalar_signals]
            + [f"{fm.signals[k]:.2f}" for k in FRAME_SIGNALS]
            + [f"{v:.2f}" for v in fm.fk_cmd] + [f"{v:.2f}" for v in fm.frame_cmd]
            + fm.source
            + [f"{v:.2f}" for v in cmd]
            + [fm.reason.replace(",", ";")])
    return ",".join(str(v) for v in vals) + "\n"
