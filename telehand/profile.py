"""Per-operator hand profile for the kinematic retargeter.

Two operators with the same intent produce different fingertip geometry, and
a single palm-width scale cannot absorb finger-length proportions. A profile
records each finger's length from a flat open hand and how much air MediaPipe
leaves when the operator actually pinches, so the solver can scale each finger
on its own and the pinch closer knows what "touching" looks like for this hand.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = PROJECT_ROOT / "handprofile.json"

MCP = {"index": 5, "middle": 9, "ring": 13, "pinky": 17}
TIP = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
THUMB_CMC = 1


@dataclass
class HandProfile:
    palm_width: float                                  # metres
    finger_len: Dict[str, float]                       # metres, thumb from CMC, fingers from MCP
    pinch_gap: Optional[float] = None                  # metres between tips when actually pinching
    name: str = ""

    def save(self, path: Path = DEFAULT_PROFILE) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")

    @classmethod
    def load(cls, path: Path = DEFAULT_PROFILE) -> Optional["HandProfile"]:
        p = Path(path)
        if not p.exists():
            return None
        d = json.loads(p.read_text())
        return cls(float(d["palm_width"]), {k: float(v) for k, v in d["finger_len"].items()},
                   None if d.get("pinch_gap") is None else float(d["pinch_gap"]), d.get("name", ""))

    def describe(self) -> str:
        rows = [f"palm width  {self.palm_width * 1000:5.1f} mm"]
        for f, v in self.finger_len.items():
            rows.append(f"{f:<7}     {v * 1000:5.1f} mm")
        if self.pinch_gap is not None:
            rows.append(f"pinch gap   {self.pinch_gap * 1000:5.1f} mm  (tracker's air in a real pinch)")
        return "\n".join(rows)


def measure_open(frames: Sequence[np.ndarray]) -> Dict[str, float]:
    """Lengths from frames of a flat, open hand (median over frames)."""
    pts = np.asarray(frames, dtype=np.float64)
    out = {"palm_width": float(np.median(np.linalg.norm(pts[:, MCP["index"]] - pts[:, MCP["pinky"]], axis=1)))}
    out["thumb"] = float(np.median(np.linalg.norm(pts[:, TIP["thumb"]] - pts[:, THUMB_CMC], axis=1)))
    for f in ("index", "middle", "ring", "pinky"):
        out[f] = float(np.median(np.linalg.norm(pts[:, TIP[f]] - pts[:, MCP[f]], axis=1)))
    return out


def measure_pinch_gap(frames: Sequence[np.ndarray]) -> float:
    pts = np.asarray(frames, dtype=np.float64)
    return float(np.median(np.linalg.norm(pts[:, TIP["thumb"]] - pts[:, TIP["index"]], axis=1)))


def build(open_frames, pinch_frames=None, name: str = "") -> HandProfile:
    m = measure_open(open_frames)
    gap = measure_pinch_gap(pinch_frames) if pinch_frames is not None and len(pinch_frames) else None
    return HandProfile(m["palm_width"], {f: m[f] for f in ("thumb", "index", "middle", "ring", "pinky")}, gap, name)
