"""3-D hand-pose overlay for the teleop camera view. Visualisation only.

The control path is untouched: this module receives the HandReading the
retargeter already consumed and only draws on the camera frame, after the
command for that frame has been sent. It returns nothing the loop acts on,
keeps its own canonicalizer and tracker (no state is shared with control),
and ``draw`` never raises -- a failing frame is skipped and counted.

How the 3-D axes land on the image
----------------------------------
MediaPipe's 2-D image landmarks and 3-D world landmarks come from the same
detection, so a per-frame least-squares affine map A (2x4) from the 21 world
points to the 21 image points (42 equations, 8 unknowns) is a weak-perspective
camera, good to a few pixels over a hand. Axis *directions* are projected
through the linear part of A; axis *origins* are the tracked 2-D landmarks
themselves, so the triads stay pinned to the hand even though A is lightly
smoothed. The canonicalizer's mirror is absorbed by A (a reflection is
affine), so the frames are built in canonical right-hand coordinates exactly
as everywhere else.

Axes are skipped for a frame when the fit residual is poor, when the palm
frame is not confident, or -- per finger -- when that root frame is not.
Axis lengths are a fixed fraction of the (smoothed) metric palm width, so they
scale with the hand on screen and foreshorten with pose, but never jump.
"""

from __future__ import annotations

import time
from typing import Optional, Tuple

import cv2
import numpy as np

from . import handpose as hp
from .frameviz import AXIS_BGR
from .viz import CONNECTIONS

ALL_FINGERS = ("thumb", "index", "middle", "ring", "pinky")
ROOT_JOINTS = {"thumb": "thumb_cmc", "index": "index_mcp", "middle": "middle_mcp",
               "ring": "ring_mcp", "pinky": "pinky_mcp"}
TIPS = (4, 8, 12, 16, 20)

OUTLINE = (22, 22, 22)
BONE = (222, 222, 222)
DOT = (255, 255, 255)
LABELS = ("x", "y", "z")


class PoseOverlay:
    def __init__(self, assume_mirrored: bool = True, *,
                 palm_axis: float = 0.34, finger_axis: float = 0.17,
                 palm_thickness: int = 6, finger_thickness: int = 4,
                 min_palm_conf: float = 0.5, min_finger_conf: float = 0.3,
                 max_resid_px: float = 15.0, fit_alpha: float = 0.6) -> None:
        self.tracker = hp.HandPoseTracker(ALL_FINGERS, history=300)
        self.canon = hp.Canonicalizer(assume_mirrored=assume_mirrored)
        self.palm_axis, self.finger_axis = palm_axis, finger_axis     # fraction of palm width
        self.palm_thickness, self.finger_thickness = palm_thickness, finger_thickness
        self.min_palm_conf, self.min_finger_conf = min_palm_conf, min_finger_conf
        self.max_resid_px = max_resid_px
        self.fit_alpha = fit_alpha
        self.enabled = True
        # Statistics the teleop prints at exit.
        self.frames = 0          # frames offered
        self.drawn = 0           # frames on which axes were drawn
        self.skipped = 0         # skeleton only: poor fit / low confidence
        self.failures = 0        # exceptions swallowed
        self.consecutive_failures = 0
        self.total_ms = 0.0
        self.last_ms = 0.0
        self.last_resid_px = float("nan")
        self._A: Optional[np.ndarray] = None
        self._A_chirality: Optional[int] = None
        self._width: Optional[float] = None
        self._depth: Optional[np.ndarray] = None

    def toggle(self) -> bool:
        self.enabled = not self.enabled
        return self.enabled

    # ------------------------------------------------------------ public

    def draw(self, frame: np.ndarray, reading, pose: Optional[hp.HandPose3D] = None) -> bool:
        """Draw onto ``frame`` in place. Never raises; returns True if axes were drawn."""
        if not self.enabled:
            return False
        t0 = time.perf_counter()
        self.frames += 1
        try:
            ok = self._draw(frame, reading, pose)
            self.consecutive_failures = 0
            if ok:
                self.drawn += 1
            else:
                self.skipped += 1
            return ok
        except Exception as exc:                          # visualisation must never reach control
            self.failures += 1
            self.consecutive_failures += 1
            if self.failures <= 3 or self.consecutive_failures in (30, 300):
                print(f"pose overlay: frame skipped ({type(exc).__name__}: {exc})")
            return False
        finally:
            self.last_ms = (time.perf_counter() - t0) * 1000.0
            self.total_ms += self.last_ms

    # ----------------------------------------------------------- drawing

    def _draw(self, frame, reading, pose) -> bool:
        if reading is None or reading.landmarks_2d is None or reading.world_pts is None:
            return False
        pts2 = np.asarray(reading.landmarks_2d, dtype=np.float64)
        self._skeleton(frame, pts2, np.asarray(reading.world_pts)[:, 2])

        canon_pts = self.canon(reading.world_pts, reading.handedness)
        if pose is None or set(pose.fingers) != set(ALL_FINGERS):
            pose = self.tracker.update(canon_pts)

        A, resid = self._fit(canon_pts, pts2)
        self.last_resid_px = resid
        if resid > self.max_resid_px:
            return False
        palm = pose.joints["palm"]
        if palm.confidence < self.min_palm_conf or pose.used_continuity:
            return False

        w = float(np.linalg.norm(pose.points[5] - pose.points[17]))
        self._width = w if self._width is None else 0.9 * self._width + 0.1 * w
        width = self._width

        # Finger roots first, palm last so the large triad is on top.
        for finger in ALL_FINGERS:
            j = pose.joints[ROOT_JOINTS[finger]]
            if j.confidence < self.min_finger_conf or j.degenerate:
                continue
            self._triad(frame, A, pts2[j.landmark], pose.wrist_rotation @ j.rotation,
                        self.finger_axis * width, self.finger_thickness, label=False)
        self._triad(frame, A, pts2[0], pose.wrist_rotation,       # palm: R_local is I
                    self.palm_axis * width, self.palm_thickness, label=True)
        return True

    def _fit(self, canon_pts: np.ndarray, pts2: np.ndarray) -> Tuple[np.ndarray, float]:
        X = np.hstack([canon_pts, np.ones((21, 1))])
        A, *_ = np.linalg.lstsq(X, pts2, rcond=None)                 # (4, 2)
        resid = float(np.sqrt(np.mean(np.sum((X @ A - pts2) ** 2, axis=1))))
        # Smooth the camera, not the landmarks; a chirality switch changes the
        # canonical coordinates, so the smoothed map must restart there.
        if self._A is None or self._A_chirality != self.canon.chirality:
            self._A = A
        else:
            self._A = self.fit_alpha * A + (1.0 - self.fit_alpha) * self._A
        self._A_chirality = self.canon.chirality
        return self._A, resid

    def _triad(self, frame, A, origin2, R_world, length_m, thickness, label):
        o = (int(round(origin2[0])), int(round(origin2[1])))
        ends = origin2[None, :] + (R_world.T * length_m) @ A[:3, :]     # rows: x, y, z tips
        order = np.argsort(-np.linalg.norm(ends - origin2, axis=1))      # longest first, shortest on top
        for i in order:
            e = (int(round(ends[i, 0])), int(round(ends[i, 1])))
            if np.hypot(e[0] - o[0], e[1] - o[1]) < 5:                  # pointing at the camera
                cv2.circle(frame, o, thickness + 2, OUTLINE, -1, cv2.LINE_AA)
                cv2.circle(frame, o, thickness, AXIS_BGR[i], -1, cv2.LINE_AA)
                continue
            cv2.arrowedLine(frame, o, e, OUTLINE, thickness + 3, cv2.LINE_AA, tipLength=0.28)
            cv2.arrowedLine(frame, o, e, AXIS_BGR[i], thickness, cv2.LINE_AA, tipLength=0.28)
            if label:
                d = np.array(e) - np.array(o)
                d = d / (np.linalg.norm(d) + 1e-9) * 12
                p = (int(e[0] + d[0]) - 5, int(e[1] + d[1]) + 5)
                cv2.putText(frame, LABELS[i], p, cv2.FONT_HERSHEY_SIMPLEX, 0.55, OUTLINE, 3, cv2.LINE_AA)
                cv2.putText(frame, LABELS[i], p, cv2.FONT_HERSHEY_SIMPLEX, 0.55, AXIS_BGR[i], 1, cv2.LINE_AA)
        cv2.circle(frame, o, thickness + 1, OUTLINE, -1, cv2.LINE_AA)
        cv2.circle(frame, o, thickness - 1, DOT, -1, cv2.LINE_AA)

    def _skeleton(self, frame, pts2: np.ndarray, z: np.ndarray) -> None:
        """Clean skeleton: outlined bones, outlined dots, a subtle smoothed depth cue."""
        zn = (z - z.mean()) / (z.std() + 1e-9)
        zn = np.clip(zn, -1.5, 1.5)
        self._depth = zn if self._depth is None else 0.7 * self._depth + 0.3 * zn
        p = [(int(round(u)), int(round(v))) for u, v in pts2]
        for a, b in CONNECTIONS:
            cv2.line(frame, p[a], p[b], OUTLINE, 5, cv2.LINE_AA)
        for a, b in CONNECTIONS:
            cv2.line(frame, p[a], p[b], BONE, 2, cv2.LINE_AA)
        for i, (u, v) in enumerate(p):
            base = 5.0 if i in TIPS else 3.5
            r = int(round(base * (1.0 - 0.18 * self._depth[i])))       # MediaPipe z < 0 is toward the camera
            cv2.circle(frame, (u, v), r + 2, OUTLINE, -1, cv2.LINE_AA)
            cv2.circle(frame, (u, v), r, DOT, -1, cv2.LINE_AA)

    # --------------------------------------------------------------- stats

    def summary(self) -> str:
        if self.frames == 0:
            return "3D pose overlay: never drawn"
        mean = self.total_ms / self.frames
        return (f"3D pose overlay: axes on {self.drawn}/{self.frames} frames "
                f"({self.skipped} skeleton-only, {self.failures} failures caught), "
                f"{mean:.2f} ms/frame, last fit residual {self.last_resid_px:.1f} px")
