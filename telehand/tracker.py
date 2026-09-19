"""MediaPipe hand tracking reduced to the six signals the hand actually needs.

Angles are computed from `hand_world_landmarks` (metric, hand-relative) rather
than image coordinates, so the readings stay stable as you move toward or away
from the camera or turn your wrist.
"""

from __future__ import annotations

import contextlib
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np

from .paths import HAND_LANDMARKER_MODEL

# MediaPipe's 21-point hand topology, as (mcp, pip, dip, tip) chains.
FINGER_CHAINS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}
WRIST = 0
THUMB_MCP = 2
THUMB_TIP = 4
INDEX_MCP = 5
PINKY_MCP = 17

# Order matches telehand.hand.JOINT_NAMES.
SIGNAL_NAMES = [
    "thumb_abduction",
    "thumb_flexion",
    "index_flexion",
    "middle_flexion",
    "ring_flexion",
    "pinky_flexion",
]


@contextlib.contextmanager
def _quiet_native_stderr():
    """Silence the C++ log lines MediaPipe writes straight to fd 2 on startup.

    They are emitted by the graph as it builds (XNNPACK delegate, feedback
    manager, landmark projection) and say nothing actionable, but they bypass
    Python logging entirely, so the fd has to be redirected. Scoped as tightly
    as possible so real errors later still reach the terminal.
    """
    saved = os.dup(2)
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(saved, 2)
        os.close(devnull)
        os.close(saved)


@dataclass
class HandReading:
    """One frame of tracking, as six raw signals in degrees."""

    signals: List[float]
    handedness: str
    landmarks_2d: Optional[np.ndarray] = None  # (21, 2) in image coords, for drawing
    world_pts: Optional[np.ndarray] = None     # (21, 3) metric, hand-relative


def _angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    cos = float(np.dot(v1, v2) / (n1 * n2))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def _chain_bend(pts: np.ndarray, chain) -> float:
    """Total bend along a finger: sum of the turn at PIP and at DIP.

    ~0 deg when the finger is straight, ~150-180 deg when fully curled.
    """
    mcp, pip, dip, tip = (pts[i] for i in chain)
    return _angle_between(pip - mcp, dip - pip) + _angle_between(dip - pip, tip - dip)


def extract_signals(world_pts: np.ndarray) -> List[float]:
    """Reduce 21 world landmarks to the six raw joint signals."""
    thumb_flexion = _chain_bend(world_pts, FINGER_CHAINS["thumb"])

    # Where the thumb tip sits across the palm, as a fraction of palm width
    # measured from the index knuckle toward the pinky knuckle, scaled to a
    # degree-like number so calibration files stay readable.
    #
    # This replaces an earlier wrist-angle measure that could only tell an open
    # hand from a fist. A pinch is neither: the fingers stay extended while the
    # thumb comes to meet the index tip, and the wrist angle barely moves, so
    # the robot thumb would swing across the palm instead of closing on the
    # index. Projecting the tip onto the palm axis separates the three cleanly:
    # spread out past the index reads negative, meeting the index reads near
    # zero, and folding across the palm toward the pinky reads positive.
    palm_axis = world_pts[PINKY_MCP] - world_pts[INDEX_MCP]
    palm_width = float(np.linalg.norm(palm_axis))
    if palm_width < 1e-9:
        thumb_abduction = 0.0
    else:
        offset = world_pts[THUMB_TIP] - world_pts[INDEX_MCP]
        thumb_abduction = float(np.dot(offset, palm_axis / palm_width)
                                / palm_width) * 100.0

    return [
        thumb_abduction,
        thumb_flexion,
        _chain_bend(world_pts, FINGER_CHAINS["index"]),
        _chain_bend(world_pts, FINGER_CHAINS["middle"]),
        _chain_bend(world_pts, FINGER_CHAINS["ring"]),
        _chain_bend(world_pts, FINGER_CHAINS["pinky"]),
    ]


class HandTracker:
    """Thin wrapper over the MediaPipe HandLandmarker task."""

    def __init__(
        self,
        model_path: Path = HAND_LANDMARKER_MODEL,
        *,
        prefer_hand: str = "Right",
        min_confidence: float = 0.5,
    ) -> None:
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision

        if not Path(model_path).exists():
            raise FileNotFoundError(
                f"hand_landmarker model missing: {model_path}\n"
                "Download it with:\n  curl -sSL -o models/hand_landmarker.task "
                "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
                "hand_landmarker/float16/1/hand_landmarker.task"
            )

        self.prefer_hand = prefer_hand
        options = vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=min_confidence,
            min_hand_presence_confidence=min_confidence,
            min_tracking_confidence=min_confidence,
        )
        with _quiet_native_stderr():
            self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._warmed_up = False

    def process(self, bgr_frame: np.ndarray, timestamp_ms: int) -> Optional[HandReading]:
        """Track one frame. Returns None when no hand is visible."""
        import cv2
        import mediapipe as mp

        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        if self._warmed_up:
            result = self._landmarker.detect_for_video(mp_image, timestamp_ms)
        else:
            # The first inference emits its own batch of native warnings.
            with _quiet_native_stderr():
                result = self._landmarker.detect_for_video(mp_image, timestamp_ms)
            self._warmed_up = True

        if not result.hand_world_landmarks:
            return None

        # Prefer the configured hand; otherwise take whatever is in view.
        index = 0
        for i, cat in enumerate(result.handedness):
            if cat and cat[0].category_name == self.prefer_hand:
                index = i
                break

        world = result.hand_world_landmarks[index]
        world_pts = np.array([[lm.x, lm.y, lm.z] for lm in world], dtype=np.float64)

        pts_2d = None
        if result.hand_landmarks:
            h, w = bgr_frame.shape[:2]
            pts_2d = np.array(
                [[lm.x * w, lm.y * h] for lm in result.hand_landmarks[index]],
                dtype=np.float32,
            )

        handedness = "?"
        if result.handedness and result.handedness[index]:
            handedness = result.handedness[index][0].category_name

        return HandReading(extract_signals(world_pts), handedness, pts_2d, world_pts)

    def close(self) -> None:
        try:
            self._landmarker.close()
        except Exception:
            pass
