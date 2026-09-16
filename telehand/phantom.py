"""Phantom 3-D hand: a motion-capture style skeletal hand from the 21 world landmarks.

Visualisation only. The renderer receives the HandReading the retargeter has
already consumed, keeps its own canonicalizer (no state shared with control),
runs after the command has gone out, and ``render`` never raises -- a failing
frame yields a placeholder panel and is counted.

What it draws
-------------
* 21 joints as shaded spheres, fingertips and the wrist emphasised;
* bones along the five chains plus the palm structure (wrist to each MCP and
  the MCP cross-links), as outlined cylinders whose thickness scales with
  depth;
* a floor grid with a drop shadow, depth fog, and painter's ordering, so depth
  reads at a glance;
* a wrist XYZ triad, and optionally (``joint_frames``) the local frame of
  every articulated joint from :class:`handpose.HandPose3D`, scaled by level:
  palm largest, finger roots medium, interior joints small, tips off by
  default. Triads are depth-sorted with the bones and joints.

Coordinates
-----------
World landmarks are canonicalised (chirality decided by the geometry, so the
phantom cannot flip when MediaPipe's label flickers), lightly smoothed, then
display-mirrored so the phantom moves the way the mirrored webcam image does.
Joint frames are built from the same smoothed canonical points as the joint
positions, so a triad is rigidly attached to its joint. A direction in
canonical coordinates maps to scene coordinates by the same mirror and axis
flip as the points: mirror x, y down->up, z away->toward, i.e. negation.
A fixed virtual camera looks at the hand from a 3/4 angle; the hand's own
rotation is preserved, so turning your hand turns the phantom.
"""

from __future__ import annotations

import time
from typing import List, Optional, Tuple

import cv2
import numpy as np

from . import handpose as hp

WRIST = 0
TIPS = (4, 8, 12, 16, 20)
PALM_PTS = (0, 5, 9, 13, 17)
CHAIN_BONES = [(0, 1), (1, 2), (2, 3), (3, 4),
               (0, 5), (5, 6), (6, 7), (7, 8),
               (0, 9), (9, 10), (10, 11), (11, 12),
               (0, 13), (13, 14), (14, 15), (15, 16),
               (0, 17), (17, 18), (18, 19), (19, 20)]
PALM_BONES = [(5, 9), (9, 13), (13, 17)]
ALL_FINGERS = ("thumb", "index", "middle", "ring", "pinky")

# MediaPipe world axes follow the image: x right, y down, z increasing away
# from the camera (smaller z = closer). Scene axes: x right, y up, z toward
# the viewer. Verified in tests/test_phantom.py: a fist's curled tips must
# come out nearer the viewer than its knuckles.
SCENE_AXES = np.array([1.0, -1.0, -1.0])
# Points are also display-mirrored (x negated) before SCENE_AXES, so a
# canonical *direction* maps to scene space as d * (-1, 1, 1) * SCENE_AXES = -d.
DIR_TO_SCENE = -1.0

# Joint-frame hierarchy: arrow length (m) and shaft thickness (px at the palm plane).
FRAME_LEVELS = {"palm": (0.028, 4), "root": (0.018, 3), "mid": (0.012, 3), "tip": (0.009, 2)}

# Palette (BGR).
BG_TOP = (44, 30, 20)
BG_BOTTOM = (70, 52, 36)
FOG = (58, 42, 30)
GRID = (86, 66, 48)
SHADOW = (34, 24, 16)
OUTLINE = (24, 20, 16)
BONE = (250, 238, 214)
PALM_BONE = (200, 184, 158)
JOINT = (255, 214, 130)
JOINT_HI = (255, 250, 236)
TIP = (120, 196, 255)
TIP_HI = (200, 236, 255)
WRIST_C = (255, 255, 255)
TEXT = (230, 224, 214)
AXIS_BGR = ((0, 0, 255), (0, 255, 0), (255, 0, 0))     # x red, y green, z blue


def _rot(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    cy, sy = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    cp, sp = np.cos(np.radians(pitch_deg)), np.sin(np.radians(pitch_deg))
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    return Rx @ Ry


def _mix(a, b, t):
    return tuple(int(round(x + (y - x) * t)) for x, y in zip(a, b))


def frame_level(name: str) -> str:
    if name == "palm":
        return "palm"
    if name.endswith("_tip"):
        return "tip"
    if name.endswith(("_cmc", "_mcp")) and not name.startswith("thumb_mcp"):
        return "root"
    return "mid"            # thumb MCP/IP, finger PIP/DIP


class PhantomHand:
    def __init__(self, assume_mirrored: bool = True, *, width: int = 380, hz: float = 30.0,
                 yaw: float = -35.0, pitch: float = 18.0, distance: float = 0.5,
                 focal: float = 860.0, smoothing: float = 0.5,
                 joint_frames: bool = False, tip_frames: bool = False) -> None:
        self.canon = hp.Canonicalizer(assume_mirrored=assume_mirrored)
        self.width = width
        self.hz = hz
        self.yaw, self.pitch = yaw, pitch
        self.distance, self.focal = distance, focal
        self.alpha = smoothing
        self.enabled = True
        self.joint_frames = joint_frames
        self.tip_frames = tip_frames
        self._smooth_canon: Optional[np.ndarray] = None
        self._lost = 0
        self._last_img: Optional[np.ndarray] = None
        self._last_t = 0.0
        self._bg_cache: dict = {}
        self.frames = 0
        self.rendered = 0
        self.failures = 0
        self.frame_failures = 0          # joint-frame construction failed; phantom still drawn
        self.consecutive_failures = 0
        self.total_ms = 0.0
        self.last_ms = 0.0
        self.last_pose: Optional[hp.HandPose3D] = None

    def toggle(self) -> bool:
        self.enabled = not self.enabled
        return self.enabled

    def toggle_frames(self) -> bool:
        self.joint_frames = not self.joint_frames
        return self.joint_frames

    # ------------------------------------------------------------- public

    def render(self, reading, height: int = 480) -> np.ndarray:
        """A (height x width) panel. Never raises."""
        self.frames += 1
        now = time.perf_counter()
        if (self._last_img is not None and self._last_img.shape[0] == height
                and self.hz > 0 and (now - self._last_t) < 1.0 / self.hz):
            return self._last_img
        try:
            img = self._render(reading, height)
            self.rendered += 1
            self.consecutive_failures = 0
        except Exception as exc:                          # visualisation must never reach control
            self.failures += 1
            self.consecutive_failures += 1
            if self.failures <= 3 or self.consecutive_failures in (30, 300):
                print(f"3D hand view: frame skipped ({type(exc).__name__}: {exc})")
            img = self._background(height)
            self._text(img, "3D hand view unavailable", height)
        finally:
            self.last_ms = (time.perf_counter() - now) * 1000.0
            self.total_ms += self.last_ms
        self._last_img = img
        self._last_t = now
        return img

    def summary(self) -> str:
        if self.rendered == 0:
            return "3D hand view: never rendered"
        return (f"3D hand view: {self.rendered} renders over {self.frames} frames, "
                f"{self.failures} failures caught ({self.frame_failures} joint-frame failures), "
                f"{self.total_ms / self.rendered:.2f} ms/render, joint frames "
                f"{'on' if self.joint_frames else 'off'}, chirality switches {self.canon.counts['switches']}")

    # ----------------------------------------------------------- pipeline

    def canonical(self, reading) -> Optional[np.ndarray]:
        if reading is None or reading.world_pts is None:
            return None
        return self.canon(reading.world_pts, getattr(reading, "handedness", None))

    @staticmethod
    def to_scene(canon: np.ndarray) -> np.ndarray:
        """Canonical -> display-mirrored -> palm-centred scene coordinates (metres)."""
        disp = hp.mirror_x(canon)                       # the phantom moves like the mirrored webcam
        centre = disp[list(PALM_PTS)].mean(axis=0)
        return (disp - centre) * SCENE_AXES

    def scene_points(self, reading) -> Optional[np.ndarray]:
        c = self.canonical(reading)
        return None if c is None else self.to_scene(c)

    def _render(self, reading, height: int) -> np.ndarray:
        img = self._background(height)
        canon = self.canonical(reading)
        if canon is None:
            self._lost += 1
            if self._lost > 15:
                self._smooth_canon = None
        else:
            self._lost = 0
            self._smooth_canon = (canon if self._smooth_canon is None
                                  else self.alpha * canon + (1 - self.alpha) * self._smooth_canon)
        if self._smooth_canon is None:
            self._text(img, "no hand tracked", height)
            return img
        pts = self.to_scene(self._smooth_canon)
        pose = None
        if self.joint_frames:
            # Same smoothed points as the positions, so triads stay attached.
            # A failure here drops the triads for this frame, not the hand.
            try:
                pose = hp.HandPose3D.from_landmarks(self._smooth_canon, fingers=ALL_FINGERS)
            except Exception:
                self.frame_failures += 1
                pose = None
        self.last_pose = pose
        self._draw_scene(img, pts, height, dim=self._lost > 0, pose=pose)
        return img

    # ------------------------------------------------------------ drawing

    def _background(self, height: int) -> np.ndarray:
        bg = self._bg_cache.get(height)
        if bg is None:
            t = np.linspace(0.0, 1.0, height)[:, None, None]
            top = np.array(BG_TOP, dtype=np.float64)[None, None, :]
            bot = np.array(BG_BOTTOM, dtype=np.float64)[None, None, :]
            bg = np.repeat((top + (bot - top) * t), self.width, axis=1).astype(np.uint8)
            self._bg_cache[height] = bg
        return bg.copy()

    def _text(self, img, msg, height):
        cv2.putText(img, msg, (12, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT, 1, cv2.LINE_AA)

    def _draw_scene(self, img: np.ndarray, pts: np.ndarray, height: int, dim: bool = False,
                    pose: Optional[hp.HandPose3D] = None) -> None:
        R = _rot(self.yaw, self.pitch)
        P = pts @ R.T                                   # camera space: x right, y up, z toward viewer
        D = self.distance
        f = self.focal * (height / 480.0)
        cx, cy = self.width / 2.0, height * 0.50

        def proj(p) -> Tuple[Tuple[int, int], float]:
            dz = max(D - p[2], 0.08)
            s = f / dz
            u = float(np.clip(cx + p[0] * s, -1e4, 1e4))
            v = float(np.clip(cy - p[1] * s, -1e4, 1e4))
            return (int(round(u)), int(round(v))), D / dz    # scale relative to the palm plane

        # Floor: a grid a little below the lowest joint, and the hand's shadow on it.
        floor_y = float(P[:, 1].min()) - 0.05
        for x in np.arange(-0.30, 0.301, 0.05):
            a, _ = proj(np.array([x, floor_y, -0.35])); b, _ = proj(np.array([x, floor_y, 0.35]))
            cv2.line(img, a, b, GRID, 1, cv2.LINE_AA)
        for z in np.arange(-0.35, 0.351, 0.05):
            a, _ = proj(np.array([-0.30, floor_y, z])); b, _ = proj(np.array([0.30, floor_y, z]))
            cv2.line(img, a, b, GRID, 1, cv2.LINE_AA)
        shadow2d = [proj(np.array([p[0], floor_y, p[2]]))[0] for p in P]
        for a, b in CHAIN_BONES + PALM_BONES:
            cv2.line(img, shadow2d[a], shadow2d[b], SHADOW, 5, cv2.LINE_AA)
        for i in range(21):
            cv2.circle(img, shadow2d[i], 4, SHADOW, -1, cv2.LINE_AA)

        # Depth ordering and fog.
        zmin, zmax = float(P[:, 2].min()), float(P[:, 2].max())
        span = max(zmax - zmin, 1e-3)

        def fog(colour, z):
            t = (z - zmin) / span                       # 1 = nearest
            k = 0.5 + 0.5 * t
            c = _mix(FOG, colour, k)
            return _mix(FOG, c, 0.6) if dim else c

        # kinds: 0 bone, 1 joint, 2 axis arrow. Sorted far -> near; at equal
        # depth bones, then joints, then arrows, so a triad sits on its sphere.
        items: List[Tuple[float, int, Tuple]] = []
        for a, b in CHAIN_BONES:
            items.append(((P[a, 2] + P[b, 2]) / 2.0, 0, (a, b, False)))
        for a, b in PALM_BONES:
            items.append(((P[a, 2] + P[b, 2]) / 2.0, 0, (a, b, True)))
        for i in range(21):
            items.append((P[i, 2] + 0.004, 1, (i,)))

        # Joint frames: every articulated joint when pose is given, else the wrist alone.
        triads = []
        if pose is not None:
            for name, j in pose.joints.items():
                level = frame_level(name)
                if level == "tip" and not self.tip_frames:
                    continue
                if j.degenerate:
                    continue
                Rw = pose.wrist_rotation @ j.rotation           # axes in canonical world coordinates
                triads.append((j.landmark, Rw, level))
        else:
            try:
                triads.append((WRIST, hp.palm_frame(self._smooth_canon).R, "palm"))
            except Exception:
                pass
        for lm, Rw, level in triads:
            length, thick = FRAME_LEVELS[level]
            origin = pts[lm]
            for i in range(3):
                end = origin + (DIR_TO_SCENE * Rw[:, i]) * length
                oc, ec = R @ origin, R @ end
                items.append(((oc[2] + ec[2]) / 2.0 + 0.006, 2, (origin, end, i, thick)))

        items.sort(key=lambda it: (it[0], it[1]))
        p2 = [proj(p) for p in P]
        for depth, kind, data in items:
            if kind == 0:
                a, b, palm = data
                (pa, sa), (pb, sb) = p2[a], p2[b]
                s = (sa + sb) / 2.0
                thick = max(2, int(round((5.5 if palm else 8.0) * s)))
                colour = fog(PALM_BONE if palm else BONE, depth)
                cv2.line(img, pa, pb, OUTLINE, thick + 4, cv2.LINE_AA)
                cv2.line(img, pa, pb, colour, thick, cv2.LINE_AA)
                cv2.line(img, pa, pb, _mix(colour, JOINT_HI, 0.35), max(1, thick // 3), cv2.LINE_AA)
            elif kind == 1:
                (i,) = data
                (c, s) = p2[i]
                base = 10.0 if i == WRIST else (8.0 if i in TIPS else 6.5)
                r = max(3, int(round(base * s)))
                fill = WRIST_C if i == WRIST else (TIP if i in TIPS else JOINT)
                hi = JOINT_HI if i == WRIST else (TIP_HI if i in TIPS else JOINT_HI)
                cv2.circle(img, c, r + 2, OUTLINE, -1, cv2.LINE_AA)
                cv2.circle(img, c, r, fog(fill, depth), -1, cv2.LINE_AA)
                cv2.circle(img, (c[0] - r // 3, c[1] - r // 3), max(1, r // 3), fog(hi, depth), -1, cv2.LINE_AA)
            else:
                origin, end, i, thick = data
                (o2, s), (e2, _) = proj(R @ origin), proj(R @ end)
                t = max(2, int(round(thick * s)))
                if np.hypot(e2[0] - o2[0], e2[1] - o2[1]) < 4:            # pointing at the viewer
                    cv2.circle(img, o2, t + 1, OUTLINE, -1, cv2.LINE_AA)
                    cv2.circle(img, o2, t, AXIS_BGR[i], -1, cv2.LINE_AA)
                    continue
                cv2.arrowedLine(img, o2, e2, OUTLINE, t + 3, cv2.LINE_AA, tipLength=0.3)
                cv2.arrowedLine(img, o2, e2, AXIS_BGR[i], t, cv2.LINE_AA, tipLength=0.3)

        cv2.putText(img, "3D hand" + ("  + joint frames" if pose is not None else ""),
                    (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, TEXT, 1, cv2.LINE_AA)
        cv2.putText(img, f"view {self.yaw:+.0f} / {self.pitch:+.0f}   [ ] rotate   f frames",
                    (12, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, _mix(TEXT, FOG, 0.4), 1, cv2.LINE_AA)
