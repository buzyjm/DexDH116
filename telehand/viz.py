"""Live visualization: what the camera sees, and what the hand is doing.

The panel is deliberately driven by *commanded* joint angles rather than the
tracker output, so what you see is what was actually sent over RS485 after
clamping and rate limiting -- if the hand is not keeping up with you, the panel
shows that rather than hiding it.
"""

from __future__ import annotations

import json
import math
from typing import Optional

import cv2
import numpy as np

from .hand import JOINT_NAMES
from .paths import VIEW_FILE, ensure_state_dir


def load_view():
    """Viewpoint saved from the panel, or None for the renderer's default."""
    try:
        d = json.loads(VIEW_FILE.read_text())
        return float(d["yaw"]), float(d["pitch"])
    except Exception:
        return None


def save_view(yaw: float, pitch: float) -> None:
    ensure_state_dir()
    VIEW_FILE.write_text(json.dumps({"yaw": yaw, "pitch": pitch}) + "\n")

_renderer = None
_renderer_failed = False


def _hand_renderer():
    """Lazily build the URDF renderer; fall back to the schematic if absent."""
    global _renderer, _renderer_failed
    if _renderer is None and not _renderer_failed:
        try:
            from .handview import HandRenderer
            _renderer = HandRenderer("R")
        except Exception as exc:
            _renderer_failed = True
            print(f"hand view: URDF render unavailable ({exc}); using the schematic")
    return _renderer

PANEL_W = 560
VIEW_W, VIEW_H = 400, 376
VIEW_X, VIEW_Y = 8, 36        # where the hand view sits inside the panel
SHORT = ["T.abd", "T.flex", "index", "mid", "ring", "pinky"]
BG = (26, 24, 22)
FG = (235, 235, 235)
MUTED = (140, 138, 136)
ACCENT_ON = (90, 200, 90)
ACCENT_OFF = (60, 170, 235)
WARN = (60, 60, 235)
BONE = (200, 160, 90)
MEASURED_LABEL = (250, 200, 90)
CONTACT_NEAR = (90, 210, 255)
CONTACT_DEEP = (60, 80, 255)

# MediaPipe hand topology, for the camera overlay.
CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
]

# Schematic finger layout: (x offset from palm centre, base length scale).
_FINGERS = [
    ("index", -0.30, 1.00),
    ("middle", -0.10, 1.10),
    ("ring", 0.10, 1.00),
    ("pinky", 0.30, 0.82),
]
_SEGMENTS = (0.42, 0.32, 0.26)  # proportion of finger length per phalanx


def draw_skeleton(frame: np.ndarray, landmarks_2d: Optional[np.ndarray]) -> None:
    """Overlay the tracked hand on the camera image."""
    if landmarks_2d is None:
        return
    pts = landmarks_2d.astype(int)
    for a, b in CONNECTIONS:
        cv2.line(frame, tuple(pts[a]), tuple(pts[b]), BONE, 2, cv2.LINE_AA)
    for i, (x, y) in enumerate(pts):
        r, c = (5, (255, 255, 255)) if i in (4, 8, 12, 16, 20) else (3, (210, 210, 210))
        cv2.circle(frame, (int(x), int(y)), r, c, -1, cv2.LINE_AA)


def _curl_chain(origin, direction, length, total_bend_deg, curl_sign):
    """Walk a finger outward, splitting the total bend evenly across its joints.

    The bend is applied at every joint including the knuckle, so a full curl
    sweeps the tip roughly 200 deg back over the palm the way a real finger
    does. Bending only the distal joints just fans the finger out sideways.
    """
    pts = [np.array(origin, dtype=np.float64)]
    ang = math.atan2(direction[1], direction[0])
    per_joint = math.radians(total_bend_deg) * curl_sign / len(_SEGMENTS)
    for frac in _SEGMENTS:
        ang += per_joint
        step = np.array([math.cos(ang), math.sin(ang)]) * length * frac
        pts.append(pts[-1] + step)
    return pts


def draw_hand_schematic(canvas, cx, cy, size, angles, limits) -> None:
    """Schematic right hand driven by the six commanded joint angles."""
    palm_w, palm_h = size * 0.52, size * 0.42
    finger_len = size * 0.46

    # Palm.
    p0 = (int(cx - palm_w / 2), int(cy - palm_h / 2))
    p1 = (int(cx + palm_w / 2), int(cy + palm_h / 2))
    cv2.rectangle(canvas, p0, p1, (70, 66, 62), -1, cv2.LINE_AA)
    cv2.rectangle(canvas, p0, p1, (110, 105, 100), 1, cv2.LINE_AA)

    # Four fingers: joint ids 3..6, angle 0 = straight, max = curled.
    for i, (_name, xoff, lscale) in enumerate(_FINGERS):
        joint = 2 + i  # index into angles
        lim = limits[joint]
        span = max(lim.max_angle - lim.min_angle, 1e-6)
        frac = (angles[joint] - lim.min_angle) / span
        base = (cx + xoff * palm_w, cy - palm_h / 2)
        # Straight up is -90 deg in image coords; curl folds toward the palm.
        # Foreshorten as it curls, so a fist stays compact over the knuckles
        # instead of sweeping a wide arc off the side of the palm.
        reach = finger_len * lscale * (1.0 - 0.34 * frac)
        pts = _curl_chain(base, (0, -1), reach, frac * 235.0, +1.0)
        pts = [(int(p[0]), int(p[1])) for p in pts]
        for a, b in zip(pts, pts[1:]):
            cv2.line(canvas, a, b, BONE, 4, cv2.LINE_AA)
        for p in pts:
            cv2.circle(canvas, p, 3, (240, 240, 240), -1, cv2.LINE_AA)

    # Thumb: abduction swings the base away from the palm, flexion curls it.
    ab_lim, fl_lim = limits[0], limits[1]
    ab_frac = (angles[0] - ab_lim.min_angle) / max(ab_lim.max_angle - ab_lim.min_angle, 1e-6)
    fl_frac = (angles[1] - fl_lim.min_angle) / max(fl_lim.max_angle - fl_lim.min_angle, 1e-6)
    # 0 -> tucked up alongside the palm, 1 -> swung out to the side.
    # Image coords put y downward, so 265 deg points up and 200 deg points left.
    theta = math.radians(265.0 - 65.0 * ab_frac)
    base = (cx - palm_w / 2, cy + palm_h * 0.20)
    direction = (math.cos(theta), math.sin(theta))
    pts = _curl_chain(base, direction, finger_len * 0.80, fl_frac * 150.0, -1.0)
    pts = [(int(p[0]), int(p[1])) for p in pts]
    for a, b in zip(pts, pts[1:]):
        cv2.line(canvas, a, b, (150, 190, 230), 4, cv2.LINE_AA)
    for p in pts:
        cv2.circle(canvas, p, 3, (240, 240, 240), -1, cv2.LINE_AA)


def draw_strip(canvas, x, y, w, h, angles, limits, alarms, selected=None,
               measured=None) -> None:
    """Six vertical bars in one row: the whole hand state in 50 px of height."""
    n = len(JOINT_NAMES)
    col = w / n
    bar_w = int(min(26, col * 0.34))
    for i, name in enumerate(SHORT):
        lim = limits[i]
        span = max(lim.max_angle - lim.min_angle, 1e-6)
        frac = float(np.clip((angles[i] - lim.min_angle) / span, 0.0, 1.0))
        cx = int(x + col * (i + 0.5))
        top, bottom = y + 13, y + h - 13
        if selected == i:
            cv2.rectangle(canvas, (int(x + col * i) + 2, y), (int(x + col * (i + 1)) - 2, y + h),
                          (58, 55, 52), -1)
        cv2.putText(canvas, name, (cx - len(name) * 3, y + 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32,
                    FG if selected == i else MUTED, 1, cv2.LINE_AA)
        cv2.rectangle(canvas, (cx - bar_w // 2, top), (cx + bar_w // 2, bottom), (58, 55, 52), -1)
        fill = int((bottom - top) * frac)
        colour = WARN if alarms and alarms[i] else ACCENT_ON
        if fill > 0:
            cv2.rectangle(canvas, (cx - bar_w // 2, bottom - fill), (cx + bar_w // 2, bottom),
                          colour, -1)
        if measured is not None:
            mfrac = float(np.clip((measured[i] - lim.min_angle) / span, 0.0, 1.0))
            my = bottom - int((bottom - top) * mfrac)
            if abs(measured[i] - angles[i]) >= 2.0:
                cv2.line(canvas, (cx - bar_w // 2 - 3, my), (cx + bar_w // 2 + 3, my),
                         MEASURED_LABEL, 2, cv2.LINE_AA)
        txt = f"{angles[i]:.0f}"
        cv2.putText(canvas, txt, (cx - len(txt) * 4, y + h - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, FG, 1, cv2.LINE_AA)


def draw_bars(canvas, x, y, w, angles, limits, alarms, selected=None) -> int:
    """One labelled bar per joint, showing position within its real limits."""
    row_h = 24
    for i, name in enumerate(JOINT_NAMES):
        lim = limits[i]
        span = max(lim.max_angle - lim.min_angle, 1e-6)
        frac = float(np.clip((angles[i] - lim.min_angle) / span, 0.0, 1.0))
        top = y + i * row_h

        if selected == i:
            cv2.rectangle(canvas, (x - 6, top - 3), (x + w, top + 21), (58, 55, 52), -1)
            cv2.putText(canvas, ">", (x - 12, top + 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, ACCENT_OFF, 1, cv2.LINE_AA)
        label_colour = FG if selected == i else MUTED
        cv2.putText(canvas, f"{i+1} " + name.replace("_", " "), (x, top + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, label_colour, 1, cv2.LINE_AA)
        cv2.putText(canvas, f"{angles[i]:.0f}/{lim.max_angle:.0f}", (x + w - 46, top + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, FG, 1, cv2.LINE_AA)

        by = top + 14
        cv2.rectangle(canvas, (x, by), (x + w - 52, by + 7), (58, 55, 52), -1)
        fill = int((w - 52) * frac)
        colour = WARN if alarms and alarms[i] else ACCENT_ON
        if fill > 0:
            cv2.rectangle(canvas, (x, by), (x + fill, by + 7), colour, -1)
    return y + len(JOINT_NAMES) * row_h


def render_panel(height, angles, limits, *, engaged, fps, alarms=None,
                 tracked=False, dry_run=False, selected=None, title=None,
                 keys=None, measured=None, yaw=None, pitch=None, zoom=1.0) -> np.ndarray:
    panel = np.full((height, PANEL_W, 3), BG, np.uint8)

    accent = ACCENT_ON if engaged else ACCENT_OFF
    label = title if title is not None else ("ENGAGED" if engaged else "PAUSED")
    if dry_run:
        label += "  (DRY RUN)"
    cv2.rectangle(panel, (0, 0), (PANEL_W, 30), accent, -1)
    cv2.putText(panel, label, (12, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(panel, f"{fps:4.1f} fps", (PANEL_W - 78, 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.46, (20, 20, 20), 1, cv2.LINE_AA)

    vx, vy = 8, 36
    contacts = []
    renderer = _hand_renderer()
    if renderer is not None:
        kw = {"zoom": zoom}
        if yaw is not None:
            kw["yaw"] = yaw
        if pitch is not None:
            kw["pitch"] = pitch
        view = renderer.render(angles, size=(VIEW_W, VIEW_H), measured=measured, **kw).copy()
        # Joint values next to the finger they drive, so the number and the
        # thing it moves are read together instead of in separate tables.
        tips = {}
        if hasattr(renderer, "fingertip_pixels"):
            try:
                tips = renderer.fingertip_pixels(angles, (VIEW_W, VIEW_H), **kw)
            except Exception:
                tips = {}
        # Self-contacts: rigid meshes overlapping where the real hand would be
        # pressing. Marked rather than hidden, because "the thumb is pressing on
        # the middle finger" is the useful reading of it.
        contacts = []
        if hasattr(renderer, "contacts"):
            try:
                contacts = renderer.contacts(angles, (VIEW_W, VIEW_H), **kw)
            except Exception:
                contacts = []
        for (px, py), depth, label in contacts:
            t = min(depth / 10.0, 1.0)
            colour = tuple(int(a + (b - a) * t) for a, b in zip(CONTACT_NEAR, CONTACT_DEEP))
            radius = int(5 + 5 * t)
            cv2.circle(view, (int(px), int(py)), radius, colour, 2, cv2.LINE_AA)
            cv2.circle(view, (int(px), int(py)), 2, colour, -1, cv2.LINE_AA)

        per_finger = {"index": f"{angles[2]:.0f}", "middle": f"{angles[3]:.0f}",
                      "ring": f"{angles[4]:.0f}", "pinky": f"{angles[5]:.0f}",
                      "thumb": f"{angles[0]:.0f}/{angles[1]:.0f}"}
        for finger, (px, py) in tips.items():
            txt = per_finger.get(finger)
            if txt is None:
                continue
            org = (int(px) + 8, int(py) + 4)
            cv2.putText(view, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(view, txt, org, cv2.FONT_HERSHEY_SIMPLEX, 0.44, (120, 235, 255), 1, cv2.LINE_AA)
            cv2.circle(view, (int(px), int(py)), 3, (120, 235, 255), -1, cv2.LINE_AA)
        panel[vy:vy + VIEW_H, vx:vx + VIEW_W] = view
        cv2.rectangle(panel, (vx, vy), (vx + VIEW_W, vy + VIEW_H), (58, 55, 52), 1)
        if measured is not None:
            lag = [c - m for c, m in zip(angles, measured)]
            worst = max(range(len(lag)), key=lambda i: abs(lag[i]))
            if abs(lag[worst]) >= 2.0:
                cv2.putText(view, f"{SHORT[worst]} short {lag[worst]:+.0f} deg",
                            (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                            MEASURED_LABEL, 1, cv2.LINE_AA)
            else:
                cv2.putText(view, "tracking", (6, 16), cv2.FONT_HERSHEY_SIMPLEX,
                            0.42, ACCENT_ON, 1, cv2.LINE_AA)
    else:
        cv2.putText(panel, "COMMANDED POSE", (vx + 4, vy + 16), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, MUTED, 1, cv2.LINE_AA)
        draw_hand_schematic(panel, vx + VIEW_W // 2, vy + VIEW_H // 2, 200, angles, limits)

    sx = vx + VIEW_W + 10
    cv2.putText(panel, "hand tracked" if tracked else "NO HAND", (sx, vy + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, ACCENT_ON if tracked else WARN, 1, cv2.LINE_AA)
    if contacts:
        cv2.putText(panel, "CONTACT", (sx, vy + 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, CONTACT_NEAR, 1, cv2.LINE_AA)
        for i, (_xy, depth, label) in enumerate(contacts[:3]):
            cv2.putText(panel, f"{label} {depth:.0f}mm", (sx, vy + 58 + i * 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, CONTACT_NEAR, 1, cv2.LINE_AA)
    if alarms and any(alarms):
        cv2.putText(panel, "ALARM", (sx, vy + 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.44, WARN, 1, cv2.LINE_AA)
        cv2.putText(panel, str(alarms), (sx, vy + 58), cv2.FONT_HERSHEY_SIMPLEX,
                    0.3, WARN, 1, cv2.LINE_AA)
    if keys is None:
        keys = ["SPACE engage", "o  open hand", "c  clear alarms", "",
                "drag   orbit view", "wheel  zoom", "dbl-clk reset",
                "v  save view", "", "q  quit"]
    for i, k in enumerate(keys):
        cv2.putText(panel, k, (sx, vy + 96 + i * 17), cv2.FONT_HERSHEY_SIMPLEX,
                    0.36, MUTED, 1, cv2.LINE_AA)

    draw_strip(panel, 14, vy + VIEW_H + 8, PANEL_W - 28, height - (vy + VIEW_H) - 14,
               angles, limits, alarms, selected, measured)
    return panel


def compose(frame, panel) -> np.ndarray:
    """Camera on the left, hand state on the right."""
    h = max(frame.shape[0], panel.shape[0])
    out = np.full((h, frame.shape[1] + panel.shape[1], 3), BG, np.uint8)
    out[:frame.shape[0], :frame.shape[1]] = frame
    out[:panel.shape[0], frame.shape[1]:] = panel
    return out


class ViewControl:
    """Mouse orbit for the hand view inside a composed teleop window.

    The window is the camera frame with the panel beside it, so the hand view
    occupies a known rectangle offset by the camera's width. Dragging inside it
    orbits, the wheel zooms, a double click restores the default, and `v` on the
    keyboard saves the current angle for next time.
    """

    DRAG_PER_DEGREE = 3.0

    def __init__(self, camera_width: int = 640) -> None:
        saved = load_view()
        self.yaw, self.pitch = saved if saved else (None, None)
        self.zoom = 1.0
        self.camera_width = camera_width
        self._last = None
        self._dirty = False

    @property
    def rect(self):
        x0 = self.camera_width + VIEW_X
        return x0, VIEW_Y, x0 + VIEW_W, VIEW_Y + VIEW_H

    def inside(self, x, y) -> bool:
        x0, y0, x1, y1 = self.rect
        return x0 <= x <= x1 and y0 <= y <= y1

    def attach(self, window: str) -> None:
        cv2.setMouseCallback(window, self._on_mouse)

    def _defaults(self):
        from .handview import MJ_AZIMUTH, MJ_ELEVATION
        return MJ_AZIMUTH, MJ_ELEVATION

    def _on_mouse(self, event, x, y, flags, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and self.inside(x, y):
            self._last = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self._last = None
        elif event == cv2.EVENT_MOUSEMOVE and self._last is not None:
            dx, dy = x - self._last[0], y - self._last[1]
            self._last = (x, y)
            dy_, dp_ = self._defaults()
            self.yaw = (self.yaw if self.yaw is not None else dy_) + dx / self.DRAG_PER_DEGREE
            pitch = (self.pitch if self.pitch is not None else dp_) - dy / self.DRAG_PER_DEGREE
            self.pitch = float(np.clip(pitch, -89, 89))
        elif event == cv2.EVENT_MOUSEWHEEL and self.inside(x, y):
            up = flags > 0
            self.zoom = float(np.clip(self.zoom * (1.15 if up else 1 / 1.15), 0.4, 4.0))
        elif event == cv2.EVENT_LBUTTONDBLCLK and self.inside(x, y):
            self.yaw, self.pitch = self._defaults()
            self.zoom = 1.0

    def handle_key(self, key: int) -> bool:
        """Keyboard fallbacks; returns True when the key was consumed."""
        dy_, dp_ = self._defaults()
        y = self.yaw if self.yaw is not None else dy_
        p = self.pitch if self.pitch is not None else dp_
        if key == ord("h"):
            self.yaw = y - 10
        elif key == ord("l"):
            self.yaw = y + 10
        elif key == ord("j"):
            self.pitch = max(-89, p - 10)
        elif key == ord("k"):
            self.pitch = min(89, p + 10)
        elif key == ord("r"):
            self.yaw, self.pitch = dy_, dp_
            self.zoom = 1.0
        elif key == ord("v"):
            save_view(y, p)
            print(f"view saved: yaw {y:.0f}  pitch {p:.0f}  ({VIEW_FILE.name})")
        else:
            return False
        return True

    @property
    def kwargs(self):
        return {"yaw": self.yaw, "pitch": self.pitch, "zoom": self.zoom}
