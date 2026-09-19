"""Kinematic retargeting: solve the six actuators so the robot's fingertips
reproduce the geometry of the operator's hand.

No pose library. Each frame, the operator's fingertip layout is expressed in
the robot's palm frame, scaled to the robot's finger length, and a small
damped Gauss-Newton solve finds the firmware angles whose forward kinematics
best match it. What is matched are *vectors between fingertips* (thumb to each
finger, palm centre to each tip) rather than absolute positions -- that is
what a grasp cares about, and it is insensitive to where the hand is.

Everything the solver knows about the hardware comes from `kinematics`:
vendor geometry and coupling, the photo-measured finger curve, the contact-
fitted thumb curve, and the abduction-dependent flexion envelope.
"""

from __future__ import annotations

from typing import List, Optional, Sequence

import numpy as np

from .kinematics import (HandModel, FINGER_ORDER, FIRMWARE_MAX_DEG,
                         thumb_flex_max)
from .profile import HandProfile

WRIST, THUMB_TIP = 0, 4
THUMB_CMC, THUMB_MCP, THUMB_IP = 1, 2, 3
DIR_SCALE = 0.04   # metres: a fully wrong thumb direction costs as much as a 40 mm miss
MCP = {"index": 5, "middle": 9, "ring": 13, "pinky": 17}
TIP = {"thumb": 4, "index": 8, "middle": 12, "ring": 16, "pinky": 20}
FINGERS = ["thumb"] + FINGER_ORDER


class FKRetargeter:
    def __init__(self, limits, *, iters: int = 6, damping: float = 0.15,
                 w_thumb: float = 2.0, w_palm: float = 1.0, smoothing: float = 0.35,
                 deadband_deg: float = 0.8, direction: str = "R",
                 near_dist: float = 0.05, reg_open: float = 5e-5,
                 gate_floor: float = 0.15, w_dir: float = 1.5,
                 profile: Optional[HandProfile] = None) -> None:
        self.model = HandModel(direction)
        self.limits = limits
        self.iters, self.damping = iters, damping
        self.w_thumb, self.w_palm = w_thumb, w_palm
        self.smoothing, self.deadband = smoothing, deadband_deg
        self.near_dist = near_dist      # metres; thumb-finger weight is full inside this
        self.reg_open = reg_open        # metres-per-degree pull toward the open pose
        self.gate_floor = gate_floor    # thumb-finger weight never fades below this fraction
        self.w_dir = w_dir              # weight of the thumb pointing-direction term
        self.q = np.zeros(6)                 # warm start, firmware degrees
        self._filtered: Optional[np.ndarray] = None
        self._held: Optional[np.ndarray] = None
        self.last_residual = float("nan")
        self.scale = float("nan")

        # Robot palm landmarks from the model: MCP origins and a wrist point
        # below them. The robot's palm frame is built by the *same* function
        # as the operator's, so any convention mismatch cancels out.
        zero = self.model.joints_from_firmware([0.0] * 6)
        bases = {f: self.model.chain_points(f, zero[f])[0] for f in FINGER_ORDER}
        self.palm_centre = np.mean(list(bases.values()), axis=0)
        fake = np.zeros((21, 3))
        fake[WRIST] = self.palm_centre - np.array([0.0, 0.0, 0.09])
        for f in FINGER_ORDER:
            fake[MCP[f]] = bases[f]
        self.R_robot = self.human_frame(fake, "Right")
        self.robot_palm_width = float(np.linalg.norm(bases["index"] - bases["pinky"]))
        # Robot finger lengths at zero, for per-finger scaling against a profile.
        self.robot_len = {f: float(np.linalg.norm(self.model.chain_points(f, zero[f])[-1] - bases[f]))
                          for f in FINGER_ORDER}
        tc = self.model.chain_points("thumb", zero["thumb"])
        self.robot_len["thumb"] = float(np.linalg.norm(tc[-1] - tc[0]))
        self.profile = profile
        self.finger_scale = None
        if profile is not None:
            self.finger_scale = {f: self.robot_len[f] / max(profile.finger_len.get(f, 0.0), 1e-6)
                                 for f in FINGERS}

    # ------------------------------------------------------------ targets

    def human_frame(self, pts: np.ndarray, handedness: str):
        """Rotation whose columns are the palm axes, in the robot's convention:
        z along the fingers, y toward the index/thumb side, x out of the palm."""
        # Palm plane from wrist + four MCPs (least squares), which is far less
        # sensitive to MediaPipe's depth noise than any three points alone.
        palm_pts = pts[[WRIST] + [MCP[f] for f in FINGER_ORDER]]
        c = palm_pts.mean(axis=0)
        _, _, vt = np.linalg.svd(palm_pts - c)
        n = vt[2]
        y0 = pts[MCP["index"]] - pts[MCP["pinky"]]
        z0 = pts[MCP["middle"]] - pts[WRIST]
        # Orient the normal out of the palm: for a right hand that is y0 x z0.
        if np.dot(n, np.cross(y0, z0)) < 0:
            n = -n
        z = z0 - n * np.dot(z0, n); z /= np.linalg.norm(z) + 1e-9
        y = np.cross(z, n); y /= np.linalg.norm(y) + 1e-9
        x = n
        if handedness == "Left":       # mirror so a left hand drives the right robot
            x = -x
        return np.stack([x, y, z], axis=1)

    def targets(self, pts: np.ndarray, handedness: str):
        R = self.human_frame(pts, handedness)
        to_robot = lambda v: R.T @ v
        # Pose-invariant scale: palm width, not a finger length (which shortens
        # as the finger curls and would inflate every target on a fist).
        h_width = np.linalg.norm(pts[MCP["index"]] - pts[MCP["pinky"]])
        self.scale = self.robot_palm_width / max(float(h_width), 1e-6)
        palm = np.mean([pts[MCP[f]] for f in FINGER_ORDER], axis=0)
        # With a profile each finger is scaled by its own length ratio, so an
        # operator with long fingers is not asked to over-curl to reach targets
        # the robot cannot span; without one, palm width serves for all.
        sc = self.finger_scale or {f: self.scale for f in FINGERS}
        tips = {f: to_robot(pts[TIP[f]] - palm) * sc[f] for f in FINGERS}
        vecs, weights = [], []
        for f in FINGER_ORDER:                       # thumb -> finger: what a grasp is
            v = tips[f] - tips["thumb"]
            # A thumb-to-finger vector only carries grasp intent when the two
            # are close. Far apart it mostly encodes hand proportions the robot
            # cannot reproduce, so let it fade rather than drag the thumb.
            gate = float(np.clip(self.near_dist / max(np.linalg.norm(v), 1e-6), self.gate_floor, 1.0))
            vecs.append(v); weights.append(self.w_thumb * gate)
        for a, b in zip(FINGER_ORDER, FINGER_ORDER[1:]):   # neighbouring fingers
            vecs.append(tips[b] - tips[a]); weights.append(0.5 * self.w_palm)
        for f in FINGER_ORDER:                       # palm -> finger tip: curl
            vecs.append(tips[f]); weights.append(self.w_palm)
        # Thumb *direction* (not position): the two thumbs sit differently on
        # their palms, but "which way is it pointing" transfers. This is what
        # tells a thumbs-up from a thumb folded over a fist.
        tdir = to_robot(pts[TIP["thumb"]] - pts[THUMB_CMC]); tdir /= np.linalg.norm(tdir) + 1e-9
        vecs.append(tdir * DIR_SCALE); weights.append(self.w_dir)
        # No palm -> thumb term: the two thumbs sit differently on their palms,
        # and pinning the robot thumb to the human thumb's position fights the
        # thumb-to-finger vectors that actually matter.
        return np.array(vecs), np.array(weights)

    # ------------------------------------------------------------ forward

    def robot_vecs(self, q_fw: Sequence[float]) -> np.ndarray:
        joints = self.model.joints_from_firmware(q_fw)
        tips = {f: self.R_robot.T @ (self.model.fingertip(f, joints[f]) - self.palm_centre)
                for f in FINGERS}
        out = [tips[f] - tips["thumb"] for f in FINGER_ORDER]
        out += [tips[b] - tips[a] for a, b in zip(FINGER_ORDER, FINGER_ORDER[1:])]
        out += [tips[f] for f in FINGER_ORDER]
        base = self.R_robot.T @ (self.model.chain_points("thumb", joints["thumb"])[1] - self.palm_centre)
        tdir = tips["thumb"] - base; tdir /= np.linalg.norm(tdir) + 1e-9
        out.append(tdir * DIR_SCALE)
        return np.array(out)

    def clamp(self, q: np.ndarray) -> np.ndarray:
        q = np.clip(q, 0.0, FIRMWARE_MAX_DEG)
        q[1] = min(q[1], thumb_flex_max(q[0]))
        return q

    # -------------------------------------------------------------- solve

    def solve(self, target: np.ndarray, weights: np.ndarray, q0: np.ndarray) -> np.ndarray:
        q = self.clamp(q0.copy())
        w = np.repeat(np.sqrt(weights), 3)
        eps = 0.5
        for _ in range(self.iters):
            base = (self.robot_vecs(q) - target).ravel() * w
            J = np.empty((base.size, 6))
            for i in range(6):
                # Perturb into the feasible side: at an upper limit a +eps step
                # would be clamped to nothing, freezing the joint there.
                dq = q.copy()
                h = eps if self.clamp(dq + np.eye(6)[i] * eps)[i] > q[i] + 1e-9 else -eps
                dq[i] += h
                J[:, i] = ((self.robot_vecs(self.clamp(dq)) - target).ravel() * w - base) / h
            JtJ = J.T @ J
            # Relative damping (Levenberg-Marquardt) plus an absolute floor, so a
            # joint pinned at a limit -- whose finite-difference column is zero --
            # cannot make the system singular. The reg_open term is a weak pull
            # toward the open pose; it decides directions the targets leave free.
            H = JtJ + (self.damping * JtJ.trace() / 6 + 1e-6 + self.reg_open ** 2) * np.eye(6)
            g = J.T @ base + (self.reg_open ** 2) * q
            step = np.linalg.lstsq(H, -g, rcond=None)[0]
            step = np.clip(step, -12.0, 12.0)
            q = self.clamp(q + step)
        self.last_residual = float(np.linalg.norm((self.robot_vecs(q) - target).ravel() * w)
                                   / np.sqrt(len(target)))
        return q

    def solve_multistart(self, target, weights, starts=None):
        if starts is None:
            starts = [self.q, np.zeros(6), np.array([30.0, 15.0, 40.0, 40.0, 40.0, 40.0])]
        best = None
        for s0 in starts:
            q = self.solve(target, weights, np.asarray(s0, dtype=np.float64))
            if best is None or self.last_residual < best[0]:
                best = (self.last_residual, q)
        self.last_residual = best[0]
        return best[1]

    def thumb_extended(self, pts: np.ndarray) -> bool:
        """True when the thumb is straight and clear of every fingertip.

        Thumbs-up and an open hand both look like this. The hardware's best
        rendering of either is the thumb fully extended and swung out, so the
        solver's compromise (a thumb bent toward the fingers it cannot reach)
        is replaced by exactly that.
        """
        a = pts[THUMB_MCP] - pts[THUMB_CMC]
        b = pts[THUMB_TIP] - pts[THUMB_IP]
        straight = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9) > 0.85
        clear = min(np.linalg.norm(pts[TIP[f]] - pts[THUMB_TIP]) for f in FINGER_ORDER) * self.scale > 0.06
        return bool(straight and clear)

    def __call__(self, pts: np.ndarray, handedness: str = "Right") -> List[float]:
        pts = np.asarray(pts, dtype=np.float64)
        target, weights = self.targets(pts, handedness)
        self.q = self.solve(target, weights, self.q)
        if self.thumb_extended(pts):
            self.q[0] = 0.0
            self.q[1] = 0.0
        a = self.smoothing
        self._filtered = self.q.copy() if self._filtered is None else a * self.q + (1 - a) * self._filtered
        if self._held is None:
            self._held = self._filtered.copy()
        else:
            move = np.abs(self._filtered - self._held) > self.deadband
            self._held = np.where(move, self._filtered, self._held)
        return [lim.clamp(float(v)) for lim, v in zip(self.limits, self._held)]

    def reset(self) -> None:
        self._filtered = None
        self._held = None


class PinchCloser:
    """Close the last centimetre of a pinch by feel.

    The solver reproduces the operator's fingertip geometry faithfully -- and
    a human pinch, as the tracker sees it, usually leaves a gap. When the
    operator's thumb-to-finger distance says "pinch", this ramps extra curl
    onto that finger until contact is felt: thumb-tip pressure, or the finger
    stalling short of its command because the thumb is in the way. The extra
    is held while the intent lasts and released afterwards.
    """

    def __init__(self, intent_mm: float = 35.0, step_deg: float = 1.5, max_extra_deg: float = 10.0,
                 pressure_on: float = 0.03, stall_deg: float = 3.0) -> None:
        self.intent_mm = intent_mm
        self.step = step_deg
        self.max_extra = max_extra_deg
        self.pressure_on = pressure_on
        self.stall_deg = stall_deg
        self.extra = np.zeros(4)          # index, middle, ring, pinky
        self.contact = np.zeros(4, bool)

    def update(self, thumb_finger_mm: Sequence[float], commanded: Sequence[float],
               measured: Optional[Sequence[float]], thumb_pressure: float) -> np.ndarray:
        # Only the finger nearest the thumb can be the pinch target: MediaPipe's
        # fingertip depth is coarse, so a plain distance threshold fires on
        # neighbours too.
        nearest = int(np.argmin(thumb_finger_mm))
        for i in range(4):
            intent = i == nearest and thumb_finger_mm[i] < self.intent_mm
            j = 2 + i
            stalled = measured is not None and (commanded[j] - measured[j]) > self.stall_deg \
                and self.extra[i] > 0
            felt = thumb_pressure >= self.pressure_on and self.extra[i] > 0
            if not intent:
                self.extra[i] = max(0.0, self.extra[i] - 2 * self.step)
                self.contact[i] = False
            elif felt:
                self.contact[i] = True          # hold what we have
            elif stalled:
                # Blocked but the thumb tip feels nothing: the finger is jammed
                # against the thumb's side. Ease off instead of grinding.
                self.contact[i] = True
                self.extra[i] = max(0.0, self.extra[i] - self.step)
            else:
                self.contact[i] = False
                self.extra[i] = min(self.max_extra, self.extra[i] + self.step)
        return self.extra.copy()

    def reset(self) -> None:
        self.extra[:] = 0
        self.contact[:] = False
