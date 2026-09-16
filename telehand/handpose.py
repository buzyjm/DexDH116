"""Wrist-local joint frames reconstructed from MediaPipe world landmarks.

MediaPipe gives 21 3-D points and nothing else: no joint rotations. This module
builds a local coordinate frame at the palm and at every joint of the chosen
fingers, purely from the point geometry, and expresses all of it in the wrist
frame so global hand translation and rotation drop out.

Right hand only. The geometry below never sees chirality: :class:`Canonicalizer`
is the input adapter that turns whatever MediaPipe emits into canonical
right-hand landmarks, and it is the only place a reflection exists. Every
rotation built here has det(R) = +1.

Conventions (full derivation in FRAMES.md):

* palm frame  +x out of the palm, +y toward the thumb/index side, +z distal.
  The palm normal's sign is fixed by  dot(x, y0 x z0) > 0  with
  y0 = p5 - p17 and z0 = mean(p5, p9, p13) - p0.
* chain frames  built by minimal-rotation propagation: the parent frame is
  rotated the shortest way so its z axis lies along the child bone. No fixed
  reference vector, so no near-parallel degeneracy; the only singularity is
  antiparallel consecutive bones (a 180 deg fold), guarded numerically.
* wrist-local  q_i = R_W^T (p_i - p_0),  R_i^local = R_W^T R_i.

What is deliberately *not* here: any claim that a component of a relative
rotation is "flexion" or "abduction". ``relative_rotation`` and
``relative_rotvec`` are exposed; what they mean for a given joint is to be
established empirically, not assumed.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from .tracker import _chain_bend

WRIST = 0
INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP = 5, 9, 13, 17
PALM_POINTS = (WRIST, INDEX_MCP, MIDDLE_MCP, RING_MCP, PINKY_MCP)

# MediaPipe landmark chains, proximal to distal.
CHAINS: Dict[str, Tuple[int, int, int, int]] = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}
_JOINT_SUFFIX = {
    "thumb": ("cmc", "mcp", "ip", "tip"),
    "index": ("mcp", "pip", "dip", "tip"),
    "middle": ("mcp", "pip", "dip", "tip"),
    "ring": ("mcp", "pip", "dip", "tip"),
    "pinky": ("mcp", "pip", "dip", "tip"),
}
DEFAULT_FINGERS: Tuple[str, ...] = ("thumb", "index")

SIGN_MARGIN_MIN = 0.2      # below this the geometric sign test is not trusted
ANTIPARALLEL_EPS = 1e-6    # 1 + a.b below this counts as a 180 deg fold
MIN_BONE_M = 1e-4          # a bone shorter than 0.1 mm has no direction
PLANARITY_MAX = 0.7        # sigma3/sigma2 above this: the five palm points are not a plane

# Chirality vote: a finger curled past CURL_MIN_DEG says which side of the
# palm plane its tip is on; the vote is confident once the summed excess curl
# reaches VOTE_CONF. Measured on fk_session.npz: confident votes are +-230,
# and 533 confident frames disagreed with MediaPipe's label 0 times.
CURL_MIN_DEG = 60.0
VOTE_CONF = 20.0


def joint_names(fingers: Sequence[str] = DEFAULT_FINGERS) -> List[str]:
    out = ["palm"]
    for f in fingers:
        out += [f"{f}_{s}" for s in _JOINT_SUFFIX[f]]
    return out


# ------------------------------------------------------------------ helpers

def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    return v / n if n > 0 else v


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def minimal_rotation(a: np.ndarray, b: np.ndarray,
                     fallback_axis: Optional[np.ndarray] = None) -> Tuple[np.ndarray, bool]:
    """Rotation taking unit vector a onto unit vector b by the shortest path.

    With v = a x b and c = a . b:   R = I + [v]x + [v]x^2 / (1 + c)
    which is exact for all c > -1 and is the identity when a == b.

    Returns (R, degenerate). ``degenerate`` is True only when a and b are
    antiparallel, where the shortest path is undefined; then R is the half
    turn about ``fallback_axis`` (which must be perpendicular to a), or about
    an arbitrary perpendicular if none is given. Consecutive bones of a finger
    cannot be antiparallel, so this is a numerical guard, not a code path the
    hand is expected to exercise.
    """
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if 1.0 + c < ANTIPARALLEL_EPS:
        if fallback_axis is None:
            helper = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            fallback_axis = unit(np.cross(a, helper))
        k = unit(fallback_axis)
        return 2.0 * np.outer(k, k) - np.eye(3), True
    K = skew(v)
    return np.eye(3) + K + (K @ K) / (1.0 + c), False


# ---------------------------------------------------------- input adapter

def mirror_x(pts: np.ndarray) -> np.ndarray:
    """The one reflection in the pipeline. Which axis is mirrored is immaterial
    for anything wrist-local (verified: mirror-x/y/z give identical frames)."""
    out = np.asarray(pts, dtype=np.float64).copy()
    out[:, 0] *= -1.0
    return out


def chirality_vote(pts: np.ndarray) -> float:
    """Label-free chirality of a landmark set: > 0 right-hand geometry, < 0 left.

    A curled fingertip lies on the palmar side. Under reflection the cross
    product y0 x z0 flips relative to the mirrored tip displacement, so
    sign(dot(tip - mcp, y0 x z0)) reads the geometry's own chirality. Each of
    the four fingers votes with its curl beyond CURL_MIN_DEG; an open hand
    returns 0 (no evidence). The thumb is excluded: its curl is not planar.
    """
    y0 = pts[INDEX_MCP] - pts[PINKY_MCP]
    z0 = pts[[INDEX_MCP, MIDDLE_MCP, RING_MCP]].mean(axis=0) - pts[WRIST]
    n = np.cross(y0, z0)
    nn = float(np.linalg.norm(n))
    if nn == 0.0:
        return 0.0
    n /= nn
    v = 0.0
    for finger in ("index", "middle", "ring", "pinky"):
        a, b, c, t = CHAINS[finger]
        bend = _chain_bend(pts, (a, b, c, t))
        if bend > CURL_MIN_DEG:
            v += float(np.sign(np.dot(pts[t] - pts[a], n))) * (bend - CURL_MIN_DEG)
    return v


class Canonicalizer:
    """MediaPipe world landmarks -> canonical right-hand world landmarks.

    MediaPipe's handedness label is unreliable per frame in both directions:
    on fk_session.npz frame 0 is labelled Right with left geometry (a pure
    label flicker), while frames 728-729 are labelled Right with geometry that
    really did flip. So the label is never applied. Instead:

    1. geometry decides when a finger is curled (``chirality_vote``), and it
       decides immediately -- a genuine flip is caught in the frame it happens;
    2. with no curl (open hand) the previous decision carries over;
    3. before any evidence at all, the default comes from the webcam setup: a
       right hand through a mirrored webcam arrives as left geometry.

    The label is only counted against the decision, for the report.
    """

    def __init__(self, assume_mirrored: bool = True) -> None:
        self.default = -1 if assume_mirrored else +1
        self.chirality: Optional[int] = None      # +1 right, -1 left
        self.established = False                  # geometry has decided at least once
        self.counts = {"geometry": 0, "continuity": 0, "default": 0,
                       "switches": 0, "label_disagreements": 0}
        self.last_vote = 0.0
        self.last_source = ""

    def __call__(self, world_pts: np.ndarray, handedness: Optional[str] = None) -> np.ndarray:
        pts = np.asarray(world_pts, dtype=np.float64)
        v = chirality_vote(pts)
        self.last_vote = v
        if abs(v) >= VOTE_CONF:
            geom = 1 if v > 0 else -1
            if self.established and geom != self.chirality:
                self.counts["switches"] += 1
            self.chirality, self.established = geom, True
            source = "geometry"
        elif self.chirality is not None:
            source = "continuity"
        else:
            self.chirality = self.default
            source = "default"
        self.counts[source] += 1
        self.last_source = source
        if handedness is not None and str(handedness) not in ("?", ""):
            label = -1 if str(handedness).lower().startswith("l") else 1
            if label != self.chirality:
                self.counts["label_disagreements"] += 1
        return mirror_x(pts) if self.chirality < 0 else pts.copy()


# --------------------------------------------------------------- palm frame

@dataclass
class PalmFrame:
    origin: np.ndarray          # wrist, world coordinates
    R: np.ndarray               # columns x, y, z in world coordinates
    planarity: float            # sigma3 / sigma2 of the palm-point fit; 0 = perfect plane
    sign_margin: float          # |n . (y0 x z0)| / |y0 x z0|; 1 = unambiguous
    used_continuity: bool       # sign taken from the previous frame's normal


def palm_frame(pts: np.ndarray, prev_normal: Optional[np.ndarray] = None) -> PalmFrame:
    """Palm frame per FRAMES.md section 2.

    The normal is the least-squares plane through wrist and the four finger
    MCPs. It is the one sign-ambiguous quantity in the whole construction; its
    sign is fixed geometrically by dot(n, y0 x z0) > 0, and only when that test
    has no margin (never observed on real data) does the previous frame's
    normal decide, so an occlusion cannot lock in a wrong sign.
    """
    P = pts[list(PALM_POINTS)]
    c = P.mean(axis=0)
    _, s, vt = np.linalg.svd(P - c)
    n = vt[2]
    planarity = float(s[2] / s[1]) if s[1] > 0 else 1.0

    y0 = pts[INDEX_MCP] - pts[PINKY_MCP]
    z0 = pts[[INDEX_MCP, MIDDLE_MCP, RING_MCP]].mean(axis=0) - pts[WRIST]
    cr = np.cross(y0, z0)
    crn = float(np.linalg.norm(cr))
    margin = abs(float(np.dot(n, cr))) / crn if crn > 0 else 0.0

    used_continuity = False
    if crn > 0 and margin >= SIGN_MARGIN_MIN:
        if np.dot(n, cr) < 0:
            n = -n
    elif prev_normal is not None:
        used_continuity = True
        if np.dot(n, prev_normal) < 0:
            n = -n
    elif crn > 0 and np.dot(n, cr) < 0:
        n = -n

    x = unit(n)
    z = unit(z0 - np.dot(z0, x) * x)
    y = np.cross(z, x)           # then x cross y == z: right-handed
    R = np.stack([x, y, z], axis=1)
    return PalmFrame(pts[WRIST].copy(), R, planarity, margin, used_continuity)


# ---------------------------------------------------------------- the pose

@dataclass
class JointPose:
    name: str
    landmark: int
    position: np.ndarray        # (3,) wrist-local, metres
    rotation: np.ndarray        # (3, 3) wrist-local, det = +1
    quaternion: np.ndarray      # (4,) x, y, z, w  (scipy convention)
    parent: Optional[str]
    confidence: float = 1.0
    degenerate: bool = False    # a zero-length bone or antiparallel fold was guarded


@dataclass
class HandPose3D:
    timestamp_ms: int
    wrist_origin: np.ndarray            # (3,) world
    wrist_rotation: np.ndarray          # (3, 3) columns are the wrist axes in world
    points: np.ndarray                  # (21, 3) all landmarks, wrist-local
    joints: Dict[str, JointPose]
    fingers: Tuple[str, ...]
    planarity: float
    sign_margin: float
    used_continuity: bool
    handedness_score: float = float("nan")
    bone_lengths: Dict[Tuple[int, int], float] = field(default_factory=dict)

    # ------------------------------------------------------------ build

    @classmethod
    def from_landmarks(cls, pts: np.ndarray, timestamp_ms: int = 0,
                       fingers: Sequence[str] = DEFAULT_FINGERS,
                       prev_normal: Optional[np.ndarray] = None,
                       bone_ref: Optional[Dict[Tuple[int, int], Tuple[float, float]]] = None,
                       handedness_score: float = float("nan")) -> "HandPose3D":
        """Build every frame from 21 canonical right-hand world landmarks.

        Pure function of the current points (plus the previous palm normal,
        used only as a sign tie-break): no history enters a frame.
        """
        pts = np.asarray(pts, dtype=np.float64)
        if pts.shape != (21, 3):
            raise ValueError(f"expected (21, 3) landmarks, got {pts.shape}")
        pf = palm_frame(pts, prev_normal)
        R_W, o = pf.R, pf.origin

        world: List[Tuple[str, int, np.ndarray, np.ndarray, Optional[str], bool]] = [
            ("palm", WRIST, o, R_W, None, False)]
        lengths: Dict[Tuple[int, int], float] = {}
        for finger in fingers:
            idx = CHAINS[finger]
            suf = _JOINT_SUFFIX[finger]
            R_prev, z_prev, parent = R_W, R_W[:, 2], "palm"
            for k in range(3):
                a, b_i = idx[k], idx[k + 1]
                bone = pts[b_i] - pts[a]
                L = float(np.linalg.norm(bone))
                lengths[(a, b_i)] = L
                degenerate = False
                if L < MIN_BONE_M:
                    b = z_prev            # no direction: keep the parent's
                    degenerate = True
                else:
                    b = bone / L
                Rm, anti = minimal_rotation(z_prev, b, fallback_axis=R_prev[:, 0])
                degenerate = degenerate or anti
                R_k = Rm @ R_prev
                name = f"{finger}_{suf[k]}"
                world.append((name, a, pts[a], R_k, parent, degenerate))
                parent, R_prev, z_prev = name, R_k, b
            # The tip has no child bone and so no rotation of its own; it
            # carries the last bone's frame for convenience.
            world.append((f"{finger}_{suf[3]}", idx[3], pts[idx[3]], R_prev, parent, False))

        RT = R_W.T
        joints: Dict[str, JointPose] = {}
        for name, lm, pos, R, parent, degenerate in world:
            q = RT @ (pos - o)
            Rl = RT @ R
            quat = Rotation.from_matrix(Rl).as_quat()
            joints[name] = JointPose(name, lm, q, Rl, quat, parent, 1.0, degenerate)

        points_local = (pts - o) @ R_W     # row-wise R_W^T (p - o)

        pose = cls(int(timestamp_ms), o.copy(), R_W, points_local, joints, tuple(fingers),
                   pf.planarity, pf.sign_margin, pf.used_continuity, handedness_score, lengths)
        pose._assign_confidence(bone_ref)
        return pose

    def _assign_confidence(self, bone_ref) -> None:
        """Derived confidence: MediaPipe reports none per landmark.

        Palm: the sign margin, zeroed if the palm points do not form a plane.
        Joints: each bone is rigid, so a length far outside its own recent
        spread is direct evidence the landmarks moved somewhere they could
        not. ``bone_ref`` maps a bone to (median, robust scale); a deviation
        under 3 scales keeps full confidence, 6 scales is zero. A joint takes
        the worst of its adjacent bones.
        """
        palm = self.joints["palm"]
        palm.confidence = 0.0 if self.planarity > PLANARITY_MAX else float(min(1.0, self.sign_margin))
        if not bone_ref:
            return
        for finger in self.fingers:
            idx = CHAINS[finger]
            suf = _JOINT_SUFFIX[finger]
            bones = [(idx[k], idx[k + 1]) for k in range(3)]

            def conf(bone):
                ref = bone_ref.get(bone)
                if not ref:
                    return 1.0
                med, scale = ref
                dev = abs(self.bone_lengths[bone] - med) / scale
                return float(np.clip(2.0 - dev / 3.0, 0.0, 1.0))

            for k in range(4):
                adjacent = [b for b in bones if idx[k] in b]
                j = self.joints[f"{finger}_{suf[k]}"]
                j.confidence = min(conf(b) for b in adjacent)
                if j.degenerate:
                    j.confidence = 0.0

    # --------------------------------------------------------- queries

    @property
    def palm_normal_world(self) -> np.ndarray:
        return self.wrist_rotation[:, 0]

    def relative_rotation(self, name: str) -> np.ndarray:
        """R_parent^T R_child, i.e. the child's frame expressed in its parent's."""
        j = self.joints[name]
        if j.parent is None:
            return np.eye(3)
        return self.joints[j.parent].rotation.T @ j.rotation

    def relative_rotvec(self, name: str, degrees: bool = True) -> np.ndarray:
        """Axis-angle of the relative rotation, in the parent's local frame.

        No anatomical meaning is attached to its components here.
        """
        rv = Rotation.from_matrix(self.relative_rotation(name)).as_rotvec()
        return np.degrees(rv) if degrees else rv

    def bend_angle_deg(self, name: str) -> float:
        """Geodesic angle of the relative rotation.

        For a chain joint this is exactly the angle between the two bones that
        meet there (minimal rotation adds no twist), so it must reproduce
        ``tracker._angle_between`` on the same points.
        """
        return float(np.linalg.norm(self.relative_rotvec(name, degrees=True)))

    def chain_bend_deg(self, finger: str) -> float:
        """Sum of the two interior joint bends: ``tracker._chain_bend`` for this finger."""
        suf = _JOINT_SUFFIX[finger]
        return self.bend_angle_deg(f"{finger}_{suf[1]}") + self.bend_angle_deg(f"{finger}_{suf[2]}")


class HandPoseTracker:
    """Frame-to-frame state that :class:`HandPose3D` deliberately does not hold.

    Only two things: the previous palm normal (a sign tie-break, used solely
    when the geometric test has no margin) and running bone-length statistics
    for the derived confidence.
    """

    def __init__(self, fingers: Sequence[str] = DEFAULT_FINGERS, history: int = 90) -> None:
        self.fingers = tuple(fingers)
        self.prev_normal: Optional[np.ndarray] = None
        self._hist: Dict[Tuple[int, int], Deque[float]] = {}
        self._history = history
        self.continuity_uses = 0

    def _bone_ref(self) -> Dict[Tuple[int, int], Tuple[float, float]]:
        ref = {}
        for b, h in self._hist.items():
            if len(h) < 5:
                continue
            arr = np.asarray(h)
            med = float(np.median(arr))
            mad = float(np.median(np.abs(arr - med)))
            # Floor at 8 % of the length: MediaPipe's bone lengths shift ~9 %
            # with pose on this rig (measured), and that is not an error to
            # act on. Below the floor a fist would read as a 6-sigma outlier
            # against an open-hand history. The gate is for gross faults --
            # a doubled or collapsed bone -- not pose-dependent shrinkage.
            scale = max(1.4826 * mad, 0.08 * med)
            ref[b] = (med, scale)
        return ref

    def update(self, pts: np.ndarray, timestamp_ms: int = 0,
               handedness_score: float = float("nan")) -> HandPose3D:
        pose = HandPose3D.from_landmarks(pts, timestamp_ms, self.fingers,
                                         prev_normal=self.prev_normal, bone_ref=self._bone_ref(),
                                         handedness_score=handedness_score)
        self.prev_normal = pose.palm_normal_world.copy()
        if pose.used_continuity:
            self.continuity_uses += 1
        for b, L in pose.bone_lengths.items():
            self._hist.setdefault(b, deque(maxlen=self._history)).append(L)
        return pose

    def reset(self) -> None:
        self.prev_normal = None
        self._hist.clear()
