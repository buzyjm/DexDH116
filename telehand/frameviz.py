"""Replay visualisation and stability metrics for :mod:`handpose` frames.

Two things live here:

* ``render_replay`` draws the wrist-local skeleton with an XYZ triad at every
  reconstructed joint (x red, y green, z blue), orbitable with the same mouse
  and keys as the teleop hand view.
* ``FrameStats`` accumulates every check the representation has to pass
  before a camera is involved: det(R) = +1, orthonormality, no 180 deg flips,
  bend angles reproducing ``tracker._angle_between``, palm-normal sign
  stability, frame-to-frame geodesic jitter, wrist-locality, and invariance
  to a global rigid transform of the input.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from . import handpose as hp
from .handview import view_matrix
from .tracker import _angle_between, extract_signals, SIGNAL_NAMES
from .viz import ViewControl, VIEW_X, VIEW_Y, VIEW_W, VIEW_H, BG, FG, MUTED

AXIS_BGR = ((0, 0, 255), (0, 255, 0), (255, 0, 0))     # x red, y green, z blue
IMPLAUSIBLE_MM = 30.0     # wrist-local landmark motion per frame that no hand produces
BONE = (170, 170, 170)
CONTEXT = (90, 88, 86)
JOINT = (240, 240, 240)
PANEL_TEXT_W = 460

# Screen basis: screen-x = wrist +y (thumb side to the right for a right hand
# facing the viewer), screen-y = wrist +z (fingers up), depth = wrist +x.
SCREEN_BASIS = np.array([[0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0],
                         [1.0, 0.0, 0.0]])
CENTRE_LOCAL = np.array([0.0, 0.0, 0.08])     # roughly the palm centre, wrist-local
DEFAULT_VIEW = (0.0, 0.0)

# Skeleton context for landmarks that have no frame yet.
CONTEXT_BONES = [(0, 5), (5, 9), (9, 13), (13, 17), (17, 0), (0, 1),
                 (9, 10), (10, 11), (11, 12), (13, 14), (14, 15), (15, 16),
                 (17, 18), (18, 19), (19, 20)]


class FrameView(ViewControl):
    """The teleop orbit control, anchored at the panel's own rectangle."""

    def __init__(self) -> None:
        super().__init__(camera_width=0)
        self.yaw, self.pitch = DEFAULT_VIEW

    def _defaults(self):
        return DEFAULT_VIEW

    def handle_key(self, key: int) -> bool:
        if key == ord("v"):         # do not overwrite teleop's saved viewpoint
            return True
        return super().handle_key(key)


def _project(q: np.ndarray, yaw: float, pitch: float, zoom: float,
             size: Tuple[int, int]) -> np.ndarray:
    V = view_matrix(yaw, pitch) @ SCREEN_BASIS
    s = (np.atleast_2d(q) - CENTRE_LOCAL) @ V.T
    scale = zoom * size[1] / 0.30
    u = size[0] / 2.0 + s[:, 0] * scale
    v = size[1] / 2.0 - s[:, 1] * scale
    return np.stack([u, v], axis=1)


AXIS_THICKNESS = 4        # shaft thickness in px for joint axes; the palm gets +2


def render_pose(pose: hp.HandPose3D, size: Tuple[int, int] = (VIEW_W, VIEW_H), *,
                yaw: float = 0.0, pitch: float = 0.0, zoom: float = 1.0,
                axis_len: float = 0.015, axis_thickness: int = AXIS_THICKNESS) -> np.ndarray:
    """Draw the wrist-local skeleton with an XYZ arrow triad at every joint.

    ``axis_len`` is the arrow length in metres (unchanged by thickness);
    ``axis_thickness`` is the shaft width in pixels.
    """
    img = np.full((size[1], size[0], 3), BG, np.uint8)
    yaw = DEFAULT_VIEW[0] if yaw is None else yaw
    pitch = DEFAULT_VIEW[1] if pitch is None else pitch
    P2 = _project(pose.points, yaw, pitch, zoom, size)

    def pt(p):
        return int(round(p[0])), int(round(p[1]))

    for a, b in CONTEXT_BONES:
        cv2.line(img, pt(P2[a]), pt(P2[b]), CONTEXT, 1, cv2.LINE_AA)
    for name, j in pose.joints.items():
        if j.parent is not None:
            p = pose.joints[j.parent]
            cv2.line(img, pt(P2[p.landmark]), pt(P2[j.landmark]), BONE, 2, cv2.LINE_AA)
    for name, j in pose.joints.items():
        L = axis_len * (2.0 if name == "palm" else 1.0)
        base = j.position
        ends = _project(np.stack([base + j.rotation[:, i] * L for i in range(3)]),
                        yaw, pitch, zoom, size)
        o = pt(P2[j.landmark])
        thick = axis_thickness + (2 if name == "palm" else 0)
        for i in range(3):
            cv2.arrowedLine(img, o, pt(ends[i]), AXIS_BGR[i], thick, cv2.LINE_AA, tipLength=0.3)
        r = 4 if name == "palm" else 3
        col = JOINT if j.confidence > 0.5 else (60, 60, 230)
        cv2.circle(img, o, r, col, -1, cv2.LINE_AA)
    return img


def render_replay(pose: hp.HandPose3D, lines: Sequence[str], *, yaw, pitch, zoom,
                  axis_thickness: int = AXIS_THICKNESS) -> np.ndarray:
    """The 3-D view at ViewControl's rectangle, a text column beside it."""
    W = VIEW_X + VIEW_W + PANEL_TEXT_W
    H = VIEW_Y + VIEW_H + 8
    canvas = np.full((H, W, 3), BG, np.uint8)
    canvas[VIEW_Y:VIEW_Y + VIEW_H, VIEW_X:VIEW_X + VIEW_W] = render_pose(
        pose, yaw=yaw, pitch=pitch, zoom=zoom, axis_thickness=axis_thickness)
    cv2.putText(canvas, "wrist-local frames   x=red  y=green  z=blue   drag/wheel to orbit",
                (VIEW_X, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, FG, 1, cv2.LINE_AA)
    x0 = VIEW_X + VIEW_W + 16
    for k, line in enumerate(lines):
        cv2.putText(canvas, line, (x0, VIEW_Y + 16 + 18 * k), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, FG if not line.startswith(" ") else MUTED, 1, cv2.LINE_AA)
    return canvas


# ---------------------------------------------------------------- metrics

def geodesic_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def _pct(v, q):
    return float(np.percentile(v, q)) if len(v) else float("nan")


def _residual(x: np.ndarray, k: int = 5) -> np.ndarray:
    """High-frequency residual against a k-frame moving average (noise, not motion)."""
    if len(x) < 2 * k:
        return np.zeros(0)
    sm = np.convolve(x, np.ones(k) / k, mode="same")
    return (x - sm)[k:-k]


class FrameStats:
    def __init__(self, fingers: Sequence[str] = hp.DEFAULT_FINGERS, rng_seed: int = 0) -> None:
        self.fingers = tuple(fingers)
        self.names = hp.joint_names(fingers)
        self.R: Dict[str, List[np.ndarray]] = {n: [] for n in self.names}
        # The wrist-local palm frame is the identity by definition, so the
        # palm's own motion has to be judged in world coordinates.
        self.palm_world: List[np.ndarray] = []
        self.pos: Dict[str, List[np.ndarray]] = {n: [] for n in self.names}
        self.normals: List[np.ndarray] = []
        self.margins: List[float] = []
        self.planarity: List[float] = []
        self.continuity: List[bool] = []
        self.signals: List[List[float]] = []
        self.bend_err: Dict[str, float] = {}
        self.chain_err: Dict[str, float] = {}
        self.rotvec: Dict[str, List[np.ndarray]] = {}
        self.axis_dir: Dict[str, List[Optional[np.ndarray]]] = {}
        self.curl_check: List[Tuple[float, float]] = []   # (index chain bend, tip-mcp dot x)
        self.invariance: List[Tuple[float, float]] = []   # (max |dpos| m, max ||dR||_F)
        # Wrist-local landmark motion per frame, to tell a flip the construction
        # produced from one the input produced. Fingertip speed measured earlier
        # on this rig: p99 465 mm/s = 15 mm/frame; 30 mm/frame is not a hand.
        self.max_disp_mm: List[float] = [0.0]
        self._prev_points: Optional[np.ndarray] = None
        self.rng = np.random.default_rng(rng_seed)
        self.frames = 0

    def add(self, pose: hp.HandPose3D, pts: np.ndarray, invariance_test: bool = False) -> None:
        self.frames += 1
        for n in self.names:
            j = pose.joints[n]
            self.R[n].append(j.rotation)
            self.pos[n].append(j.position)
        self.normals.append(pose.palm_normal_world)
        self.palm_world.append(pose.wrist_rotation)
        if self._prev_points is not None:
            self.max_disp_mm.append(float(np.linalg.norm(pose.points - self._prev_points, axis=1).max() * 1000))
        self._prev_points = pose.points
        self.margins.append(pose.sign_margin)
        self.planarity.append(pose.planarity)
        self.continuity.append(pose.used_continuity)
        self.signals.append(extract_signals(pts))

        # Bend angles must reproduce the existing angle function exactly.
        for finger in self.fingers:
            idx = hp.CHAINS[finger]
            suf = hp._JOINT_SUFFIX[finger]
            for k in (1, 2):
                name = f"{finger}_{suf[k]}"
                ref = _angle_between(pts[idx[k]] - pts[idx[k - 1]], pts[idx[k + 1]] - pts[idx[k]])
                err = abs(pose.bend_angle_deg(name) - ref)
                self.bend_err[name] = max(self.bend_err.get(name, 0.0), err)
            ref_chain = _angle_between(pts[idx[1]] - pts[idx[0]], pts[idx[2]] - pts[idx[1]]) + \
                _angle_between(pts[idx[2]] - pts[idx[1]], pts[idx[3]] - pts[idx[2]])
            self.chain_err[finger] = max(self.chain_err.get(finger, 0.0),
                                         abs(pose.chain_bend_deg(finger) - ref_chain))
            # Relative rotation of every non-tip joint, and the bend axis
            # direction where the bend is large enough for it to be defined.
            for k in range(3):
                name = f"{finger}_{suf[k]}"
                rv = pose.relative_rotvec(name)
                self.rotvec.setdefault(name, []).append(rv)
                ang = float(np.linalg.norm(rv))
                self.axis_dir.setdefault(name, []).append(rv / ang if ang > 10.0 else None)

        if "index" in self.fingers:
            q = pose.points
            self.curl_check.append((pose.chain_bend_deg("index"),
                                    float(np.dot(q[8] - q[5], np.array([1.0, 0.0, 0.0])))))

        if invariance_test:
            Rg = hp.Rotation.random(random_state=self.rng.integers(1 << 30)).as_matrix()
            tg = self.rng.normal(0, 0.5, 3)
            moved = pts @ Rg.T + tg
            other = hp.HandPose3D.from_landmarks(moved, pose.timestamp_ms, self.fingers)
            dp = max(float(np.linalg.norm(other.joints[n].position - pose.joints[n].position))
                     for n in self.names)
            # Frobenius, not geodesic: arccos near 1 turns 1e-15 of matrix
            # error into 1e-6 deg and would fail a test that has passed.
            dr = max(float(np.linalg.norm(other.joints[n].rotation - pose.joints[n].rotation))
                     for n in self.names)
            self.invariance.append((dp, dr))

    # ------------------------------------------------------------ report

    def report(self) -> Dict[str, object]:
        out: Dict[str, object] = {"frames": self.frames}
        dets, orth = [], []
        jitter: Dict[str, Dict[str, float]] = {}
        flips: Dict[str, int] = {}
        glitch_frames: Dict[int, list] = {}
        for n in self.names:
            Rs = self.palm_world if n == "palm" else self.R[n]
            dets += [abs(float(np.linalg.det(R)) - 1.0) for R in Rs]
            orth += [float(np.linalg.norm(R.T @ R - np.eye(3))) for R in Rs]
            g = np.array([geodesic_deg(Rs[i - 1], Rs[i]) for i in range(1, len(Rs))])
            jitter[n] = {"p50": _pct(g, 50), "p95": _pct(g, 95), "max": float(g.max()) if len(g) else float("nan")}
            disp = np.array(self.max_disp_mm[1:len(Rs)])
            big = g > 90.0
            flips[n] = int((big & (disp < IMPLAUSIBLE_MM)).sum())
            for i in np.where(big & (disp >= IMPLAUSIBLE_MM))[0]:
                glitch_frames.setdefault(int(i + 1), []).append((n, float(g[i]), float(disp[i])))
        out["det_max_dev"] = max(dets) if dets else float("nan")
        out["orth_max_dev"] = max(orth) if orth else float("nan")
        out["jitter"] = jitter
        out["flips"] = flips
        out["glitch_flips"] = {k: v for k, v in sorted(glitch_frames.items())}

        N = np.array(self.normals)
        dots = np.einsum("ij,ij->i", N[1:], N[:-1]) if len(N) > 1 else np.zeros(0)
        out["normal_min_dot"] = float(dots.min()) if len(dots) else float("nan")
        out["normal_sign_flips"] = int((dots < 0).sum())
        out["sign_margin_min"] = float(min(self.margins)) if self.margins else float("nan")
        out["planarity_max"] = float(max(self.planarity)) if self.planarity else float("nan")
        out["continuity_uses"] = int(sum(self.continuity))

        out["bend_err_max"] = dict(self.bend_err)
        out["chain_err_max"] = dict(self.chain_err)

        palm_pos = np.array(self.pos["palm"])
        out["wrist_local_pos_max"] = float(np.abs(palm_pos).max()) if len(palm_pos) else float("nan")
        out["wrist_local_rot_max"] = float(max(np.linalg.norm(R - np.eye(3)) for R in self.R["palm"])) \
            if self.R["palm"] else float("nan")

        if self.invariance:
            inv = np.array(self.invariance)
            out["invariance"] = {"tests": len(inv), "pos_max_m": float(inv[:, 0].max()),
                                 "rot_max_frob": float(inv[:, 1].max())}

        S = np.array(self.signals)
        out["signal_residual"] = {SIGNAL_NAMES[i]: float(np.sqrt(np.mean(_residual(S[:, i]) ** 2)))
                                 for i in range(S.shape[1])} if len(S) >= 10 else {}
        rv_res: Dict[str, float] = {}
        for n, rvs in self.rotvec.items():
            A = np.array(rvs)
            rv_res[n] = float(np.sqrt(np.mean(np.stack([_residual(A[:, i]) for i in range(3)]) ** 2)))
        out["rotvec_residual"] = rv_res
        axis_j: Dict[str, Dict[str, float]] = {}
        for n, dirs in self.axis_dir.items():
            angs = [float(np.degrees(np.arccos(np.clip(np.dot(a, b), -1, 1))))
                    for a, b in zip(dirs[:-1], dirs[1:]) if a is not None and b is not None]
            axis_j[n] = {"frames": len(angs), "p50": _pct(angs, 50), "p95": _pct(angs, 95)}
        out["axis_dir_jitter"] = axis_j

        if self.curl_check:
            C = np.array(self.curl_check)
            curled = C[C[:, 0] > 90.0]
            out["curl_check"] = {"curled_frames": int(len(curled)),
                                 "palmar_fraction": float((curled[:, 1] > 0).mean()) if len(curled) else float("nan")}
        return out


def format_report(r: Dict[str, object]) -> str:
    L: List[str] = []
    ok = lambda cond: "PASS" if cond else "FAIL"
    L.append("")
    L.append(f"frame reconstruction over {r['frames']} frames")
    L.append(f"  det(R)          max |det-1|   {r['det_max_dev']:.2e}   {ok(r['det_max_dev'] < 1e-9)}")
    L.append(f"  R^T R           max ||.-I||   {r['orth_max_dev']:.2e}   {ok(r['orth_max_dev'] < 1e-9)}")
    L.append(f"  wrist-local     |palm pos| max {r['wrist_local_pos_max']:.1e}, ||R_palm-I|| max "
             f"{r['wrist_local_rot_max']:.1e}   {ok(r['wrist_local_pos_max'] < 1e-12 and r['wrist_local_rot_max'] < 1e-12)}")
    if "invariance" in r:
        i = r["invariance"]
        L.append(f"  rigid invariance  {i['tests']} random SE(3) transforms: max |dpos| {i['pos_max_m']:.2e} m, "
                 f"max ||dR||_F {i['rot_max_frob']:.2e}   {ok(i['pos_max_m'] < 1e-9 and i['rot_max_frob'] < 1e-9)}")
    L.append(f"  palm normal     min dot(n_t, n_t-1) {r['normal_min_dot']:+.4f}, sign flips {r['normal_sign_flips']}, "
             f"margin min {r['sign_margin_min']:.3f}, planarity max {r['planarity_max']:.3f}, "
             f"continuity fallback used {r['continuity_uses']}x   {ok(r['normal_sign_flips'] == 0)}")
    flips_total = sum(r["flips"].values())
    L.append(f"  180-deg flips   {flips_total} with plausible input (< {IMPLAUSIBLE_MM:.0f} mm/frame landmark motion)"
             f"   {ok(flips_total == 0)}")
    gf = r.get("glitch_flips", {})
    if gf:
        n_ev = sum(len(v) for v in gf.values())
        L.append(f"                  {n_ev} coincident with landmark jumps >= {IMPLAUSIBLE_MM:.0f} mm in the INPUT "
                 f"(not a frame-construction event):")
        for frame, ev in gf.items():
            L.append(f"                    frame {frame}: " + ", ".join(f"{n} {g:.0f} deg" for n, g, d in ev)
                     + f"   landmark moved {ev[0][2]:.0f} mm")
    L.append("")
    L.append("bend angle vs tracker._angle_between (max |diff| deg)")
    for n, e in r["bend_err_max"].items():
        L.append(f"  {n:<12} {e:.2e}   {ok(e < 1e-9)}")
    for f, e in r["chain_err_max"].items():
        L.append(f"  {f+' chain':<12} {e:.2e}   {ok(e < 1e-9)}   (== existing {f}_flexion signal)")
    L.append("")
    L.append("frame-to-frame geodesic jitter (deg, whole rotation; palm in world coords)   p50     p95     max")
    for n, j in r["jitter"].items():
        L.append(f"  {n:<12} {j['p50']:>8.2f}{j['p95']:>8.2f}{j['max']:>8.2f}")
    L.append("")
    L.append("high-frequency residual vs 5-frame average (RMS)")
    L.append("  existing signals (deg):")
    for n, v in r.get("signal_residual", {}).items():
        L.append(f"    {n:<18} {v:6.2f}")
    L.append("  relative-rotation axis-angle, all 3 components (deg):")
    for n, v in r.get("rotvec_residual", {}).items():
        L.append(f"    {n:<18} {v:6.2f}")
    L.append("  bend-axis direction between consecutive frames, where bend > 10 deg (deg)   frames   p50    p95")
    for n, a in r.get("axis_dir_jitter", {}).items():
        L.append(f"    {n:<18} {a['frames']:>5}  {a['p50']:6.2f} {a['p95']:6.2f}")
    if "curl_check" in r:
        c = r["curl_check"]
        L.append("")
        L.append(f"chirality self-check: index curled (>90 deg) in {c['curled_frames']} frames; tip on the palmar (+x) "
                 f"side in {100 * c['palmar_fraction']:.1f}% of them   {ok(c['palmar_fraction'] > 0.95)}")
    return "\n".join(L)
