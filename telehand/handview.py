"""Render the hand from its URDF, for the teleop and jog panels.

Two backends. MuJoCo draws the vendor MJCF with real meshes and lighting and
is both prettier and faster (0.15 ms a frame on the GPU); it needs a GL context,
which rules it out when running headless. The hull renderer below is the
fallback: pure numpy and cv2, every link reduced to a sampled point cloud and
drawn as the convex hull of its projection. `HandRenderer()` picks whichever
works.

The panel used to show a stylised hand drawn by hand: a rectangle for the palm
and chains of line segments that curled in the image plane. It was legible but
wrong in every proportion. Now that `kinematics` carries the vendor geometry
and a measured firmware-to-joint mapping, the same joint angles that go out
over RS485 can drive a picture of the actual hand.

The meshes total 358k triangles, far too many to transform per frame, so each
link is reduced to a sampled point cloud once and drawn as the convex hull of
its projection. Finger links are convex already; the palm is not, so it is cut
into slabs along its long axis and each slab hulled separately, which brings
back the slots the fingers sit in. Cost is about 3 ms a frame.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .kinematics import (URDF_ROOT, FINGER_CHAINS, HandModel, _axis_rotation,
                         _stl_vertices)
from .paths import CACHE_DIR

# Default viewpoint: three-quarters from the palm side. Of the six tried it is
# the only one showing palm, four fingers and thumb at once, which is what
# makes a thumbs-up or a closing pinch readable at a glance.
DEFAULT_YAW = -115.0
DEFAULT_PITCH = -35.0

MEASURED_MIN_DEG = 2.0    # below this the hand has arrived; drawing it is noise
PALM_SLABS = 5
SAMPLES_PER_LINK = 1500

BG = (26, 24, 22)
PALM_BASE = (38, 40, 44)
LINK_BASE = (70, 72, 76)
EDGE = (24, 24, 26)
TIP_DOT = (235, 235, 235)
MEASURED = (90, 200, 250)   # outline of the pose the hand actually reached


def _link_chains() -> Dict[str, List[str]]:
    """Every link, with the joint chain that positions it."""
    chains: Dict[str, List[str]] = {"base_link": []}
    for _finger, (chain, tip) in FINGER_CHAINS.items():
        for i, joint in enumerate(chain):
            chains[f"{joint}_link"] = list(chain[: i + 1])
        chains[f"{tip}_link"] = list(chain)      # the tip link rides the last joint
    return chains


def _slabs(points: np.ndarray, n: int) -> List[np.ndarray]:
    centre = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centre, full_matrices=False)
    t = (points - centre) @ vt[0]
    edges = np.linspace(t.min(), t.max(), n + 1)
    out = []
    for i in range(n):
        m = (t >= edges[i] - 1e-9) & (t <= edges[i + 1] + 1e-9)
        if m.sum() >= 3:
            out.append(points[m])
    return out


def lag(commanded, measured):
    """Per-joint shortfall in degrees, commanded minus measured."""
    return [float(c) - float(m) for c, m in zip(commanded, measured)]


def lagging(commanded, measured, threshold: float = MEASURED_MIN_DEG) -> bool:
    """True when any joint is far enough from its command to be worth drawing."""
    values = lag(commanded, measured)
    return bool(values) and max(abs(v) for v in values) >= threshold


def view_matrix(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    cy, sy = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    cp, sp = np.cos(np.radians(pitch_deg)), np.sin(np.radians(pitch_deg))
    return (np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
            @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]))


class HullHandRenderer:
    def __init__(self, direction: str = "R", samples: int = SAMPLES_PER_LINK) -> None:
        self.model = HandModel(direction)
        self.direction = direction
        self.chains = _link_chains()
        self.points = self._load_points(samples)
        # A fixed framing so the hand does not jump about as it moves: bound the
        # union of the open and fist poses once, and reuse it for every frame.
        self._frame: Optional[Tuple[np.ndarray, float, Tuple[int, int]]] = None

    # ------------------------------------------------------------- geometry

    def _load_points(self, samples: int) -> Dict[str, np.ndarray]:
        pkg = URDF_ROOT / f"DH116-{self.direction}000-A1"
        cache = CACHE_DIR / f"{self.direction}_{samples}.npz"
        if cache.exists():
            data = np.load(cache)
            return {k: data[k] for k in data.files}
        rng = np.random.default_rng(0)
        out = {}
        for link in self.chains:
            v = _stl_vertices(pkg / "meshes" / f"{link}.STL")
            if len(v) > samples:
                v = v[rng.choice(len(v), samples, replace=False)]
            out[link] = v.astype(np.float32)
        try:
            CACHE_DIR.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(cache, **out)
        except OSError:
            pass
        return out

    def _joint_angles(self, firmware: Sequence[float]) -> Dict[str, float]:
        per_finger = HandModel.joints_from_firmware(firmware)
        angles = {}
        for finger, (chain, _tip) in FINGER_CHAINS.items():
            for name, a in zip(chain, per_finger[finger]):
                angles[name] = a
        return angles

    def _link_pose(self, chain: Sequence[str], angles: Dict[str, float]):
        pos = np.zeros(3)
        rot = np.eye(3)
        for joint in chain:
            j = self.model.joints[joint]
            pos = pos + rot @ j.xyz
            rot = rot @ j.rotation @ _axis_rotation(j.axis, angles[joint])
        return pos, rot

    def world_parts(self, firmware: Sequence[float], view: np.ndarray):
        """Per-part point clouds in view space, plus the five fingertips."""
        angles = self._joint_angles(firmware)
        parts: List[Tuple[np.ndarray, bool]] = []
        for link, chain in self.chains.items():
            pos, rot = self._link_pose(chain, angles)
            pts = (self.points[link] @ rot.T + pos) @ view.T
            if link == "base_link" and PALM_SLABS > 1:
                parts += [(s, True) for s in _slabs(pts, PALM_SLABS)]
            else:
                parts.append((pts, False))
        per_finger = HandModel.joints_from_firmware(firmware)
        tips = {f: self.model.fingertip(f, per_finger[f]) @ view.T for f in per_finger}
        return parts, tips

    # -------------------------------------------------------------- drawing

    def _framing(self, view: np.ndarray, size: Tuple[int, int]):
        """Centre and scale that hold both the open hand and a fist."""
        if self._frame is not None and self._frame[2] == size:
            return self._frame[0], self._frame[1]
        pts = []
        for fw in ([0.0] * 6, [60.0, 30.0, 80.0, 80.0, 80.0, 80.0]):
            parts, _ = self.world_parts(fw, view)
            pts.append(np.vstack([p for p, _ in parts]))
        allp = np.vstack(pts)
        lo, hi = allp.min(axis=0), allp.max(axis=0)
        centre = (lo + hi) / 2
        scale = 0.90 * min(size[0] / (hi[0] - lo[0] + 1e-6),
                           size[1] / (hi[1] - lo[1] + 1e-6))
        self._frame = (centre, scale, size)
        return centre, scale

    def render(self, firmware: Sequence[float], size: Tuple[int, int] = (400, 300),
               yaw: float = DEFAULT_YAW, pitch: float = DEFAULT_PITCH,
               measured: Optional[Sequence[float]] = None,
               background=BG) -> np.ndarray:
        import cv2

        view = view_matrix(yaw, pitch)
        # Framing depends on the viewpoint, so drop it when the view moves.
        if self._frame is not None and getattr(self, "_frame_view", None) is not None \
                and not np.allclose(self._frame_view, view):
            self._frame = None
        self._frame_view = view

        centre, scale = self._framing(view, size)
        img = np.full((size[1], size[0], 3), background, np.uint8)

        def project(p: np.ndarray) -> np.ndarray:
            return np.c_[(p[:, 0] - centre[0]) * scale + size[0] / 2,
                         -(p[:, 1] - centre[1]) * scale + size[1] / 2].astype(np.float32)

        parts, tips = self.world_parts(firmware, view)
        parts.sort(key=lambda t: t[0][:, 2].mean())
        depths = [p[:, 2].mean() for p, _ in parts]
        z0, z1 = min(depths), max(depths)
        for pts, is_palm in parts:
            hull = cv2.convexHull(project(pts)).astype(np.int32)
            t = (pts[:, 2].mean() - z0) / max(z1 - z0, 1e-9)
            base = PALM_BASE if is_palm else LINK_BASE
            gain = 1.0 if is_palm else 1.35
            colour = tuple(int(min(255, b + gain * 95 * t)) for b in base)
            cv2.fillConvexPoly(img, hull, colour, cv2.LINE_AA)
            cv2.polylines(img, [hull], True, EDGE, 1, cv2.LINE_AA)

        if measured is not None and lagging(firmware, measured):
            # Where the hand actually is, outlined over the commanded pose, and
            # only when it is somewhere else -- see MujocoHandRenderer.render.
            mparts, mtips = self.world_parts(measured, view)
            for pts, is_palm in mparts:
                if is_palm:
                    continue
                hull = cv2.convexHull(project(pts)).astype(np.int32)
                cv2.polylines(img, [hull], True, MEASURED, 1, cv2.LINE_AA)
            for tip in mtips.values():
                xy = project(tip.reshape(1, 3))[0]
                cv2.circle(img, (int(xy[0]), int(xy[1])), 4, MEASURED, 1, cv2.LINE_AA)

        for name, tip in tips.items():
            xy = project(tip.reshape(1, 3))[0]
            cv2.circle(img, (int(xy[0]), int(xy[1])), 3, TIP_DOT, -1, cv2.LINE_AA)
        return img


# MuJoCo camera for the same three-quarter palm view the hull renderer uses.
MJ_AZIMUTH = -115.0
MJ_ELEVATION = -20.0
MJ_DISTANCE = 0.22
OVERSCAN = 1.6        # render this much larger, then crop to centre the hand
TARGET_FILL = 0.88    # fraction of the frame the hand should span
MEASURED_EDGE = (90, 200, 250)

# Geom holding each fingertip, in the MJCF. Its mesh is searched once for the
# vertex furthest from the geom origin, which is the pad that touches things.
TIP_GEOM = {"thumb": "finger14_link", "index": "finger23_link",
            "middle": "finger33_link", "ring": "finger43_link",
            "pinky": "finger53_link"}

# Which finger a MJCF geom belongs to, for naming contacts.
GEOM_FINGER = {"1": "thumb", "2": "index", "3": "middle", "4": "ring", "5": "pinky"}
CONTACT_MIN_MM = 0.5     # ignore grazes; only report real interpenetration
CONTACT_NEAR = (90, 210, 255)
CONTACT_DEEP = (60, 80, 255)


class MujocoHandRenderer:
    """Render the vendor MJCF with MuJoCo's own renderer."""

    def __init__(self, direction: str = "R") -> None:
        import os
        os.environ.setdefault("MUJOCO_GL", "glfw")
        import mujoco

        self._mj = mujoco
        xml = URDF_ROOT / f"DH116-{direction}000-A1" / "mjcf" / f"DH116-{direction}000-A1.xml"
        if not xml.exists():
            raise FileNotFoundError(f"MJCF not found: {xml}")
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        # The MJCF does not declare an offscreen buffer, and MuJoCo's default is
        # smaller than the oversized canvas the framing pass renders into.
        self.model.vis.global_.offwidth = max(int(self.model.vis.global_.offwidth), 1600)
        self.model.vis.global_.offheight = max(int(self.model.vis.global_.offheight), 1200)
        # The MJCF ships with collisions off. Turning them on costs nothing
        # measurable and lets the renderer show where the hand is pressing on
        # itself: the meshes are rigid, so a real contact reads as overlap.
        self.model.geom_contype[:] = 1
        self.model.geom_conaffinity[:] = 1
        self.data = mujoco.MjData(self.model)
        self.direction = direction
        self._qadr = {mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i):
                      self.model.jnt_qposadr[i] for i in range(self.model.njnt)}
        self._option = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self._option)
        self._camera = mujoco.MjvCamera()
        self._renderers: Dict[Tuple[int, int], object] = {}
        self._framing: Dict[Tuple, Tuple[np.ndarray, float]] = {}

        # Fingertip in each tip geom's own frame. The MJCF body layout does not
        # match the URDF's, so tip positions from `kinematics` cannot be reused
        # here -- they must come from this model.
        self._tip_local = {}
        for finger, geom in TIP_GEOM.items():
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom)
            if gid < 0:
                continue
            mid = self.model.geom_dataid[gid]
            adr, n = self.model.mesh_vertadr[mid], self.model.mesh_vertnum[mid]
            verts = self.model.mesh_vert[adr:adr + n]
            # MuJoCo recentres mesh vertices on the centroid, so the vertex
            # furthest from the origin here is whichever end of the pad is
            # further from its middle -- often the root, not the tip. Measure
            # from the body origin instead, which is what "fingertip" means.
            rot = np.zeros(9)
            mujoco.mju_quat2Mat(rot, self.model.geom_quat[gid])
            in_body = verts @ rot.reshape(3, 3).T + self.model.geom_pos[gid]
            far = verts[np.argmax(np.linalg.norm(in_body, axis=1))]
            self._tip_local[finger] = (gid, far)

        # Frame on the union of the open and fist poses, so the hand neither
        # drifts nor clips as it moves.
        pts = []
        for fw in ([0.0] * 6, [60.0, 30.0, 80.0, 80.0, 80.0, 80.0]):
            self._apply(fw)
            pts.append(self.data.geom_xpos.copy())
        allp = np.vstack(pts)
        self._lookat = (allp.min(axis=0) + allp.max(axis=0)) / 2

    def _apply(self, firmware: Sequence[float]) -> None:
        per_finger = HandModel.joints_from_firmware(firmware)
        for finger, (chain, _tip) in FINGER_CHAINS.items():
            for name, angle in zip(chain, per_finger[finger]):
                adr = self._qadr.get(name)
                if adr is not None:
                    self.data.qpos[adr] = angle
        self._mj.mj_forward(self.model, self.data)

    def _renderer_for(self, size: Tuple[int, int]):
        r = self._renderers.get(size)
        if r is None:
            r = self._mj.Renderer(self.model, height=size[1], width=size[0])
            self._renderers[size] = r
        return r

    def _raw(self, firmware, size, yaw, pitch, lookat, distance):
        import cv2
        self._apply(firmware)
        self._camera.lookat[:] = lookat
        self._camera.azimuth = yaw
        self._camera.elevation = pitch
        self._camera.distance = distance
        renderer = self._renderer_for(size)
        renderer.update_scene(self.data, camera=self._camera, scene_option=self._option)
        return cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)

    def _frame_for(self, size, yaw, pitch):
        """Distance and crop that centre the hand and fill the frame.

        A fixed camera distance frames well from one direction and badly from
        the next, and nudging `lookat` means getting MuJoCo's azimuth/elevation
        convention exactly right. Instead: render a larger canvas, measure where
        the hand actually lands, scale by distance and centre by cropping. The
        crop costs nothing and cannot point the camera the wrong way. Solved
        once per viewpoint and cached.
        """
        import cv2
        key = (size, round(yaw, 1), round(pitch, 1))
        cached = self._framing.get(key)
        if cached is not None:
            return cached

        big = (int(size[0] * OVERSCAN), int(size[1] * OVERSCAN))
        poses = ([0.0] * 6, [60.0, 30.0, 80.0, 80.0, 80.0, 80.0])
        distance = MJ_DISTANCE

        def union_bbox(dist):
            lo = np.array([big[0], big[1]], dtype=float)
            hi = np.zeros(2)
            for fw in poses:
                img = self._raw(fw, big, yaw, pitch, self._lookat, dist)
                ys, xs = np.nonzero(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) > 12)
                if len(xs):
                    lo = np.minimum(lo, [xs.min(), ys.min()])
                    hi = np.maximum(hi, [xs.max(), ys.max()])
            return lo, hi

        for _ in range(5):
            lo, hi = union_bbox(distance)
            if hi[0] <= lo[0]:
                break
            fill = max((hi[0] - lo[0]) / size[0], (hi[1] - lo[1]) / size[1])
            if abs(fill - TARGET_FILL) < 0.02:
                break
            distance *= fill / TARGET_FILL
        lo, hi = union_bbox(distance)
        centre = (lo + hi) / 2 if hi[0] > lo[0] else np.array(big) / 2
        x0 = int(np.clip(centre[0] - size[0] / 2, 0, big[0] - size[0]))
        y0 = int(np.clip(centre[1] - size[1] / 2, 0, big[1] - size[1]))
        self._framing[key] = (distance, (x0, y0), big)
        return self._framing[key]

    def _shot(self, firmware, size, yaw, pitch, zoom=1.0):
        distance, (x0, y0), big = self._frame_for(size, yaw, pitch)
        img = self._raw(firmware, big, yaw, pitch, self._lookat, distance / max(zoom, 1e-3))
        return img[y0:y0 + size[1], x0:x0 + size[0]]

    def _projector(self, firmware, size, yaw, pitch, zoom):
        """Render once, then return a world -> cropped-pixel function."""
        distance, (x0, y0), big = self._frame_for(size, yaw, pitch)
        self._raw(firmware, big, yaw, pitch, self._lookat, distance / max(zoom, 1e-3))
        scene = self._renderer_for(big).scene
        c0, c1 = scene.camera[0], scene.camera[1]
        # camera[0] and [1] are the two eyes; their midpoint is the mono camera.
        pos = (np.array(c0.pos) + np.array(c1.pos)) / 2
        fwd, up = np.array(c0.forward), np.array(c0.up)
        right = np.cross(fwd, up)
        W, H = big
        half_h = c0.frustum_top
        half_w = half_h * W / H

        def project(world):
            v = np.asarray(world) - pos
            z = float(np.dot(v, fwd))
            if z <= 1e-6:
                return None
            px = W / 2 * (1 + np.dot(v, right) * c0.frustum_near / z / half_w) - x0
            py = H / 2 * (1 - np.dot(v, up) * c0.frustum_near / z / half_h) - y0
            if -20 <= px <= size[0] + 20 and -20 <= py <= size[1] + 20:
                return float(px), float(py)
            return None
        return project

    def fingertip_pixels(self, firmware: Sequence[float], size: Tuple[int, int],
                         yaw: float = MJ_AZIMUTH, pitch: float = MJ_ELEVATION,
                         zoom: float = 1.0):
        """Where each fingertip lands in the cropped image, or None if behind."""
        project = self._projector(firmware, size, yaw, pitch, zoom)
        out = {}
        for finger, (gid, local) in self._tip_local.items():
            world = self.data.geom_xpos[gid] + self.data.geom_xmat[gid].reshape(3, 3) @ local
            xy = project(world)
            if xy is not None:
                out[finger] = xy
        return out

    def _geom_finger(self, gid: int) -> str:
        name = self._mj.mj_id2name(self.model, self._mj.mjtObj.mjOBJ_GEOM, gid) or ""
        if name.startswith("finger") and len(name) > 6:
            return GEOM_FINGER.get(name[6], name)
        return "palm" if name.startswith("base") else name

    def contacts(self, firmware: Sequence[float], size: Tuple[int, int],
                 yaw: float = MJ_AZIMUTH, pitch: float = MJ_ELEVATION,
                 zoom: float = 1.0):
        """Self-contacts as (pixel, depth_mm, 'thumb-index') tuples.

        The meshes are rigid while the real fingertips are rubber, so a genuine
        press reads here as overlap. Depth is how far the two have merged, which
        tracks roughly with how hard the hand is pressing on itself.
        """
        project = self._projector(firmware, size, yaw, pitch, zoom)
        out = []
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            depth = -c.dist * 1000.0
            if depth < CONTACT_MIN_MM:
                continue
            xy = project(c.pos)
            if xy is None:
                continue
            a, b = self._geom_finger(c.geom1), self._geom_finger(c.geom2)
            if a == b:
                continue
            out.append((xy, depth, f"{a}-{b}"))
        # Deepest first, and collapse duplicates from the same pair of parts.
        out.sort(key=lambda t: -t[1])
        seen, uniq = set(), []
        for xy, depth, label in out:
            if label in seen:
                continue
            seen.add(label)
            uniq.append((xy, depth, label))
        return uniq

    def render(self, firmware: Sequence[float], size: Tuple[int, int] = (400, 278),
               yaw: float = MJ_AZIMUTH, pitch: float = MJ_ELEVATION,
               measured: Optional[Sequence[float]] = None,
               background=BG, zoom: float = 1.0) -> np.ndarray:
        import cv2

        img = self._shot(firmware, size, yaw, pitch, zoom)
        if measured is not None and lagging(firmware, measured):
            # Outline of where the hand actually is, drawn only when it is
            # somewhere else. Drawing it unconditionally put a halo round the
            # hand at all times, so an arrived joint and a stuck one looked
            # identical and the comparison was worthless.
            ghost = self._shot(measured, size, yaw, pitch, zoom)
            gray = cv2.cvtColor(ghost, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, 12, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # Silhouette only. Edge detection would trace every panel line on
            # the mesh and bury the comparison in detail.
            cv2.drawContours(img, contours, -1, MEASURED_EDGE, 2, cv2.LINE_AA)
        return img


def HandRenderer(direction: str = "R", **kwargs):
    """Best available renderer: MuJoCo if it has a GL context, else hulls."""
    try:
        return MujocoHandRenderer(direction)
    except Exception as exc:
        print(f"hand view: MuJoCo renderer unavailable ({type(exc).__name__}); using hulls")
        return HullHandRenderer(direction, **kwargs)
