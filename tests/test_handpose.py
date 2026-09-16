"""Synthetic checks for telehand.handpose. No hardware, no camera, no MediaPipe.

    cd ~/telehand-lat && python3 tests/test_handpose.py
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vendor"))
sys.path.insert(0, str(ROOT))

import numpy as np
from scipy.spatial.transform import Rotation

from telehand import handpose as hp
from telehand.tracker import _angle_between, _chain_bend, FINGER_CHAINS

rng = np.random.default_rng(7)
failures = 0


def check(cond, label):
    global failures
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        failures += 1


def synthetic_right_hand(curl_deg=0.0, thumb_out=40.0):
    """A plausible right hand in the canonical frame: wrist at origin, fingers
    along +z, thumb side +y, palm facing +x, fingers curling toward +x.
    Index/middle/ring MCPs are y-symmetric so the specified z0 is exactly +z."""
    p = np.zeros((21, 3))
    mcp_y = {5: 0.020, 9: 0.0, 13: -0.020, 17: -0.040}
    for lm, y in mcp_y.items():
        p[lm] = [0.0, y, 0.090]
    c = np.radians(curl_deg)
    for base in (5, 9, 13, 17):
        pos = p[base].copy()
        ang = 0.0
        for k, L in enumerate((0.040, 0.025, 0.022)):
            ang += c / 3.0
            pos = pos + L * np.array([np.sin(ang), 0.0, np.cos(ang)])
            p[base + k + 1] = pos
    t = np.radians(thumb_out)
    p[1] = [0.010, 0.030, 0.030]
    d = np.array([0.2, np.sin(t), np.cos(t)])
    d /= np.linalg.norm(d)
    p[2] = p[1] + 0.040 * d
    p[3] = p[2] + 0.030 * d
    p[4] = p[3] + 0.025 * d
    return p


def rigid(p, R=None, t=None):
    R = Rotation.random(random_state=rng.integers(1 << 30)).as_matrix() if R is None else R
    t = rng.normal(0, 0.5, 3) if t is None else t
    return p @ R.T + t


print("minimal_rotation")
for _ in range(200):
    a = hp.unit(rng.normal(size=3)); b = hp.unit(rng.normal(size=3))
    R, deg = hp.minimal_rotation(a, b)
    assert not deg
    assert np.allclose(R @ a, b, atol=1e-12) and abs(np.linalg.det(R) - 1) < 1e-12 \
        and np.allclose(R.T @ R, np.eye(3), atol=1e-12)
check(True, "R a == b, det +1, orthonormal on 200 random pairs")
R, deg = hp.minimal_rotation(np.array([0, 0, 1.0]), np.array([0, 0, 1.0]))
check(np.allclose(R, np.eye(3)), "identity when a == b")
a = np.array([0, 0, 1.0]); R, deg = hp.minimal_rotation(a, -a, fallback_axis=np.array([1.0, 0, 0]))
check(deg and np.allclose(R @ a, -a) and abs(np.linalg.det(R) - 1) < 1e-12,
      "antiparallel: flagged, still a proper rotation taking a to -a")

print("palm frame convention")
p = synthetic_right_hand()
pose = hp.HandPose3D.from_landmarks(p)
Rw = pose.wrist_rotation
y0 = p[5] - p[17]; z0 = p[[5, 9, 13]].mean(0) - p[0]
check(np.dot(Rw[:, 0], np.cross(y0, z0)) > 0, "dot(x, y0 x z0) > 0")
check(np.allclose(Rw, np.eye(3), atol=1e-9), "canonical hand gives R_W == I (x=+X out of palm, y=+Y thumb side, z=+Z distal)")
check(abs(np.linalg.det(Rw) - 1) < 1e-12, "det(R_W) == +1")
for _ in range(100):
    q = synthetic_right_hand(curl_deg=rng.uniform(0, 170), thumb_out=rng.uniform(10, 70))
    q = rigid(q) + rng.normal(0, 0.002, q.shape)
    po = hp.HandPose3D.from_landmarks(q)
    y0 = q[5] - q[17]; z0 = q[[5, 9, 13]].mean(0) - q[0]
    assert np.dot(po.wrist_rotation[:, 0], np.cross(y0, z0)) > 0
    for j in po.joints.values():
        assert abs(np.linalg.det(j.rotation) - 1) < 1e-9 and np.allclose(j.rotation.T @ j.rotation, np.eye(3), atol=1e-9)
check(True, "sign rule and det +1 hold on 100 noisy, randomly posed hands, every joint")

print("wrist-locality and rigid invariance")
base = hp.HandPose3D.from_landmarks(synthetic_right_hand(curl_deg=60))
check(np.allclose(base.joints["palm"].position, 0) and np.allclose(base.joints["palm"].rotation, np.eye(3)),
      "palm joint is the origin with identity rotation")
worst_p = worst_r = 0.0
for _ in range(100):
    moved = rigid(synthetic_right_hand(curl_deg=60))
    other = hp.HandPose3D.from_landmarks(moved)
    for n in base.joints:
        worst_p = max(worst_p, np.linalg.norm(other.joints[n].position - base.joints[n].position))
        worst_r = max(worst_r, np.linalg.norm(other.joints[n].rotation - base.joints[n].rotation))
check(worst_p < 1e-9 and worst_r < 1e-9,
      f"100 random SE(3) transforms of the input: max |dpos| {worst_p:.1e} m, max |dR| {worst_r:.1e}")

print("bend angles reproduce the existing angle functions")
worst = 0.0
for _ in range(100):
    q = rigid(synthetic_right_hand(curl_deg=rng.uniform(0, 170))) + rng.normal(0, 0.003, (21, 3))
    po = hp.HandPose3D.from_landmarks(q)
    for finger, (a, b, c, d) in (("index", (5, 6, 7, 8)), ("thumb", (1, 2, 3, 4))):
        suf = hp._JOINT_SUFFIX[finger]
        worst = max(worst, abs(po.bend_angle_deg(f"{finger}_{suf[1]}") - _angle_between(q[b] - q[a], q[c] - q[b])))
        worst = max(worst, abs(po.bend_angle_deg(f"{finger}_{suf[2]}") - _angle_between(q[c] - q[b], q[d] - q[c])))
        worst = max(worst, abs(po.chain_bend_deg(finger) - _chain_bend(q, FINGER_CHAINS[finger])))
check(worst < 1e-9, f"PIP/DIP/MCP/IP bends and chain sums match tracker to {worst:.1e} deg")

print("chain frames")
po = hp.HandPose3D.from_landmarks(synthetic_right_hand(curl_deg=90))
q = po.points
for finger in ("index", "thumb"):
    idx = hp.CHAINS[finger]; suf = hp._JOINT_SUFFIX[finger]
    for k in range(3):
        j = po.joints[f"{finger}_{suf[k]}"]
        bone = hp.unit(q[idx[k + 1]] - q[idx[k]])
        assert np.allclose(j.rotation[:, 2], bone, atol=1e-12)
check(True, "every joint's local z lies along its child bone")
po = hp.HandPose3D.from_landmarks(synthetic_right_hand(curl_deg=0))
check(np.allclose(po.joints["index_mcp"].rotation, np.eye(3), atol=1e-9)
      and np.allclose(po.joints["index_dip"].rotation, np.eye(3), atol=1e-9),
      "a straight index finger along palm z inherits the palm frame exactly (no twist introduced)")
# Geometry check only, not an anatomical claim: R_y(theta) z = (sin, 0, cos), so a
# synthetic curl toward +x is a positive rotation about the local +y axis.
po = hp.HandPose3D.from_landmarks(synthetic_right_hand(curl_deg=90))
rv = po.relative_rotvec("index_pip")
check(np.allclose(rv, [0.0, 30.0, 0.0], atol=1e-6),
      f"synthetic curl of 30 deg/joint toward +x is a 30 deg rotation about local +y ({rv.round(4)})")

print("input adapter: geometry-authoritative canonicalization")
right = synthetic_right_hand(curl_deg=100)
left_geometry = hp.mirror_x(right)                                  # what MediaPipe emits for a 'Left' label
check(hp.chirality_vote(right) > hp.VOTE_CONF and hp.chirality_vote(left_geometry) < -hp.VOTE_CONF,
      "curl vote reads chirality from the geometry alone")
check(abs(hp.chirality_vote(synthetic_right_hand(curl_deg=0))) < 1e-9, "an open hand casts no vote")
c = hp.Canonicalizer(assume_mirrored=True)
out = c(left_geometry, "Left")
check(np.allclose(out, right) and c.last_source == "geometry", "left geometry is mirrored back to the right hand, by geometry")
out = c(right, "Right")
check(np.allclose(out, right) and c.counts["switches"] == 1, "a genuine flip in the emitted geometry is caught in the same frame")
# Label flicker on an open hand must not change anything.
c = hp.Canonicalizer(assume_mirrored=True)
c(left_geometry, "Left")
open_left = hp.mirror_x(synthetic_right_hand(curl_deg=0))
a1 = c(open_left, "Left"); a2 = c(open_left, "Right"); a3 = c(open_left, "Left")
check(np.allclose(a1, a2) and np.allclose(a2, a3) and c.last_source == "continuity"
      and c.counts["label_disagreements"] == 1 and c.counts["switches"] == 0,
      "label flicker on an open hand changes nothing (continuity), and is only counted")
# Before any evidence, the default comes from the webcam setup, not the label.
c = hp.Canonicalizer(assume_mirrored=True)
first = c(open_left, "Right")
check(np.allclose(first, hp.mirror_x(open_left)) and c.last_source == "default",
      "first open-hand frame uses the mirrored-webcam default even when the label says Right")
c = hp.Canonicalizer(assume_mirrored=False)
check(np.allclose(c(synthetic_right_hand(), "Left"), synthetic_right_hand()) and c.last_source == "default",
      "with an unmirrored setup the default is right, label ignored")
pa = hp.HandPose3D.from_landmarks(hp.Canonicalizer()(left_geometry)); pb = hp.HandPose3D.from_landmarks(right)
check(all(np.allclose(pa.joints[n].rotation, pb.joints[n].rotation) for n in pa.joints),
      "identical frames from the canonicalized and the original right hand")

print("guards")
q = synthetic_right_hand(); q[7] = q[6]          # zero-length index middle bone
po = hp.HandPose3D.from_landmarks(q)
check(po.joints["index_pip"].degenerate and abs(np.linalg.det(po.joints["index_dip"].rotation) - 1) < 1e-12,
      "zero-length bone flagged, downstream frames still proper rotations")
tr = hp.HandPoseTracker()
for i in range(40):
    tr.update(synthetic_right_hand() + rng.normal(0, 0.002, (21, 3)))     # 2 mm noise, like MediaPipe
po = tr.update(synthetic_right_hand() + rng.normal(0, 0.002, (21, 3)))
check(all(j.confidence > 0.9 for j in po.joints.values()),
      f"confidence stays high under the same noise it was calibrated on (min {min(j.confidence for j in po.joints.values()):.2f})")
q = synthetic_right_hand(); q[8] = q[7] + 3 * (q[8] - q[7])    # index tip bone 3x too long
po = tr.update(q)
check(po.joints["index_dip"].confidence == 0.0 and po.joints["index_tip"].confidence == 0.0
      and po.joints["index_mcp"].confidence > 0.9,
      "a stretched bone zeroes only the joints it touches")

print()
print("ALL PASSED" if failures == 0 else f"{failures} FAILED")
sys.exit(1 if failures else 0)
