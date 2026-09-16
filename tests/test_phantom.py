"""Phantom hand verification without hardware.

1. commanded angles identical with the view on vs off on the same replay;
2. it tracks open hand, fist, pinch and index curl (montage);
3. no chirality flips / 180-degree turns over the recording;
4. a rendering failure cannot reach the loop;
5. render cost, and the refresh throttle.

    cd ~/telehand-lat && python3 tests/test_phantom.py [montage.png]
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vendor"))
sys.path.insert(0, str(ROOT))

import cv2
import numpy as np

from telehand import phantom as ph
from telehand.fkretarget import FKRetargeter
from telehand.hand import JointLimit
from telehand.kinematics import FIRMWARE_MAX_DEG
from telehand.tracker import HandReading, extract_signals, _chain_bend, FINGER_CHAINS

failures = 0


def check(cond, label):
    global failures
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        failures += 1


d = np.load(str(ROOT / "tests" / "data" / "fk_session.npz"), allow_pickle=True)
P, LAB = d["world_pts"].astype(np.float64), d["handedness"]
readings = [HandReading(extract_signals(p), str(l), None, p) for p, l in zip(P, LAB)]
limits = [JointLimit(0.0, m) for m in FIRMWARE_MAX_DEG]
H = 480

curl = np.array([_chain_bend(p, FINGER_CHAINS["index"]) for p in P])
allcurl = np.array([sum(_chain_bend(p, FINGER_CHAINS[f]) for f in ("index", "middle", "ring", "pinky")) for p in P])
pinch = np.array([np.linalg.norm(p[4] - p[8]) for p in P])
# Skip the first second: the hand is still entering the view there.
S = 30
i_open, i_fist = S + int(np.argmin(allcurl[S:700])), S + int(np.argmax(allcurl[S:700]))
i_pinch = S + int(np.argmin(pinch[S:700]))
i_curl = S + int(np.argmax(curl[S:700] - 0.3 * (allcurl[S:700] - curl[S:700])))

print("0. depth convention")
fist = P[i_fist]
tips_z = fist[list(ph.TIPS)][:, 2].mean(); mcp_z = fist[[5, 9, 13, 17]][:, 2].mean()
print(f"       fist frame {i_fist}: mean world z of tips {tips_z:+.4f}, of knuckles {mcp_z:+.4f} "
      f"(MediaPipe: smaller z = closer to the camera)")
view = ph.PhantomHand()
scene = view.scene_points(readings[i_fist])
check(scene[list(ph.TIPS)][:, 2].mean() > scene[[5, 9, 13, 17]][:, 2].mean(),
      "curled fingertips come out nearer the viewer than the knuckles (scene z toward viewer)")

print("1. control path invariance (FK over the replay, view off vs on)")
fk_off, fk_on = FKRetargeter(limits, smoothing=0.35, deadband_deg=0.8), FKRetargeter(limits, smoothing=0.35, deadband_deg=0.8)
view = ph.PhantomHand(hz=0)                # hz=0: render every call (the throttle is tested separately)
a_off, a_on = [], []
for r in readings:
    a_off.append(fk_off(r.world_pts, "Left"))
    a = fk_on(r.world_pts, "Left")
    view.render(r, H)                         # as in the display block, after the command
    a_on.append(a)
a_off, a_on = np.array(a_off), np.array(a_on)
check(np.array_equal(a_off, a_on), f"commanded angles identical on all {len(P)} frames (max |diff| {np.abs(a_off - a_on).max():.1e})")
check(view.failures == 0 and view.rendered == len(P), f"rendered every frame without failure ({view.rendered}/{len(P)})")

print("3. chirality / 180-degree turns over the recording (display points, before smoothing)")
view = ph.PhantomHand()
prev_n = None; prev_pts = None; flips = []; jumps = []
for i, r in enumerate(readings):
    s = view.scene_points(r)
    y0 = s[5] - s[17]; z0 = s[[5, 9, 13]].mean(0) - s[0]
    n = np.cross(y0, z0); n /= np.linalg.norm(n)
    if prev_n is not None:
        if np.dot(n, prev_n) < 0:
            flips.append(i)
        jumps.append(float(np.linalg.norm(s - prev_pts, axis=1).max() * 1000))
    prev_n, prev_pts = n, s
jumps = np.array(jumps)
check(len([f for f in flips if f < 728]) == 0,
      f"palm-normal sign never flips on frames 1..727 (flips at {flips or 'none'}; the recording's own "
      f"landmark glitch is at 728-729)")
check(view.canon.counts["switches"] == 1,
      f"the one genuine geometry flip (frame 728) was absorbed by the canonicalizer ({view.canon.counts['switches']} switch)")
print(f"       largest per-frame joint move: p95 {np.percentile(jumps, 95):.1f} mm, max {jumps.max():.1f} mm "
      f"at frame {int(jumps.argmax()) + 1}; frames above 30 mm: {[int(j) + 1 for j in np.where(jumps > 30)[0]]}")

print("4. rendering failure cannot reach the loop")
bad = ph.PhantomHand(hz=0)
bad._draw_scene = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("synthetic render failure"))
raised = False
try:
    imgs = [bad.render(r, H) for r in readings[:40]]
except Exception:
    raised = True
check(not raised and bad.failures == 40 and all(im.shape == (H, bad.width, 3) for im in imgs),
      f"40 failing frames swallowed, counted ({bad.failures}), placeholder panels returned")
blank = ph.PhantomHand(hz=0)
im = blank.render(None, H)
check(im.shape == (H, blank.width, 3) and blank.failures == 0, "no reading -> 'no hand' panel, no failure")

print("5. render cost and throttle")
view = ph.PhantomHand(hz=0)
t0 = time.perf_counter()
for r in readings:
    view.render(r, H)
ms = (time.perf_counter() - t0) / len(readings) * 1000
check(ms < 4.0, f"{ms:.2f} ms/frame at {view.width}x{H} (all 21 joints, 23 bones, grid, shadow, fog)")
slow = ph.PhantomHand(hz=15.0)
t = time.perf_counter()
for r in readings[:60]:
    slow.render(r, H)
    while time.perf_counter() - t < 1 / 30.0:
        pass
    t = time.perf_counter()
check(20 <= slow.rendered <= 40, f"hz=15 at a 30 Hz feed re-renders about every other frame ({slow.rendered}/60)")

print("6. joint frames: attached rigidly, cost, toggle, control invariance")
v = ph.PhantomHand(hz=0, joint_frames=True)
worst = 0.0; n_axes = 0
for r in readings[30:700:7]:
    v.render(r, H)
    pose = v.last_pose
    assert pose is not None
    pts = ph.PhantomHand.to_scene(v._smooth_canon)
    for name, j in pose.joints.items():
        if name == "palm" or name.endswith("_tip"):
            continue
        child = next(c for c in pose.joints.values() if c.parent == name)
        bone = pts[child.landmark] - pts[j.landmark]; bone /= np.linalg.norm(bone)
        z_scene = ph.DIR_TO_SCENE * (pose.wrist_rotation @ j.rotation)[:, 2]
        worst = max(worst, 1.0 - float(np.dot(bone, z_scene))); n_axes += 1
check(worst < 1e-9, f"every joint's drawn z axis lies along its child bone in scene space ({n_axes} joints, worst 1-dot {worst:.1e})")
check(v.frame_failures == 0 and v.failures == 0, f"joint frames built on every frame ({v.rendered} renders)")
off, on = ph.PhantomHand(hz=0), ph.PhantomHand(hz=0, joint_frames=True)
t0 = time.perf_counter()
for r in readings:
    off.render(r, H)
t_off = (time.perf_counter() - t0) / len(readings) * 1000
t0 = time.perf_counter()
for r in readings:
    on.render(r, H)
t_on = (time.perf_counter() - t0) / len(readings) * 1000
check(t_on - t_off < 3.0, f"joint frames add {t_on - t_off:.2f} ms/render ({t_off:.2f} -> {t_on:.2f} ms)")
tg = ph.PhantomHand(hz=0)
tg.render(readings[100], H); a = tg.last_pose
tg.toggle_frames(); tg.render(readings[101], H); b = tg.last_pose
tg.toggle_frames(); tg.render(readings[102], H); c = tg.last_pose
check(a is None and b is not None and c is None, "f toggle switches joint frames on and off")
fk1, fk2 = FKRetargeter(limits, smoothing=0.35, deadband_deg=0.8), FKRetargeter(limits, smoothing=0.35, deadband_deg=0.8)
pv = ph.PhantomHand(hz=0, joint_frames=True); o1, o2 = [], []
for r in readings:
    o1.append(fk1(r.world_pts, "Left")); o2.append(fk2(r.world_pts, "Left")); pv.render(r, H)
check(np.array_equal(np.array(o1), np.array(o2)), "commanded angles identical with joint frames on")

print("2. representative poses (all joint frames on)")
picks = [("open hand", i_open), ("fist", i_fist), ("pinch", i_pinch), ("index curl", i_curl)]
tiles = []
for label, i in picks:
    v = ph.PhantomHand(hz=0, joint_frames=True)
    for k in range(max(0, i - 20), i):
        v.render(readings[k], H)
    img = v.render(readings[i], H).copy()
    cv2.putText(img, f"#{i} {label}", (12, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    tiles.append(img)
    check(v.failures == 0 and v.frame_failures == 0, f"{label} (frame {i}) rendered with joint frames")
montage = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/phantom_montage.png")
cv2.imwrite(str(out), montage)
print(f"       montage -> {out}")

print()
print("ALL PASSED" if failures == 0 else f"{failures} FAILED")
sys.exit(1 if failures else 0)
