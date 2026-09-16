"""Overlay verification without hardware.

1. commanded angles are identical with the overlay on vs off on the same replay;
2. an overlay failure cannot reach the loop;
3. render cost;
4. representative poses rendered to a montage.

2-D landmarks are synthesised from the recorded world landmarks with a pinhole
camera (fk_session.npz predates landmarks_2d recording), so the fit sees real
perspective; the live run is where real MediaPipe 2-D points are tested.

    cd ~/telehand-lat && python3 tests/test_overlay.py [montage.png]
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vendor"))
sys.path.insert(0, str(ROOT))

import cv2
import numpy as np

from telehand import poseoverlay as ov
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


REC = Path("/home/xu-square/telehand/fk_session.npz")
d = np.load(REC, allow_pickle=True)
P, LAB = d["world_pts"].astype(np.float64), d["handedness"]
W, H = 640, 480


def pinhole(pts, fx=650.0, dist=0.55, cx=W / 2, cy=H / 2):
    """Perspective camera in front of the hand: image x right, y down."""
    Z = pts[:, 2] + dist
    return np.stack([cx + fx * pts[:, 0] / Z, cy + fx * pts[:, 1] / Z], axis=1).astype(np.float32)


readings = [HandReading(extract_signals(p), str(l), pinhole(p), p) for p, l in zip(P, LAB)]
limits = [JointLimit(0.0, m) for m in FIRMWARE_MAX_DEG]

print("1. control path invariance (FK solver over the replay, overlay off vs on)")
fk_off = FKRetargeter(limits, smoothing=0.35, deadband_deg=0.8)
fk_on = FKRetargeter(limits, smoothing=0.35, deadband_deg=0.8)
overlay = ov.PoseOverlay(assume_mirrored=True)
canvas = np.zeros((H, W, 3), np.uint8)
out_off, out_on = [], []
for r in readings:
    out_off.append(fk_off(r.world_pts, "Left"))
    a = fk_on(r.world_pts, "Left")             # same call teleop makes ...
    overlay.draw(canvas, r)                     # ... then the overlay draws, as in the display block
    out_on.append(a)
out_off, out_on = np.array(out_off), np.array(out_on)
check(np.array_equal(out_off, out_on), f"commanded angles identical on all {len(P)} frames (max |diff| {np.abs(out_off - out_on).max():.1e})")
check(overlay.failures == 0, f"no overlay failures on real landmarks ({overlay.drawn} drawn, {overlay.skipped} skeleton-only)")

print("2. visualisation failure cannot reach the loop")
bad = ov.PoseOverlay()
bad._draw = lambda *a, **k: (_ for _ in ()).throw(RuntimeError("synthetic overlay failure"))
raised = False
try:
    for r in readings[:50]:
        bad.draw(canvas, r)
except Exception:
    raised = True
check(not raised and bad.failures == 50, f"50 failing frames swallowed and counted ({bad.failures})")
none = ov.PoseOverlay()
check(none.draw(canvas, None) is False and none.draw(canvas, HandReading([0] * 6, "?", None, P[0])) is False,
      "missing reading / missing 2-D landmarks simply skip")

print("3. render cost")
ov2 = ov.PoseOverlay()
t0 = time.perf_counter()
for r in readings:
    ov2.draw(canvas, r)
ms = (time.perf_counter() - t0) / len(readings) * 1000
resid = []
ov3 = ov.PoseOverlay()
for r in readings:
    ov3.draw(canvas, r); resid.append(ov3.last_resid_px)
resid = np.array(resid)
check(ms < 3.0, f"{ms:.2f} ms/frame on a {W}x{H} canvas (skeleton + pose + fit + 6 triads)")
print(f"       affine fit residual vs a real pinhole: p50 {np.median(resid):.2f} px, p95 {np.percentile(resid, 95):.2f} px, "
      f"max {resid.max():.2f} px (axes skipped above {ov3.max_resid_px:.0f})")

print("4. representative poses")
curl = np.array([_chain_bend(p, FINGER_CHAINS["index"]) for p in P])
allcurl = np.array([sum(_chain_bend(p, FINGER_CHAINS[f]) for f in ("index", "middle", "ring", "pinky")) for p in P])
pinch = np.array([np.linalg.norm(p[4] - p[8]) for p in P])
picks = [("open hand", int(np.argmin(allcurl[:700]))), ("fist", int(np.argmax(allcurl[:700]))),
         ("pinch", int(np.argmin(pinch[:700]))), ("index curl", int(np.argmax(curl[:700] - 0.3 * (allcurl[:700] - curl[:700]))))]
ov4 = ov.PoseOverlay()
tiles = []
for label, i in picks:
    for k in range(max(0, i - 15), i):          # warm the smoothers the way a live run would
        ov4.draw(np.zeros((H, W, 3), np.uint8), readings[k])
    img = np.full((H, W, 3), 38, np.uint8)
    ok = ov4.draw(img, readings[i])
    cv2.putText(img, f"#{i} {label}   axes {'drawn' if ok else 'SKIPPED'}   fit {ov4.last_resid_px:.1f}px",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (235, 235, 235), 1, cv2.LINE_AA)
    tiles.append(img)
    check(ok, f"{label} (frame {i}): axes drawn, fit residual {ov4.last_resid_px:.1f} px")
montage = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/overlay_montage.png")
cv2.imwrite(str(out), montage)
print(f"       montage -> {out}")

print()
print("ALL PASSED" if failures == 0 else f"{failures} FAILED")
sys.exit(1 if failures else 0)
