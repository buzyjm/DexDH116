"""Checks for telehand.framemap: mapping, fallback, smoothing, calibration.

    cd ~/telehand-lat && python3 tests/test_framemap.py
"""
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vendor"))
sys.path.insert(0, str(ROOT))

import numpy as np

from telehand import handpose as hp
from telehand import framemap as fm
from telehand.hand import JointLimit

failures = 0


def check(cond, label):
    global failures
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        failures += 1


def hand(curl_deg=0.0, thumb_out=40.0):
    p = np.zeros((21, 3))
    for lm, y in {5: 0.020, 9: 0.0, 13: -0.020, 17: -0.040}.items():
        p[lm] = [0.0, y, 0.090]
    c = np.radians(curl_deg)
    for base in (5, 9, 13, 17):
        pos = p[base].copy(); ang = 0.0
        for k, L in enumerate((0.040, 0.025, 0.022)):
            ang += c / 3.0
            pos = pos + L * np.array([np.sin(ang), 0.0, np.cos(ang)])
            p[base + k + 1] = pos
    t = np.radians(thumb_out)
    p[1] = [0.010, 0.030, 0.030]
    d = np.array([0.2, np.sin(t), np.cos(t)]); d /= np.linalg.norm(d)
    p[2] = p[1] + 0.040 * d; p[3] = p[2] + 0.030 * d; p[4] = p[3] + 0.025 * d
    return p


LIMITS = [JointLimit(0, 60), JointLimit(0, 30)] + [JointLimit(0, 80)] * 4
FK = [10.0, 5.0, 20.0, 33.0, 44.0, 55.0]

print("frame signals")
s_open = fm.frame_signals(hp.HandPose3D.from_landmarks(hand(0)))
s_fist = fm.frame_signals(hp.HandPose3D.from_landmarks(hand(150)))
check(set(s_open) == set(fm.FRAME_SIGNALS), "all four signals present")
check(s_fist["index_curl"] > s_open["index_curl"] + 100, f"index_curl rises with curl ({s_open['index_curl']:.0f} -> {s_fist['index_curl']:.0f})")
sw = [fm.frame_signals(hp.HandPose3D.from_landmarks(hand(0, th)))["thumb_swing"] for th in (20, 40, 60)]
check(sw[0] > sw[1] > sw[2], f"thumb_swing falls as the metacarpal points further toward +y ({[round(v) for v in sw]})")

print("mapping")
m = fm.FrameMapper(LIMITS, fm.FrameCalibration(), smoothing=1.0, deadband_deg=0.0)
r_open = m.map(hp.HandPose3D.from_landmarks(hand(0)), FK)
m.reset(); r_fist = m.map(hp.HandPose3D.from_landmarks(hand(150)), FK)
check(r_open.source == ["F", "F", "F"] and r_fist.source == ["F", "F", "F"], "clean poses use the frame signals")
check(r_fist.applied[2] > r_open.applied[2] and 0 <= r_open.applied[2] <= 80 and 0 <= r_fist.applied[2] <= 80,
      f"index command rises with curl and stays inside limits ({r_open.applied[2]:.1f} -> {r_fist.applied[2]:.1f})")
check(r_open.angles[3:] == FK[3:], "middle/ring/pinky are the FK values untouched")
check(r_open.fk_cmd == FK[:3], "the FK values for the mapped joints are recorded")
ranges = fm.FrameCalibration(); ranges.ranges["index_curl"] = [s_open["index_curl"], s_fist["index_curl"]]
m2 = fm.FrameMapper(LIMITS, ranges, smoothing=1.0, deadband_deg=0.0)
a = m2.map(hp.HandPose3D.from_landmarks(hand(0)), FK).applied[2]; m2.reset()
b = m2.map(hp.HandPose3D.from_landmarks(hand(150)), FK).applied[2]
check(abs(a) < 1e-6 and abs(b - 80) < 1e-6, f"calibrated open/fist hit exactly joint min/max ({a:.2f}, {b:.2f})")

print("fallback")
m = fm.FrameMapper(LIMITS, fm.FrameCalibration(), smoothing=1.0, deadband_deg=0.0, max_disp_mm=30.0)
m.map(hp.HandPose3D.from_landmarks(hand(0)), FK)
far = hand(0); far[8] += [0.0, 0.0, 0.05]            # index tip jumps 50 mm
r = m.map(hp.HandPose3D.from_landmarks(far), FK)
check(r.source == ["K", "K", "K"] and not r.plausible and "landmark jump" in r.reason
      and r.applied == FK[:3], f"a 50 mm landmark jump falls every mapped joint back to FK ({r.reason})")
m = fm.FrameMapper(LIMITS, fm.FrameCalibration(), smoothing=1.0, deadband_deg=0.0)
q = hand(60); q[7] = q[6]                             # zero-length index bone -> degenerate
r = m.map(hp.HandPose3D.from_landmarks(q), FK)
check(r.source == ["F", "F", "K"] and r.applied[2] == FK[2] and "index degenerate" in r.reason,
      "a degenerate index bone falls back only the index joint")
tr = hp.HandPoseTracker(history=50)
for _ in range(30):
    tr.update(hand(0))
stretched = hand(0); stretched[8] = stretched[7] + 4 * (stretched[8] - stretched[7])
po = tr.update(stretched)
m = fm.FrameMapper(LIMITS, fm.FrameCalibration(), smoothing=1.0, deadband_deg=0.0, min_conf=0.05)
r = m.map(po, FK)
check(r.source[2] == "K" and r.source[0] == "F" and "index conf" in r.reason,
      f"near-zero index confidence falls back the index joint only (conf {r.conf_index:.2f})")

print("smoothing / deadband")
m = fm.FrameMapper(LIMITS, fm.FrameCalibration(), smoothing=0.35, deadband_deg=0.8)
start = m.map(hp.HandPose3D.from_landmarks(hand(0)), FK).applied[2]
vals = [m.map(hp.HandPose3D.from_landmarks(hand(150)), FK).applied[2] for _ in range(40)]
target = m.map(hp.HandPose3D.from_landmarks(hand(150)), FK).frame_cmd[2]
check(all(b >= a for a, b in zip(vals, vals[1:])) and vals[0] > start and abs(vals[-1] - target) <= 0.8,
      f"EMA rises monotonically from {start:.1f} to within the deadband of the target ({vals[-1]:.1f} vs {target:.1f})")
before = vals[-1]
m2 = fm.FrameMapper(LIMITS, fm.FrameCalibration(), smoothing=1.0, deadband_deg=5.0)
m2.map(hp.HandPose3D.from_landmarks(hand(100)), FK)
r1 = m2.map(hp.HandPose3D.from_landmarks(hand(101)), FK).applied[2]
r2 = m2.map(hp.HandPose3D.from_landmarks(hand(100)), FK).applied[2]
check(r1 == r2, "a change under the deadband is held")

print("calibration file")
with tempfile.TemporaryDirectory() as d:
    path = Path(d) / "frames_calibration.json"
    cal = fm.FrameCalibration(); cap = fm.FrameRangeCapture(frames=3)
    cap.start(1)
    done = [cap.feed(s_fist, cal) for _ in range(3)]
    check(done[-1] == "closed" and cal.calibrated and abs(cal.ranges["index_curl"][1] - s_fist["index_curl"]) < 1e-9,
          "capture averages N frames into the closed end")
    cal.save(path)
    back = fm.FrameCalibration.load(path)
    check(back.ranges == cal.ranges and back.calibrated, "save/load round-trips")
    check(not fm.FrameCalibration.load(Path(d) / "missing.json").calibrated, "a missing file gives defaults, uncalibrated")

print("log row")
row = fm.log_row(1.5, "frames", True, [0.0] * 6, r_open, hp.Canonicalizer(), [1, 2, 3, 4, 5, 6])
check(row.count(",") == len(fm.LOG_COLUMNS) - 1, "log row has exactly the header's columns")

print()
print("ALL PASSED" if failures == 0 else f"{failures} FAILED")
sys.exit(1 if failures else 0)
