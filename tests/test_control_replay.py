"""Regression guard for the stable control path.

The FK retargeter's output on the canonical recording is frozen in
tests/data/control_reference.npz (default and recommended settings, plus the
six scalar tracker signals). Any change to tracker.py, fkretarget.py or
kinematics.py that alters a commanded angle fails here, before it can reach
the hand. Rendering the phantom view between frames must not change anything
either.

    cd ~/telehand-lat && python3 tests/test_control_replay.py
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vendor"))
sys.path.insert(0, str(ROOT))

import numpy as np

from telehand.fkretarget import FKRetargeter
from telehand.hand import JointLimit
from telehand.kinematics import FIRMWARE_MAX_DEG
from telehand.tracker import HandReading, extract_signals
from telehand import phantom as ph

failures = 0


def check(cond, label):
    global failures
    print(f"  {'PASS' if cond else 'FAIL'}  {label}")
    if not cond:
        failures += 1


rec = np.load(ROOT / "tests" / "data" / "fk_session.npz", allow_pickle=True)
ref = np.load(ROOT / "tests" / "data" / "control_reference.npz")
P, LAB = rec["world_pts"].astype(np.float64), rec["handedness"]
limits = [JointLimit(0.0, m) for m in FIRMWARE_MAX_DEG]

print("scalar FK commands vs frozen reference")
for name, (sm, db) in {"default": (0.35, 0.8), "recommended": (0.7, 0.4)}.items():
    fk = FKRetargeter(limits, smoothing=sm, deadband_deg=db)
    out = np.array([fk(p, "Left") for p in P])
    check(np.array_equal(out, ref[name]),
          f"{name} settings (smoothing {sm}, deadband {db}): identical on {len(P)} frames "
          f"(max |diff| {np.abs(out - ref[name]).max():.1e})")
sig = np.array([extract_signals(p) for p in P])
check(np.array_equal(sig, ref["signals"]), "tracker scalar signals identical")

print("phantom view between frames leaves the commands untouched")
fk = FKRetargeter(limits, smoothing=0.7, deadband_deg=0.4)
view = ph.PhantomHand(hz=0, joint_frames=True)
out = []
for p, l in zip(P, LAB):
    out.append(fk(p, "Left"))
    view.render(HandReading(extract_signals(p), str(l), None, p), 480)
check(np.array_equal(np.array(out), ref["recommended"]), f"identical with the phantom rendering ({view.failures} failures)")

print()
print("ALL PASSED" if failures == 0 else f"{failures} FAILED")
sys.exit(1 if failures else 0)
