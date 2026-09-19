"""``photostep``: park joints at preset angles for calibration photos."""

from __future__ import annotations

import time

from ..hand import JOINT_NAMES, NUM_JOINTS, Hand


PHOTO_PLAN = {
    # joint index -> firmware angles to photograph at
    2: [0, 20, 40, 60, 80],        # index flexion, side view
    1: [0, 7.5, 15, 22.5, 30],     # thumb flexion, side view
    0: [0, 30, 60],                # thumb abduction, view from above the palm
}


def run(args) -> int:
    """Park one joint at preset angles, pausing for a photo at each.

    The firmware angle is a linear function of actuator stroke, not of the
    joint; a few side-view photos at known commands give the joint angle
    directly, which is the one thing the vendor files do not contain.
    """
    joints = [int(v) for v in args.joints.split(",")]
    hand = Hand(args.port, args.baud, max_current=args.max_current)
    print(f"Connecting to hand on {args.port} ...")
    print("Homing: the hand will move through its full range. Keep it clear.")
    hand.connect(home=True)
    print("Connected.\n")
    try:
        for j in joints:
            name = JOINT_NAMES[j]
            print(f"=== {name} (joint {j+1}) ===")
            for a in PHOTO_PLAN[j]:
                pose = [0.0] * NUM_JOINTS
                if j in (0, 1):
                    pose[0] = a if j == 0 else 15.0    # keep the thumb clear of the palm
                pose[j] = a
                t0 = time.monotonic()
                while time.monotonic() - t0 < 2.5:
                    hand.write_angles(pose); time.sleep(0.03)
                meas = hand.read_angles()
                print(f"  commanded {a:5.1f}   measured {meas[j]:5.1f}   -> photo name: {name}_{a:g}.jpg")
                input("  take the photo, then press Enter ...")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 1.5:
                hand.open_hand(); time.sleep(0.03)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        hand.disconnect()
    return 0
