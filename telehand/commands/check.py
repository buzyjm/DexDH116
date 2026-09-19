"""``check``: verify camera, tracking and the hand link without moving anything."""

from __future__ import annotations

import time
from pathlib import Path

import cv2

from .. import viz
from ..camera import Camera
from ..hand import JOINT_NAMES, Hand
from ..poses import PoseLibrary
from ..tracker import SIGNAL_NAMES, HandTracker


def run(args) -> int:
    """Verify camera, tracking and hand link without commanding any motion."""
    print("== camera ==")
    cap = Camera(args.camera)
    tracker = HandTracker(Path(args.model), prefer_hand=args.hand)
    print(f"  camera {cap.target} open")

    duration = 8.0 if not args.no_display else 5.0
    print(f"\n== tracking ({duration:.0f} s, hold your hand up) ==")
    if not args.no_display:
        print("  A preview window is open so you can check your framing.")
    start = time.monotonic()
    seen = 0
    frames = 0
    last = None
    while time.monotonic() - start < duration:
        ok, frame = cap.read()
        if not ok:
            continue
        frames += 1
        if args.mirror:
            frame = cv2.flip(frame, 1)
        elapsed = time.monotonic() - start
        reading = tracker.process(frame, int(elapsed * 1000))
        if reading is not None:
            seen += 1
            last = reading

        if args.no_display:
            continue
        viz.draw_skeleton(frame, reading.landmarks_2d if reading else None)
        rate = seen / max(frames, 1)
        cv2.putText(frame, f"detected {seen}/{frames} ({rate:4.0%})", (10, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (90, 200, 90) if reading else (60, 60, 235), 2, cv2.LINE_AA)
        cv2.putText(frame, f"{duration - elapsed:3.0f}s left - q to stop", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.imshow("telehand check", frame)
        if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
            break

    cap.release()
    tracker.close()
    if not args.no_display:
        cv2.destroyAllWindows()
    rate = seen / max(frames, 1)
    verdict = "good" if rate > 0.8 else "usable" if rate > 0.5 else "POOR"
    print(f"  {seen}/{frames} frames with a hand detected ({rate:.0%} - {verdict})")
    if last is not None:
        print(f"  handedness: {last.handedness}")
        if last.handedness != args.hand:
            print(f"\n  NOTE: tracking your {last.handedness} hand, but --hand is "
                  f"{args.hand}.")
            print("  MediaPipe labels handedness assuming a mirrored, selfie-style")
            print("  view, which --mirror (on by default) provides -- so the label")
            print(f"  matches reality: you held up your {last.handedness.lower()} hand.")
            print(f"  Either hold up your {args.hand.lower()} hand, or run with "
                  f"--hand {last.handedness}.")
            print("  The robot is a RIGHT hand. Driving it with your left still works")
            print("  (only per-finger curl is used, which has no handedness), but the")
            print("  thumb will feel mirrored.\n")
        print("  raw signals:")
        for name, value in zip(SIGNAL_NAMES, last.signals):
            print(f"    {name:<16} {value:7.1f}")
    else:
        print("  WARNING: no hand was detected - check lighting and framing")
    if 0 < rate <= 0.5:
        print("  Detection rate is low: more light on your hand, and keep the")
        print("  whole hand inside the frame.")

    print("\n== hand ==")
    with Hand(args.port, args.baud, dry_run=True) as hand:
        hand.connect(home=False)
        print(f"  connected on {args.port} @ {args.baud}")
        for i, (name, lim) in enumerate(zip(JOINT_NAMES, hand.limits)):
            print(f"    {i+1} {name:<16} {lim.min_angle:6.1f} .. {lim.max_angle:6.1f}")
        print(f"  measured angles: {[round(a,1) for a in hand.read_angles()]}")
        print(f"  alarms:          {hand.alarms()}")

    print("\n== pose library ==")
    library = PoseLibrary.load(Path(args.poses))
    done = library.complete_anchors
    print(f"  {args.poses}: {len(done)}/{len(library.anchors)} poses calibrated")
    for a in library.anchors:
        state = "ok" if a.complete else "NOT CALIBRATED"
        joints = "measured" if a.measured else "estimated joints"
        print(f"    {a.name:<14} {state:<15} {joints}")
    if len(done) < 2:
        print("  Run:  python3 run.py calibrate --hand " + args.hand)
    return 0
