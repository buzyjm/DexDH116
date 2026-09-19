"""``calibrate``: record the tracker signals for every pose in the library."""

from __future__ import annotations

import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from .. import viz
from ..camera import Camera
from ..poses import POSE_PLAN, PoseLibrary
from ..tracker import SIGNAL_NAMES, HandTracker


def run(args) -> int:
    """Record the tracker signals for every pose in the library."""
    library = PoseLibrary.load(Path(args.poses))
    estimates = [a.name for a in library.anchors if not a.measured]
    print(f"Pose library: {args.poses}")
    for a in library.anchors:
        mark = "measured" if a.measured else "ESTIMATED joints"
        print(f"  {a.name:<14} {[round(v) for v in a.joints]}  ({mark})")
    if estimates:
        print("\nNOTE: robot joint angles for " + ", ".join(estimates) +
              " are estimates.\n  Measure them with `run.py jog` for a better"
              " match; calibration still works meanwhile.")
    print()

    cap = Camera(args.camera)
    tracker = HandTracker(Path(args.model), prefer_hand=args.hand)
    headless = args.no_display

    # One timebase for the whole session. MediaPipe's VIDEO mode requires
    # timestamps to increase monotonically across the landmarker's lifetime,
    # not just within a single capture, so this must not be reset per pose.
    t0 = time.monotonic()

    def settle(message: str, seconds: float = 2.5) -> None:
        """Countdown between poses, and drain the key queue while doing it.

        Holding SPACE generates autorepeat events that otherwise carry over and
        fire the next capture before you have changed pose -- which silently
        records the same hand shape twice.
        """
        if headless:
            return
        end_at = time.monotonic() + seconds
        while time.monotonic() < end_at:
            ok, frame = cap.read()
            if not ok:
                continue
            if args.mirror:
                frame = cv2.flip(frame, 1)
            h, w = frame.shape[:2]
            left = end_at - time.monotonic()
            cv2.rectangle(frame, (0, 0), (w, 34), (40, 38, 36), -1)
            cv2.putText(frame, f"{message}  ({left:.0f})", (10, 23),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (60, 170, 235), 2, cv2.LINE_AA)
            cv2.imshow("calibrate", frame)
            cv2.waitKey(1)  # discarded on purpose

    def capture(step: str, description: str) -> Optional[List[float]]:
        print(f"\n{step}: {description}")
        print("  Press SPACE to capture (q to abort). Averaging 30 frames.")
        samples: List[List[float]] = []
        collecting = False
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            if args.mirror:
                frame = cv2.flip(frame, 1)
            reading = tracker.process(frame, int((time.monotonic() - t0) * 1000))

            if collecting and reading is not None:
                samples.append(reading.signals)
                if len(samples) >= 30:
                    return [float(np.mean(c)) for c in zip(*samples)]

            if headless:
                if reading is not None:
                    collecting = True
                continue

            h, w = frame.shape[:2]
            viz.draw_skeleton(frame, reading.landmarks_2d if reading else None)
            cv2.rectangle(frame, (0, 0), (w, 34), (40, 38, 36), -1)
            cv2.putText(frame, f"{step}  {description}", (10, 23),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (90, 200, 90) if collecting else (255, 255, 255), 1, cv2.LINE_AA)

            if collecting:
                cv2.rectangle(frame, (10, h - 30), (w - 10, h - 16), (60, 58, 56), -1)
                fill = int((w - 20) * len(samples) / 30.0)
                cv2.rectangle(frame, (10, h - 30), (10 + fill, h - 16), (90, 200, 90), -1)
                cv2.putText(frame, f"capturing {len(samples)}/30", (10, h - 38),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (90, 200, 90), 1, cv2.LINE_AA)
            else:
                hint = "SPACE = capture     q = abort"
                if reading is None:
                    hint = "no hand detected - show your hand to the camera"
                cv2.putText(frame, hint, (10, h - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (200, 200, 200) if reading else (60, 60, 235), 1, cv2.LINE_AA)

            if reading is not None:
                for i, (nm, val) in enumerate(zip(SIGNAL_NAMES, reading.signals)):
                    ty = 56 + i * 18
                    for colour, thick in (((255, 255, 255), 2), ((40, 40, 40), 1)):
                        cv2.putText(frame, f"{nm:<16}{val:7.1f}", (w - 220, ty),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, thick,
                                    cv2.LINE_AA)
            cv2.imshow("calibrate", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                return None
            if key == ord(" "):
                collecting = True

    plan = {name: desc for name, desc in POSE_PLAN}
    targets = library.anchors if args.all else [a for a in library.anchors if not a.complete]
    if not targets:
        print("Every pose already has signals. Use --all to re-record all of them.")
        return 0
    if not args.all and len(targets) < len(library.anchors):
        print(f"Recording only the {len(targets)} pose(s) without signals: "
              + ", ".join(a.name for a in targets) + "  (--all re-records everything)\n")
    total = len(targets)
    try:
        for i, anchor in enumerate(targets, start=1):
            description = plan.get(anchor.name, anchor.name.replace("_", " "))
            if i > 1:
                settle(f"GET READY - next: {description}")
            signals = capture(f"{i}/{total} {anchor.name}", description)
            if signals is None:
                print("aborted - nothing saved")
                return 1
            anchor.signals = signals
    finally:
        cap.release()
        tracker.close()
        if not headless:
            cv2.destroyAllWindows()

    print()
    header = f"{'pose':<14}" + "".join(f"{n[:8]:>9}" for n in SIGNAL_NAMES)
    print(header)
    for anchor in library.anchors:
        print(f"{anchor.name:<14}" + "".join(f"{v:>9.1f}" for v in anchor.signals))

    # Two anchors that read almost the same give the blend nothing to separate,
    # so the robot cannot tell those poses apart either.
    problems = []
    for i, a in enumerate(library.anchors):
        for b in library.anchors[i + 1:]:
            d = max(abs(x - y) for x, y in zip(a.signals, b.signals))
            if d < 10.0:
                problems.append(f"{a.name} vs {b.name} (max diff {d:.1f})")
    if problems:
        print("\nNOT SAVED - these poses look too alike to the tracker:")
        for line in problems:
            print(f"  {line}")
        print("  Make each pose more distinct and re-run. Watch the live numbers:")
        print("  they should change clearly between poses.")
        return 1

    library.save(Path(args.poses))
    print(f"\nSaved {args.poses}")
    print(f"{len(library.anchors)} anchors calibrated.")
    return 0
