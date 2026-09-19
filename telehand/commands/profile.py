"""``profile``: measure the operator's hand for the kinematic retargeter."""

from __future__ import annotations

import time
from pathlib import Path

import cv2

from .. import profile as _profile
from .. import viz
from ..camera import Camera
from ..tracker import HandTracker


def run(args) -> int:
    """Measure this operator's hand: finger lengths from an open hand, and the
    tracker's residual gap in a real pinch. No motors involved."""
    cap = Camera(args.camera)
    tracker = HandTracker(Path(args.model), prefer_hand=args.hand)
    t0 = time.monotonic()
    headless = args.no_display

    def capture(label: str, seconds: float = 2.0):
        print(f"\n{label}")
        print("  Press SPACE when ready; frames are collected for %.0f s." % seconds)
        frames = []; collecting = False; t_end = None
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            if args.mirror:
                frame = cv2.flip(frame, 1)
            reading = tracker.process(frame, int((time.monotonic() - t0) * 1000))
            if collecting and reading is not None and reading.world_pts is not None:
                frames.append(reading.world_pts.copy())
                if time.monotonic() >= t_end:
                    return frames
            if headless:
                if reading is not None and not collecting:
                    collecting, t_end = True, time.monotonic() + seconds
                continue
            viz.draw_skeleton(frame, reading.landmarks_2d if reading else None)
            msg = f"collecting {len(frames)}" if collecting else label + "   [SPACE]"
            cv2.putText(frame, msg, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (90, 200, 90) if collecting else (255, 255, 255), 2, cv2.LINE_AA)
            cv2.imshow("profile", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                return None
            if key == ord(" ") and not collecting and reading is not None:
                collecting, t_end = True, time.monotonic() + seconds

    try:
        open_frames = capture("1/2  OPEN hand, flat, fingers straight and spread")
        if open_frames is None:
            print("aborted"); return 1
        pinch_frames = capture("2/2  PINCH: thumb tip pressed against index tip")
        if pinch_frames is None:
            print("aborted"); return 1
    finally:
        cap.release(); tracker.close()
        if not headless:
            cv2.destroyAllWindows()
    prof = _profile.build(open_frames, pinch_frames, name=args.name)
    prof.save(Path(args.out))
    print(f"\nSaved {args.out}\n" + prof.describe())
    return 0
