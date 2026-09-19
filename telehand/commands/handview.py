"""``handview``: render the hand from its model at a pose; no camera or motors."""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple

import cv2

from ..handview import DEFAULT_PITCH, DEFAULT_YAW, HandRenderer
from ..poses import PoseLibrary


def run(args) -> int:
    """Show the hand rendered from its URDF at a pose. No camera, no motors."""
    renderer = HandRenderer(args.direction)
    poses: List[Tuple[str, List[float]]] = []
    if args.poses:
        lib = PoseLibrary.load(Path(args.poses))
        poses = [(a.name, list(a.joints)) for a in lib.anchors]
    elif args.pose:
        poses = [("pose", [float(v) for v in args.pose.split(",")])]
    else:
        poses = [("open", [0.0] * 6), ("fist", [60.0, 30.0, 80.0, 80.0, 80.0, 80.0])]

    yaw, pitch, i = DEFAULT_YAW, DEFAULT_PITCH, 0
    print("n/N pose    h/l yaw    j/k pitch    r reset view    q quit")
    while True:
        name, fw = poses[i % len(poses)]
        img = renderer.render(fw, size=(args.width, args.height), yaw=yaw, pitch=pitch)
        cv2.putText(img, f"{name}  {[round(v) for v in fw]}", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (210, 210, 210), 1, cv2.LINE_AA)
        cv2.putText(img, f"yaw {yaw:.0f}  pitch {pitch:.0f}", (8, args.height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (140, 138, 136), 1, cv2.LINE_AA)
        if args.out:
            cv2.imwrite(args.out, img)
            print(f"wrote {args.out}")
            return 0
        cv2.imshow("handview", img)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            break
        elif key == ord("n"):
            i += 1
        elif key == ord("N"):
            i -= 1
        elif key == ord("h"):
            yaw -= 5
        elif key == ord("l"):
            yaw += 5
        elif key == ord("j"):
            pitch -= 5
        elif key == ord("k"):
            pitch += 5
        elif key == ord("r"):
            yaw, pitch = DEFAULT_YAW, DEFAULT_PITCH
    cv2.destroyAllWindows()
    return 0
