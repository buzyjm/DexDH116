"""``jog``: drive individual joints by hand and save measured poses."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from .. import viz
from ..hand import JOINT_NAMES, NUM_JOINTS, Hand
from ..poses import PoseLibrary


def run(args) -> int:
    """Drive individual joints by hand, to learn what each one physically does.

    Camera-free: this is the tool for answering questions like "which direction
    does thumb abduction actually move?" without a tracker in the loop.
    """
    hand = Hand(args.port, args.baud, dry_run=args.dry_run,
                keep_enabled=args.keep_enabled, stall_kick=args.stall_kick,
                max_deg_per_s=args.max_speed, max_current=args.max_current)
    print(f"Connecting to hand on {args.port} ...")
    if args.home and not args.dry_run:
        print("Homing: the hand will move through its full range. Keep it clear.")
    hand.connect(home=args.home)
    print("Connected.\n")

    library = PoseLibrary.load(Path(args.poses))
    view = viz.ViewControl(camera_width=300)   # jog composes a 300 px side panel
    jog_mouse_attached = False
    angles = [lim.min_angle for lim in hand.limits]
    hand.write_angles(angles)
    selected = 0
    step = 5.0
    saved = {}

    print("Poses to measure (n / N steps through them, loading each as a start):")
    for anchor in library.anchors:
        mark = "measured" if anchor.measured else "ESTIMATE - please measure"
        print(f"  {anchor.name:<14} {[round(v) for v in anchor.joints]}  ({mark})")
    print()
    plan_names = [a.name for a in library.anchors]
    todo = [i for i, a in enumerate(library.anchors) if not a.measured]
    plan_at = todo[0] if todo else 0
    if todo:
        print(f"{len(todo)} pose(s) still estimated: " + ", ".join(plan_names[i] for i in todo))
        print(f"Starting on {plan_names[plan_at]}. Adjust, press s, then n for the next.\n")
    angles = list(library.anchors[plan_at].joints)
    hand.write_angles(angles)   # go there now rather than on the first keypress

    print("Keys (click the window first):")
    print("  1-6      select joint")
    print("  , / .    jog selected joint down / up")
    print("  < / >    jog by 1 degree")
    print("  [ / ]    change step size")
    print("  n / N    load the next / previous planned pose as a starting point")
    print("  o / f    all joints to min / max")
    print("  p        print the current pose")
    print("  s        save this pose under the name shown in the window")
    print("  q        quit\n")

    keys_help = ["1-6 select  ,/. jog", "[/] step   n/N pose",
                 "o open  f close", "p print   s save", "",
                 "hjkl rotate view", "v  save view", "", "q quit"]
    try:
        while True:
            shown = hand.read_angles() if not args.dry_run else angles
            panel = viz.render_panel(
                480, angles, hand.limits, engaged=True, fps=0.0,
                alarms=hand.alarms() if not args.dry_run else [0] * NUM_JOINTS,
                tracked=True, dry_run=args.dry_run, selected=selected,
                title=f"JOG  {plan_names[plan_at]}", keys=keys_help, **view.kwargs)
            side = np.full((480, 300, 3), (26, 24, 22), np.uint8)
            cv2.putText(side, f"EDITING   {plan_names[plan_at]}", (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 170, 235), 1, cv2.LINE_AA)
            cv2.putText(side, f"joint {selected+1} {JOINT_NAMES[selected]}  step {step:g}",
                        (12, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (140, 138, 136), 1,
                        cv2.LINE_AA)
            cv2.putText(side, "s = save under this name", (12, 66),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (90, 200, 90) if plan_names[plan_at] in saved else (140, 138, 136),
                        1, cv2.LINE_AA)
            cv2.putText(side, "MEASURED", (12, 96), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (140, 138, 136), 1, cv2.LINE_AA)
            for i, name in enumerate(JOINT_NAMES):
                cv2.putText(side, f"{name:<16}{shown[i]:6.1f}", (12, 124 + i * 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (235, 235, 235), 1,
                            cv2.LINE_AA)
            cv2.putText(side, "SAVED THIS SESSION", (12, 268),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 138, 136), 1, cv2.LINE_AA)
            for i, (nm, vals) in enumerate(saved.items()):
                cv2.putText(side, f"{nm}: {[round(v) for v in vals]}",
                            (12, 292 + i * 18), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                            (90, 200, 90), 1, cv2.LINE_AA)
            cv2.imshow("telehand jog", viz.compose(side, panel))
            if not jog_mouse_attached:
                view.camera_width = side.shape[1]
                view.attach("telehand jog")
                jog_mouse_attached = True

            key = cv2.waitKey(30) & 0xFF
            if key == 255:
                continue
            if key in (ord("q"), 27):
                break
            elif ord("1") <= key <= ord("6"):
                selected = key - ord("1")
            elif key == ord(","):
                angles[selected] -= step
            elif key == ord("."):
                angles[selected] += step
            elif key == ord("<"):
                angles[selected] -= 1.0
            elif key == ord(">"):
                angles[selected] += 1.0
            elif key == ord("["):
                step = max(1.0, step / 2)
            elif key == ord("]"):
                step = min(20.0, step * 2)
            elif key == ord("o"):
                angles = [l.min_angle for l in hand.limits]
            elif key == ord("f"):
                angles = [l.max_angle for l in hand.limits]
            elif key == ord("p"):
                print(f"pose: {[round(a, 1) for a in angles]}")
            elif key in (ord("h"), ord("l"), ord("j"), ord("k"), ord("v")) and view.handle_key(key):
                pass
            elif key == ord("n"):
                plan_at = (plan_at + 1) % len(plan_names)
                angles = list(library.get(plan_names[plan_at]).joints)
                print(f"loaded {plan_names[plan_at]}: {[round(a) for a in angles]}")
            elif key == ord("N"):
                plan_at = (plan_at - 1) % len(plan_names)
                angles = list(library.get(plan_names[plan_at]).joints)
                print(f"loaded {plan_names[plan_at]}: {[round(a) for a in angles]}")
            elif key == ord("s"):
                # Named by whichever planned pose is selected, never by typing.
                # Reading a name from stdin here let window keystrokes leak into
                # the prompt and saved poses under junk names like "p" and "s".
                name = plan_names[plan_at]
                saved[name] = list(angles)
                library.set_joints(name, angles, measured=True)
                library.save(Path(args.poses))
                print(f"saved {name} = {[round(a, 1) for a in angles]}")
            angles = hand.write_angles(angles)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        cv2.destroyAllWindows()
        hand.disconnect()

    if saved:
        print(f"\nSaved to {args.poses}:")
        for nm, vals in saved.items():
            print(f"  {nm:<14} {[round(v, 1) for v in vals]}")
        print("\nNext: python3 run.py calibrate --hand " + args.hand)
    return 0
