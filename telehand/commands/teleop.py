"""``teleop``: live teleoperation in any of the four retargeting modes."""

from __future__ import annotations

import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from .. import direct as _direct
from .. import profile as _profile
from .. import viz
from ..camera import Camera
from ..fkretarget import FKRetargeter, PinchCloser
from ..hand import JOINT_NAMES, NUM_JOINTS, Hand
from ..poses import PoseLibrary, PoseRetargeter
from ..tracker import SIGNAL_NAMES, HandTracker


def run(args) -> int:
    mode = args.mode
    library = None
    if mode in ("poses", "hybrid"):
        library = PoseLibrary.load(Path(args.poses))
        if not args.allow_estimates:
            skipped = [a.name for a in library.anchors if a.complete and not a.measured]
            if skipped:
                print("Excluding poses whose robot joints were never measured: "
                      + ", ".join(skipped) + "  (measure them with `run.py jog`, "
                      "or pass --allow-estimates)")
                library.anchors = [a for a in library.anchors if a.measured]
        ready = library.complete_anchors
        if len(ready) < 2:
            print(f"ERROR: {args.poses} has fewer than two calibrated poses.")
            print("Run:  python3 run.py calibrate --hand " + args.hand)
            return 1
        print(f"Pose library: {len(ready)} anchors - " + ", ".join(a.name for a in ready))
        estimates = [a.name for a in ready if not a.measured]
        if estimates:
            print("NOTE: robot joints are estimates for " + ", ".join(estimates)
                  + "; measure them with `run.py jog` if those poses feel off.")
    direct_calib = None
    if mode in ("direct", "hybrid"):
        direct_calib = _direct.DirectCalibration.load(Path(args.direct_calibration))
        state = ("calibrated" if direct_calib.calibrated
                 else "DEFAULT ranges - press 1 (open) / 2 (fist) to capture")
        print(f"Direct mode ranges: {state}")
    print(f"Mode: {mode}")

    cap = Camera(args.camera)
    tracker = HandTracker(Path(args.model), prefer_hand=args.hand)

    hand = Hand(args.port, args.baud, dry_run=args.dry_run,
                keep_enabled=args.keep_enabled, stall_kick=args.stall_kick,
                max_deg_per_s=args.max_speed, max_current=args.max_current)

    print(f"Connecting to hand on {args.port} ...")
    if args.home and not args.dry_run:
        print("Homing: the hand will move through its full range. Keep it clear.")
    hand.connect(home=args.home)
    print("Connected.")
    if args.dry_run:
        print("DRY RUN: tracking runs but no motion is commanded.")

    pose_rt = None
    if library is not None:
        pose_rt = PoseRetargeter(hand.limits, library, smoothing=args.smoothing,
                                 deadband_deg=args.deadband, power=args.blend_power)
    prof = _profile.HandProfile.load(Path(args.profile)) if mode == "fk" else None
    if mode == "fk":
        print("Hand profile: " + (f"{args.profile}" + (f" ({prof.name})" if prof and prof.name else "")
                                 if prof else "none - run `run.py profile` for per-finger scaling"))
    fk_rt = FKRetargeter(hand.limits, smoothing=args.smoothing, deadband_deg=args.deadband,
                         profile=prof) if mode == "fk" else None
    intent = (prof.pinch_gap * 1000 + 12.0) if (prof and prof.pinch_gap) else 35.0
    closer = PinchCloser(intent_mm=intent) if (mode == "fk" and args.tactile and not args.dry_run) else None
    if closer:
        hand._sdk.set_sensor_enable(True); time.sleep(0.8); hand._sdk.set_finger_pressure_reset()
        print("Tactile pinch closure: ON (thumb-tip sensor + finger stall)")
    direct_rt = smoother = capture = None
    if direct_calib is not None:
        direct_rt = _direct.DirectRetargeter(hand.limits, direct_calib)
        smoother = _direct.LandmarkSmoother(alpha=args.lm_alpha, deadband=args.lm_deadband,
                                            use_kalman=not args.no_kalman)
        capture = _direct.RangeCapture()

    def reset_filters():
        if pose_rt: pose_rt.reset()
        if fk_rt: fk_rt.reset()
        if closer: closer.reset()
        if smoother: smoother.reset()

    diag = {"nearest": None, "dist": None, "top": []}
    rec = open(args.record, "w") if args.record else None
    rec_pts = [] if args.record else None
    if rec:
        rec.write("t," + ",".join(SIGNAL_NAMES) + ",nearest,dist," + ",".join(JOINT_NAMES) + "\n")

    def compute_angles(reading, frame_shape):
        """Joint angles for the current mode; None if this mode has nothing yet."""
        pose_angles = pose_rt(reading.signals) if pose_rt else None
        if rec_pts is not None and reading.world_pts is not None:
            rec_pts.append((time.monotonic() - t0, reading.handedness, reading.world_pts.copy()))
        if pose_rt:
            w = pose_rt.weights(reading.signals)
            order = np.argsort(w)[::-1]
            sig = (np.asarray(reading.signals) - pose_rt._lo) / pose_rt._span
            dists = np.linalg.norm(pose_rt._norm - sig, axis=1)
            k = int(np.argmin(dists))
            diag["nearest"], diag["dist"] = pose_rt.names[k], float(dists[k])
            diag["top"] = [(pose_rt.names[i], float(w[i])) for i in order[:2]]
            if rec and pose_angles is not None:
                rec.write(f"{time.monotonic()-t0:.3f}," + ",".join(f"{v:.2f}" for v in reading.signals)
                          + f",{diag['nearest']},{diag['dist']:.3f},"
                          + ",".join(f"{v:.1f}" for v in pose_angles) + "\n")
        direct_angles = None
        if direct_rt and reading.landmarks_2d is not None:
            h, w = frame_shape[:2]
            norm = reading.landmarks_2d / np.array([w, h], dtype=np.float64)
            sm = smoother(norm)
            sig = _direct.bends_to_signals(_direct.finger_bends(sm))
            done = capture.feed(sig, direct_calib)
            if done:
                direct_calib.save(Path(args.direct_calibration))
                print(f"direct: captured {done} pose -> {args.direct_calibration}")
            direct_angles = direct_rt(sig)
        if mode == "fk":
            if reading.world_pts is None:
                return None
            # Chirality decides whether the palm normal is mirrored. Trust the
            # operator's --hand over MediaPipe's label, which has disagreed with
            # the operator in every session so far.
            # The frame is mirrored for display, so the landmark geometry is of
            # the opposite hand: a physical right hand arrives as a left one.
            geom = args.hand if not args.mirror else ("Left" if args.hand == "Right" else "Right")
            out = list(fk_rt(reading.world_pts, geom))
            # The thumb abduction joint is slow to let go. While it still sits
            # more than a few degrees across from where it was sent, the fingers
            # hold their current curl instead of closing into its path.
            if not args.dry_run:
                meas = hand.read_angles()
                if meas[0] > out[0] + 6.0:
                    for i in range(2, 6):
                        out[i] = min(out[i], max(meas[i], 0.0))
                    diag["top"] = [("thumb clearing", 1.0)]
            if closer:
                tgt, _ = fk_rt.targets(np.asarray(reading.world_pts, dtype=np.float64), geom)
                dist_mm = [float(np.linalg.norm(v)) * 1000 for v in tgt[:4]]
                try:
                    pressure = max(hand._sdk.get_finger_pressure(1) or [0.0])
                except Exception:
                    pressure = 0.0
                sent = list(out)
                for i in range(4):
                    sent[2 + i] = hand.limits[2 + i].clamp(out[2 + i] + closer.extra[i])
                extra = closer.update(dist_mm, sent, hand.read_angles(), pressure)
                for i in range(4):
                    out[2 + i] = hand.limits[2 + i].clamp(out[2 + i] + extra[i])
                diag["nearest"] = "pinch:" + ",".join(
                    f"{'*' if closer.contact[i] else ''}{extra[i]:.0f}" for i in range(4))
                diag["dist"] = min(dist_mm) / 100.0
                diag["top"] = [(f"p={pressure:.2f}", 1.0)]
            return out
        if mode == "poses":
            return pose_angles
        if mode == "direct":
            return direct_angles
        # hybrid: vendor-style linear map for the four fingers, our pinch-aware
        # pose blend for the two thumb joints.
        if direct_angles is None or pose_angles is None:
            return direct_angles or pose_angles
        return pose_angles[:2] + direct_angles[2:]

    view = viz.ViewControl()
    engaged = False
    lost_frames = 0
    alarm_state = [0] * NUM_JOINTS
    frame_no = 0
    mouse_attached = False
    warned_handedness = False
    t0 = time.monotonic()
    last_t = t0
    fps = 0.0
    angles: Optional[List[float]] = None
    headless = args.no_display

    if headless:
        engaged = True
        print("Headless mode: engaging immediately. Ctrl+C to stop.")
    else:
        print("Press SPACE in the video window to engage.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            if args.mirror:
                frame = cv2.flip(frame, 1)

            now = time.monotonic()
            dt = now - last_t
            last_t = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            reading = tracker.process(frame, int((now - t0) * 1000))

            # Alarms come over the same RS485 bus as the motion commands, so
            # poll them a few times a second rather than every frame.
            frame_no += 1
            if frame_no % 15 == 0:
                alarm_state = hand.alarms()

            if reading is not None:
                if not warned_handedness and reading.handedness != args.hand:
                    print(f"NOTE: tracking your {reading.handedness} hand "
                          f"(--hand is {args.hand}). Run `check` for details.")
                    warned_handedness = True
                lost_frames = 0
                new_angles = compute_angles(reading, frame.shape)
                if new_angles is not None:
                    angles = new_angles
                    if engaged:
                        angles = hand.write_angles(angles)
            else:
                lost_frames += 1
                # Tracking dropped out: hold the last commanded pose rather than
                # letting the hand snap anywhere.
                if lost_frames > args.lost_timeout_frames:
                    reset_filters()

            if headless:
                continue

            viz.draw_skeleton(frame, reading.landmarks_2d if reading else None)
            if args.diag and diag["nearest"]:
                top = "  ".join(f"{n} {w:.0%}" for n, w in diag["top"])
                txt = f"nearest {diag['nearest']}  d={diag['dist']:.2f}   {top}"
                cv2.rectangle(frame, (0, frame.shape[0] - 26), (frame.shape[1], frame.shape[0]),
                              (30, 28, 26), -1)
                cv2.putText(frame, txt, (8, frame.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (90, 200, 90) if diag["dist"] < 0.35 else (60, 170, 235), 1, cv2.LINE_AA)
            panel = viz.render_panel(
                frame.shape[0],
                angles if angles is not None else [l.min_angle for l in hand.limits],
                hand.limits,
                engaged=engaged, fps=fps, alarms=alarm_state,
                tracked=reading is not None, dry_run=args.dry_run,
                title=f"{'ENGAGED' if engaged else 'PAUSED'}  [{mode}]",
                measured=(hand.read_angles() if args.show_measured and not args.dry_run else None),
                **view.kwargs,
            )
            composed = viz.compose(frame, panel)
            cv2.imshow("telehand", composed)
            if not mouse_attached:
                view.camera_width = frame.shape[1]
                view.attach("telehand")
                mouse_attached = True
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == 27:
                break
            elif key == ord(" "):
                engaged = not engaged
                reset_filters()
                print("ENGAGED" if engaged else "PAUSED")
            elif key == ord("o"):
                engaged = False
                hand.open_hand()
                print("opened hand, disengaged")
            elif key == ord("c"):
                hand.clear_alarms()
                print("alarms cleared")
            elif view.handle_key(key):
                pass
            elif key in (ord("1"), ord("2")) and capture is not None:
                which = key - ord("1")
                capture.start(which)
                which_pose = "OPEN hand" if which == 0 else "FIST"
                print(f"direct: hold the {which_pose} still, averaging 20 frames ...")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        print("Stopping...")
        cap.release()
        tracker.close()
        if rec:
            rec.close()
            print(f"signals recorded to {args.record}")
            if rec_pts:
                npz = Path(args.record).with_suffix(".npz")
                np.savez_compressed(npz, t=np.array([p[0] for p in rec_pts]),
                                    handedness=np.array([p[1] for p in rec_pts]),
                                    world_pts=np.stack([p[2] for p in rec_pts]))
                print(f"world landmarks recorded to {npz} ({len(rec_pts)} frames)")
        if not headless:
            cv2.destroyAllWindows()
        hand.disconnect()
        print("Disconnected.")
    return 0
