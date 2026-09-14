"""Camera hand-tracking teleoperation for the LHandPro dexterous hand."""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time

import numpy as np
from pathlib import Path
from typing import List, Optional, Tuple

from .hand import Hand, JOINT_NAMES, NUM_JOINTS, DEFAULT_PORT, DEFAULT_BAUD
from .tracker import HandTracker, SIGNAL_NAMES, DEFAULT_MODEL
from . import viz
from .poses import (PoseLibrary, PoseRetargeter, POSE_PLAN,
                     DEFAULT_POSE_FILE)
from . import session
from . import touchscan as _touchscan
from .fkretarget import FKRetargeter, PinchCloser
from . import profile as _profile
from . import direct as _direct

# Minimum open-to-fist separation per signal for a calibration to be usable.
# The thumb travels far less than the fingers, so it gets a lower bar.
MIN_SEPARATION = [6.0, 12.0, 30.0, 30.0, 30.0, 30.0]


CAMERA_BY_PATH = Path("/dev/v4l/by-path")
CAMERA_CHOICE = Path(__file__).resolve().parents[1] / ".camera.json"


def saved_camera():
    """The camera chosen with `run.py cameras --use`, if it is still present."""
    try:
        spec = json.loads(CAMERA_CHOICE.read_text())["camera"]
    except Exception:
        return None
    if str(spec).startswith("/dev/") and not Path(spec).exists():
        print(f"saved camera {spec} is not connected; falling back")
        return None
    return spec


def list_cameras():
    """Every capture device, with the path that survives a reboot or replug.

    Device numbers shuffle when cameras are plugged in a different order, and
    two identical models share a serial, so `by-id` cannot tell them apart.
    `by-path` is keyed on the USB port and stays unique.
    """
    out = []
    for node in sorted(Path("/dev").glob("video*"), key=lambda p: int(p.name[5:])):
        sysdir = Path(f"/sys/class/video4linux/{node.name}")
        # Each camera exposes several nodes; index 0 is the one that captures.
        # Reading /sys instead of probing with OpenCV keeps this quiet and
        # instant -- opening a metadata node logs a warning and wastes a second.
        try:
            if (sysdir / "index").read_text().strip() != "0":
                continue
        except OSError:
            continue
        stable = None
        if CAMERA_BY_PATH.is_dir():
            for link in sorted(CAMERA_BY_PATH.iterdir()):
                if link.resolve() == node.resolve() and "index0" in link.name:
                    stable = str(link)
                    break
        name = ""
        info = sysdir / "name"
        if info.exists():
            name = info.read_text().strip()
        out.append({"node": str(node), "index": int(node.name[5:]),
                    "name": name, "by_path": stable})
    return out


def resolve_camera(spec):
    """Accept an index, a /dev node, a by-path link, or a name fragment."""
    if spec in (None, "", "auto"):
        spec = saved_camera()
        if spec is None:
            cams = list_cameras()
            if not cams:
                raise RuntimeError("no capture devices found")
            spec = cams[0]["node"]
    if isinstance(spec, int):
        return spec
    text = str(spec)
    if text.isdigit():
        return int(text)
    if text.startswith("/dev/"):
        return text
    matches = [c for c in list_cameras() if text.lower() in c["name"].lower()]
    if len(matches) == 1:
        return matches[0]["node"]
    if len(matches) > 1:
        raise RuntimeError(f"{text!r} matches {len(matches)} cameras; use a by-path from "
                           "`run.py cameras`")
    raise RuntimeError(f"no camera matches {text!r}; see `run.py cameras`")


class Camera:
    """V4L2 capture that always hands back the newest frame.

    Two things matter for teleop latency and neither is the default:

    * ``CAP_PROP_BUFFERSIZE = 1`` looks like the way to avoid stale frames, but
      with the V4L2 backend it halves the delivered rate (measured 15 fps on
      both cameras here, against 30 without it). So we leave the queue alone.
    * With a normal queue, a slow consumer falls behind and you end up
      teleoperating from frames that are several hundred ms old. A reader
      thread that keeps only the most recent frame fixes that instead.
    """

    def __init__(self, index, width: int = 640, height: int = 480, fps: int = 30,
                 settle: float = 1.5):
        import cv2

        self._cv2 = cv2
        target = resolve_camera(index)
        cap = cv2.VideoCapture(target, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"Could not open camera {index!r} (resolved to {target!r})")
        # FOURCC must be set before the frame size to take effect.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)

        self._cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0
        self._last_seq = 0
        self._running = True
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()
        # Auto-exposure needs a moment: the first frames off a freshly opened
        # camera come back blown out or black, which reads as a broken device.
        deadline = time.monotonic() + settle
        while time.monotonic() < deadline:
            with self._lock:
                if self._seq > 8:
                    break
            time.sleep(0.02)
        self._last_seq = self._seq

    def _pump(self) -> None:
        while self._running:
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame = frame
                self._seq += 1

    def read(self, timeout: float = 1.0):
        """Block until a frame newer than the last one returned is available."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._frame is not None and self._seq != self._last_seq:
                    self._last_seq = self._seq
                    return True, self._frame.copy()
            time.sleep(0.002)
        return False, None

    def release(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._cap.release()


# --------------------------------------------------------------------- checks

def cmd_check(args) -> int:
    """Verify camera, tracking and hand link without commanding any motion."""
    import cv2

    print("== camera ==")
    cap = Camera(args.camera)
    tracker = HandTracker(Path(args.model), prefer_hand=args.hand)
    print(f"  /dev/video{args.camera} open")

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
        cv2.imshow("DexDH116 check", frame)
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


# ---------------------------------------------------------------- calibration

def cmd_calibrate(args) -> int:
    """Record the tracker signals for every pose in the library."""
    import cv2
    import numpy as np

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


# --------------------------------------------------------------------- teleop

def cmd_teleop(args) -> int:
    import cv2

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
        state = "calibrated" if direct_calib.calibrated else "DEFAULT ranges - press 1 (open) / 2 (fist) to capture"
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
                          + f",{diag['nearest']},{diag['dist']:.3f}," + ",".join(f"{v:.1f}" for v in pose_angles) + "\n")
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
                diag["nearest"] = "pinch:" + ",".join(f"{'*' if closer.contact[i] else ''}{extra[i]:.0f}" for i in range(4))
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
                cv2.rectangle(frame, (0, frame.shape[0] - 26), (frame.shape[1], frame.shape[0]), (30, 28, 26), -1)
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
            cv2.imshow("DexDH116", composed)
            if not mouse_attached:
                view.camera_width = frame.shape[1]
                view.attach("DexDH116")
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
                print(f"direct: hold the {'OPEN hand' if which == 0 else 'FIST'} still, averaging 20 frames ...")
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



# ------------------------------------------------------------------------ jog

def cmd_jog(args) -> int:
    """Drive individual joints by hand, to learn what each one physically does.

    Camera-free: this is the tool for answering questions like "which direction
    does thumb abduction actually move?" without a tracker in the loop.
    """
    import cv2
    import numpy as np

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
            cv2.imshow("DexDH116 jog", viz.compose(side, panel))
            if not jog_mouse_attached:
                view.camera_width = side.shape[1]
                view.attach("DexDH116 jog")
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


# ------------------------------------------------------------------- touchscan

def cmd_touchscan(args) -> int:
    """Sweep fingers onto the thumb and record every contact the sensors feel."""
    fingers = [f.strip() for f in args.fingers.split(",") if f.strip()]
    abd = [float(v) for v in args.abd.split(",")]
    flex = [float(v) for v in args.flex.split(",")]
    out = Path(args.out)

    hand = Hand(args.port, args.baud, max_current=args.max_current,
                angular_velocity=args.speed, max_deg_per_s=args.speed)
    print(f"Connecting to hand on {args.port} ...")
    print("Homing: the hand will move through its full range. Keep it clear.")
    hand.connect(home=True)
    print(f"Connected. Current limit {args.max_current} permille, {args.speed} deg/s.")
    print(f"Scan: fingers={fingers}  thumb_abd={abd}  thumb_flex={flex}  "
          f"-> {len(fingers)*len(abd)*len(flex)} sweeps, saving to {out}\n")
    try:
        contacts = _touchscan.run_scan(hand, fingers, abd, flex, out,
                                       threshold=args.threshold,
                                       step_deg=args.step, dwell_s=args.dwell)
    except KeyboardInterrupt:
        print("\ninterrupted - partial results are saved")
    finally:
        hand.disconnect()
    print()
    print(_touchscan.summarize(out))
    return 0


# ------------------------------------------------------------------- photostep

PHOTO_PLAN = {
    # joint index -> firmware angles to photograph at
    2: [0, 20, 40, 60, 80],        # index flexion, side view
    1: [0, 7.5, 15, 22.5, 30],     # thumb flexion, side view
    0: [0, 30, 60],                # thumb abduction, view from above the palm
}


def cmd_photostep(args) -> int:
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


# ------------------------------------------------------------------- cameras

def cmd_cameras(args) -> int:
    """List capture devices, with a measured frame rate for each."""
    import cv2

    cams = list_cameras()
    if not cams:
        print("No capture devices found.")
        return 1
    if args.use is not None:
        node = resolve_camera(args.use)
        chosen = next((c for c in cams if c["node"] == str(node)
                       or c["index"] == node), None)
        # Prefer the by-path link: device numbers move, USB ports do not.
        spec = (chosen or {}).get("by_path") or str(node)
        CAMERA_CHOICE.write_text(json.dumps({"camera": spec}, indent=2) + "\n")
        print(f"default camera set to {spec}")
        if chosen:
            print(f"  ({chosen['name']}, currently {chosen['node']})")
        return 0
    current = saved_camera()
    if current:
        print(f"saved default: {current}\n")
    print(f"{'index':>5}  {'node':<13}{'name':<34}measured")
    for c in cams:
        fps = brightness = float("nan")
        try:
            cam = Camera(c["node"], settle=args.settle)
            t0 = time.monotonic()
            n, frame = 0, None
            while n < 45 and time.monotonic() - t0 < 4.0:
                ok, f = cam.read()
                if ok:
                    n += 1
                    frame = f
            fps = n / max(time.monotonic() - t0, 1e-6)
            brightness = float(frame.mean()) if frame is not None else float("nan")
            cam.release()
        except Exception as exc:
            print(f"{c['index']:>5}  {c['node']:<13}{c['name']:<34}unavailable: {exc}")
            continue
        print(f"{c['index']:>5}  {c['node']:<13}{c['name']:<34}{fps:5.1f} fps  brightness {brightness:5.1f}")
        if c["by_path"]:
            print(f"       stable: {c['by_path']}")
    print("\nDevice numbers move when cameras are replugged. Pass a stable path or a "
          "name fragment:\n  python3 run.py teleop --camera /dev/v4l/by-path/...-index0"
          "\n  python3 run.py teleop --camera icspring")
    return 0


# ------------------------------------------------------------------ handview

def cmd_handview(args) -> int:
    """Show the hand rendered from its URDF at a pose. No camera, no motors."""
    import cv2
    from .handview import HandRenderer, DEFAULT_YAW, DEFAULT_PITCH

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


# ------------------------------------------------------------------- profile

def cmd_profile(args) -> int:
    """Measure this operator's hand: finger lengths from an open hand, and the
    tracker's residual gap in a real pinch. No motors involved."""
    import cv2

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


# ------------------------------------------------------------------------ stop

def cmd_stop(args) -> int:
    return session.stop(timeout=args.timeout)


# ------------------------------------------------------------------------ cli

def build_parser() -> argparse.ArgumentParser:
    # Shared options live on a parent parser so they can be given after the
    # subcommand, which is where people naturally reach for them.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", default=DEFAULT_PORT, help="RS485 serial port")
    common.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    common.add_argument("--camera", default="auto",
                        help="camera index, /dev node, by-path link, or name fragment; "
                             "'auto' uses the one saved by `run.py cameras --use`")
    common.add_argument("--hand", default="Right", choices=["Right", "Left"],
                        help="which of your hands to track")
    common.add_argument("--model", default=str(DEFAULT_MODEL))
    common.add_argument("--poses", default=str(DEFAULT_POSE_FILE),
                        help="pose library: robot joint angles per named pose")
    common.add_argument("--mirror", action="store_true", default=True,
                        help="mirror the camera image (default: on)")
    common.add_argument("--no-mirror", dest="mirror", action="store_false")
    common.add_argument("--no-display", action="store_true",
                        help="run without an OpenCV window")

    p = argparse.ArgumentParser(
        prog="dexdh116",
        description="Teleoperate the DH116 dexterous hand with camera hand tracking.",
    )
    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("check", parents=[common],
                   help="verify camera, tracking and hand link; no motion")
    cal = sub.add_parser("calibrate", parents=[common],
                         help="record the tracker signals for the poses in the library")
    cal.add_argument("--all", action="store_true",
                     help="re-record every pose, not just the ones missing signals")

    t = sub.add_parser("teleop", parents=[common], help="run live teleoperation")
    t.add_argument("--dry-run", action="store_true",
                   help="track and display but never command the hand")
    t.add_argument("--no-home", dest="home", action="store_false", default=True,
                   help="skip homing (hand must already be homed)")
    t.add_argument("--max-speed", type=float, default=180.0,
                   help="host-side joint rate limit, deg/s")
    t.add_argument("--max-current", type=int, default=500,
                   help="per-motor current limit in per-mille (1000 = full)")
    t.add_argument("--smoothing", type=float, default=0.35,
                   help="EMA factor, lower is smoother but laggier")
    t.add_argument("--deadband", type=float, default=0.8,
                   help="ignore commanded changes smaller than this, in degrees")
    t.add_argument("--lost-timeout-frames", type=int, default=15)
    t.add_argument("--no-stall-kick", dest="stall_kick", action="store_false", default=True,
                   help="disable the brief overshoot that unsticks a worm-drive joint parked short of target")
    t.add_argument("--keep-enabled", action="store_true",
                   help="leave the motors energized and holding on exit "
                        "(default: de-energize so the hand goes limp)")
    t.add_argument("--blend-power", type=float, default=2.5,
                   help="pose blending sharpness; higher snaps harder to the "
                        "nearest recorded pose")
    t.add_argument("--diag", action="store_true",
                   help="overlay nearest anchor, its normalized distance and top blend weights")
    t.add_argument("--record", default=None, metavar="CSV",
                   help="log per-frame signals, nearest anchor and commanded angles to CSV")
    t.add_argument("--show-measured", action="store_true",
                   help="outline the pose the hand actually reached over the commanded one")
    t.add_argument("--profile", default=str(_profile.DEFAULT_PROFILE),
                   help="fk mode: operator hand profile from `run.py profile`")
    t.add_argument("--no-tactile", dest="tactile", action="store_false", default=True,
                   help="fk mode: disable closing pinches by feel (thumb-tip sensor / finger stall)")
    t.add_argument("--allow-estimates", action="store_true",
                   help="also blend poses whose robot joints are estimates, not jog-measured")
    t.add_argument("--mode", choices=["poses", "direct", "hybrid", "fk"], default="poses",
                   help="poses: pose-library blend (ours); direct: vendor-style "
                        "per-joint linear map; hybrid: direct fingers + pose-blend thumb")
    t.add_argument("--direct-calibration", default=str(_direct.DEFAULT_DIRECT_FILE))
    t.add_argument("--lm-alpha", type=float, default=0.3,
                   help="direct: landmark EMA factor (vendor default 0.3)")
    t.add_argument("--lm-deadband", type=float, default=0.02,
                   help="direct: landmark deadband in normalized units (vendor 0.02)")
    t.add_argument("--no-kalman", action="store_true",
                   help="direct: disable the per-landmark Kalman stage")

    j = sub.add_parser("jog", parents=[common],
                       help="drive joints by hand to see what each one does")
    j.add_argument("--dry-run", action="store_true")
    j.add_argument("--no-home", dest="home", action="store_false", default=True)
    j.add_argument("--max-speed", type=float, default=180.0)
    j.add_argument("--max-current", type=int, default=500)
    j.add_argument("--no-stall-kick", dest="stall_kick", action="store_false", default=True,
                   help="disable the brief overshoot that unsticks a worm-drive joint parked short of target")
    j.add_argument("--keep-enabled", action="store_true",
                   help="leave the motors energized and holding on exit "
                        "(default: de-energize so the hand goes limp)")

    ts = sub.add_parser("touchscan", parents=[common],
                        help="map contact geometry with the tactile sensors")
    ts.add_argument("--fingers", default="index,middle,ring,pinky")
    ts.add_argument("--abd", default="0,10,20,30,40,50,60",
                    help="thumb abduction grid, degrees")
    ts.add_argument("--flex", default="0,10,20,30",
                    help="thumb flexion grid, degrees")
    ts.add_argument("--threshold", type=float, default=0.05,
                    help="thumb-tip pressure that counts as contact")
    ts.add_argument("--step", type=float, default=2.0, help="finger step, degrees")
    ts.add_argument("--dwell", type=float, default=0.12, help="seconds per step")
    ts.add_argument("--speed", type=float, default=90.0, help="deg/s")
    ts.add_argument("--max-current", type=int, default=300)
    ts.add_argument("--out", default=str(_touchscan.DEFAULT_SCAN_FILE))

    ph = sub.add_parser("photostep", parents=[common],
                        help="park joints at preset angles for calibration photos")
    ph.add_argument("--joints", default="2,1,0",
                    help="joint indices to step, 0-based (2=index, 1=thumb flex, 0=thumb abd)")
    ph.add_argument("--max-current", type=int, default=500)

    cams = sub.add_parser("cameras", help="list capture devices and their stable paths")
    cams.add_argument("--settle", type=float, default=1.5,
                      help="seconds to let auto-exposure settle before measuring")
    cams.add_argument("--use", default=None,
                      help="remember this camera (index, node or name) as the default")

    hv = sub.add_parser("handview", help="render the hand from its URDF; no camera or motors")
    hv.add_argument("--pose", default=None, help="six firmware angles, comma separated")
    hv.add_argument("--poses", default=None, help="browse every anchor in a pose library")
    hv.add_argument("--direction", default="R", choices=["R", "L"])
    hv.add_argument("--width", type=int, default=560)
    hv.add_argument("--height", type=int, default=420)
    hv.add_argument("--out", default=None, help="write one image and exit")

    pf = sub.add_parser("profile", parents=[common],
                        help="measure this operator's hand for fk mode (no motors)")
    pf.add_argument("--out", default=str(_profile.DEFAULT_PROFILE))
    pf.add_argument("--name", default="")

    st = sub.add_parser("stop", help="stop the running DexDH116 session cleanly")
    st.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for a clean shutdown before SIGTERM")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    handlers = {"check": cmd_check, "calibrate": cmd_calibrate,
                "teleop": cmd_teleop, "jog": cmd_jog, "stop": cmd_stop,
                "touchscan": cmd_touchscan, "photostep": cmd_photostep,
                "profile": cmd_profile, "handview": cmd_handview,
                "cameras": cmd_cameras}
    try:
        if args.command in ("stop", "handview", "cameras"):
            return handlers[args.command](args)
        with session.hold(args.command, port=getattr(args, "port", None),
                          camera=getattr(args, "camera", None)):
            return handlers[args.command](args)
    except session.SessionBusy as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
