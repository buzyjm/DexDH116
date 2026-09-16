"""Experimental subcommands: actuator characterisation and hand-frame replay.

Registered by ``teleop.build_parser`` so they appear in ``run.py --help``, but
the heavy modules behind them are imported only when the command runs, so
plain teleop never touches them.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

from ..hand import Hand, JOINT_NAMES, NUM_JOINTS, DEFAULT_PORT, DEFAULT_BAUD


def add_parsers(sub) -> None:
    sr = sub.add_parser("stepresponse",
                        help="[experimental] measure the DH116's actuator step response (MOVES THE HAND)")
    sr.add_argument("--port", default=DEFAULT_PORT)
    sr.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    sr.add_argument("--joints", default=None, help="comma-separated joint indices 0-5 (default: all)")
    sr.add_argument("--speeds", default="150,200,250", help="set_angular_velocity values to compare")
    sr.add_argument("--steps", default="20,40,80", help="step sizes in degrees (clamped to each joint's travel)")
    sr.add_argument("--drive", default="raw",
                    help="raw = straight to firmware (no host rate limit, no stall kick); "
                         "host = through write_angles as teleop does; 'raw,host' runs both")
    sr.add_argument("--host-rate", type=float, default=300.0, metavar="DEG_S",
                    help="host path: write_angles rate limit, i.e. how fast the commanded target ramps")
    sr.add_argument("--host-hz", type=float, default=30.0, metavar="HZ",
                    help="host path: how often a new target is issued (teleop ~30)")
    sr.add_argument("--compare-kick", action="store_true",
                    help="repeat every host-driven step with the stall kick on and off")
    sr.add_argument("--repeats", type=int, default=1)
    sr.add_argument("--settle", type=float, default=1.2, help="seconds allowed to reach the start pose")
    sr.add_argument("--window", type=float, default=2.0, help="seconds of response recorded after each step")
    sr.add_argument("--max-current", type=int, default=500)
    sr.add_argument("--no-home", dest="home", action="store_false")
    sr.add_argument("--serial-flush", action="store_true",
                    help="keep the vendor's per-write flush (slower feedback); off by default")
    sr.add_argument("--traces", action="store_true", help="also write the full per-sample traces as JSON")
    sr.add_argument("--out", default="data/benchmarks/stepresponse.csv")

    fr = sub.add_parser("frames",
                        help="[experimental] wrist-local joint frames from a recording: replay viewer "
                             "and stability metrics (no hardware)")
    fr.add_argument("--replay", required=True, metavar="NPZ",
                    help="recording from `teleop --record` (world_pts, handedness, t)")
    fr.add_argument("--fingers", default="thumb,index", help="which chains to reconstruct")
    fr.add_argument("--assume-mirrored", dest="assume_mirrored", action="store_true", default=True,
                    help="the recording came from a mirrored webcam (default)")
    fr.add_argument("--no-assume-mirrored", dest="assume_mirrored", action="store_false")
    fr.add_argument("--fps", type=float, default=30.0, help="playback rate")
    fr.add_argument("--axis-thickness", type=int, default=4, metavar="PX",
                    help="axis arrow shaft thickness (palm gets +2)")
    fr.add_argument("--no-display", action="store_true", help="metrics only, no window")
    fr.add_argument("--out", default=None, metavar="JSON", help="write the report and per-frame log")

def probe_velocity_ceiling(hand) -> dict:
    """Ask the firmware what angular velocities it will actually accept.

    set_angular_velocity is write-only from our side until read back, and the
    value telehand has always written (150) is neither a measured ceiling nor
    the vendor's default (200). Write a ladder, read each back, and report
    where it stops honouring the request.
    """
    out = {}
    original = None
    try:
        original = hand._sdk.get_angular_velocity(1)
    except Exception:
        pass
    for want in (50, 100, 150, 200, 300, 400, 600, 800, 1000, 1500):
        try:
            hand._sdk.set_angular_velocity(0, float(want))
            time.sleep(0.05)
            got = float(hand._sdk.get_angular_velocity(1))
        except Exception as exc:
            out[want] = f"error: {type(exc).__name__}"
            continue
        out[want] = got
    if original is not None:
        hand._sdk.set_angular_velocity(0, float(original))
    return out


def cmd_stepresponse(args) -> int:
    """Measure the hand's real actuator response, isolated from our software."""
    from . import stepresponse as _step
    joints = ([int(j) for j in args.joints.split(",")] if args.joints
              else list(range(NUM_JOINTS)))
    speeds = [float(v) for v in args.speeds.split(",")]
    steps = [float(v) for v in args.steps.split(",")]

    hand = Hand(args.port, args.baud, max_current=args.max_current,
                stall_kick=False, max_deg_per_s=args.host_rate,
                no_serial_flush=not args.serial_flush)
    print("Serial flush: %s" % ("ON (vendor default - monitor will be slower)"
                                if args.serial_flush else "BYPASSED (faster feedback)"))
    print("Connecting to hand ...")
    if args.home:
        print("Homing: the hand will move through its full range. Keep it clear.")
    hand.connect(home=args.home)
    print("Connected.")
    print("Firmware limits: " + ", ".join(
        f"{JOINT_NAMES[i]} {l.min_angle:.0f}-{l.max_angle:.0f}"
        for i, l in enumerate(hand.limits)))

    results, probe = [], None
    sampling = {}
    try:
        print("\nProbing the firmware's angular-velocity ceiling ...")
        ladder = probe_velocity_ceiling(hand)
        for want, got in ladder.items():
            mark = "" if isinstance(got, str) or abs(got - want) < 1e-3 else "   <- clamped"
            print(f"  set {want:>5} -> reads back {got}{mark}")

        print("\nMeasuring feedback update rate (this bounds every timing below) ...")
        samp = _step.measure_sampling(hand, seconds=2.0)
        sampling.update(samp)
        print(f"  polled at {samp['poll_rate_hz']:.0f} Hz; feedback changed "
              f"{samp['update_rate_hz']:.1f} times/s")
        if "update_interval_p50_ms" in samp:
            print(f"  update interval: p50 {samp['update_interval_p50_ms']:.1f} ms, "
                  f"p95 {samp['update_interval_p95_ms']:.1f} ms")
            print(f"  => timings finer than ~{samp['update_interval_p50_ms']:.0f} ms "
                  "are not resolvable")

        probe = _step.StepProbe(hand, settle_s=args.settle, window_s=args.window,
                                host_hz=args.host_hz)
        if "host" in args.drive:
            print(f"Host path: write_angles rate limit {args.host_rate:.0f} deg/s, "
                  f"targets issued at {args.host_hz:.0f} Hz "
                  f"({args.host_rate / args.host_hz:.1f} deg per frame)")
        drives = args.drive.split(",")
        kicks = [False, True] if args.compare_kick else [False]
        total = len(joints) * len(speeds) * len(steps) * len(drives) * len(kicks) * 2 * args.repeats
        print(f"\n{total} steps to run, about {total * (args.settle + args.window):.0f} s.\n")

        n = 0
        for j in joints:
            lo, hi = hand.limits[j].min_angle, hand.limits[j].max_angle
            # Velocity ascends so a failure can stop the escalation for this
            # joint rather than repeating a fault at every higher setting.
            capped_at = None
            for v in sorted(speeds):
                if capped_at is not None:
                    print(f"  skipping {JOINT_NAMES[j]} at v={v:.0f}: "
                          f"a step already failed at v={capped_at:.0f}")
                    continue
                for step in steps:
                    span = min(step, hi - lo)
                    for drive in drives:
                        # The raw path never calls write_angles, so the stall
                        # kick cannot act on it; running both would duplicate
                        # every step for nothing.
                        for kick in (kicks if drive == "host" else [False]):
                            for _ in range(args.repeats):
                                for a, b in ((lo, lo + span), (lo + span, lo)):
                                    n += 1
                                    r = probe.run_step(j, a, b, velocity=v, drive=drive,
                                                       stall_kick=kick,
                                                       max_current=args.max_current)
                                    results.append(r)
                                    flag = "" if r.reached else "   <- NOT REACHED"
                                    print(f"  [{n}/{total}] {r.joint_name:<15} "
                                          f"{r.direction:<5} {r.step_deg:>+5.0f} deg  "
                                          f"v={v:<5.0f} {drive:<4} kick={'Y' if kick else 'n'}  "
                                          f"upd={r.updates:>3} lag={r.cmd_lag_max:>5.1f} "
                                          f"t90={r.t90_ms:>5.0f}ms "
                                          f"sust={r.sustained_vel_deg_s:>5.0f}d/s "
                                          f"err={r.final_err_deg:>+5.1f}{flag}")
                                    if not r.reached:
                                        print(f"       stopped at {r.stop_deg:.1f} deg ({r.stop_ms:.0f} ms)"
                                              f" | target sent {r.target_deg:.1f}, at stop {r.target_at_stop:.1f},"
                                              f" after {r.target_after:.1f}"
                                              f" | status {r.status_before}->{r.status_at_stop}->{r.status_after}"
                                              f" [{r.statuses_seen}] | reached end {r.reached_after}")
                                        print(f"       => {r.stop_class}")
                                        alm = probe.recover()
                                        r.cleared = alm
                                        probe.traces[-1]["result"]["cleared"] = alm
                                        print(f"       recovered (alarm {'cleared' if alm else 'none'}"
                                              f", before [{r.alarm_before}] after [{r.alarm_after}]"
                                              + (f", during {r.alarm_during}" if r.alarm_during else "")
                                              + f", peak current {r.peak_current:.0f}/{args.max_current})")
                                        capped_at = v
        print("\nReturning to open ...")
        hand._sdk.set_angular_velocity(0, 150.0)
        probe._goto([l.min_angle for l in hand.limits])
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if results:
            _step.save(results, probe.traces if probe else [], Path(args.out),
                       Path(args.out).with_suffix(".traces.json") if args.traces else None)
            print(_step.summarize(results, sampling, probe.traces if probe else None))
            print(f"\n  {len(results)} steps -> {args.out}")
            if args.traces:
                print(f"  full traces -> {Path(args.out).with_suffix('.traces.json')}")
        hand.disconnect()
        print("Disconnected.")
    return 0


def cmd_frames(args) -> int:
    """Reconstruct wrist-local joint frames from a landmark recording and check them.

    Replay only for now: no camera, no motors. The recording is what
    `teleop --record` writes (world_pts, handedness, t).
    """
    import cv2
    from .. import handpose as _hp
    from . import frameviz as _fv

    d = np.load(args.replay, allow_pickle=True)
    if "world_pts" not in d.files:
        print(f"ERROR: {args.replay} has no world_pts (record with `teleop --record`)", file=sys.stderr)
        return 1
    P = d["world_pts"].astype(np.float64)
    labels = d["handedness"] if "handedness" in d.files else np.array(["?"] * len(P))
    T = d["t"] if "t" in d.files else np.arange(len(P)) / 30.0
    fingers = tuple(f.strip() for f in args.fingers.split(",") if f.strip())

    canon = _hp.Canonicalizer(assume_mirrored=args.assume_mirrored)
    tracker = _hp.HandPoseTracker(fingers)
    stats = _fv.FrameStats(fingers)
    counts = dict(zip(*np.unique(labels, return_counts=True)))
    print(f"replay {args.replay}: {len(P)} frames, labels {counts}, fingers {fingers}")
    print("canonicalization: geometry decides when a finger is curled; continuity otherwise; "
          f"default {'LEFT geometry (mirrored webcam)' if args.assume_mirrored else 'RIGHT'} until then. "
          "Labels are counted, never applied.")

    display = not args.no_display
    view = None
    if display:
        view = _fv.FrameView()
        cv2.namedWindow("frames")
        view.attach("frames")
        print("keys: space pause, n step, hjkl/r/drag orbit, wheel zoom, q quit")
    per_frame = []            # (i, label, source, vote, chirality, palm jump deg)
    prev_R = None
    i = 0
    paused = False
    try:
        while i < len(P):
            pts = canon(P[i], str(labels[i]))
            pose = tracker.update(pts, int(float(T[i]) * 1000))
            stats.add(pose, pts, invariance_test=(i % 10 == 0))
            jump = _fv.geodesic_deg(prev_R, pose.wrist_rotation) if prev_R is not None else 0.0
            prev_R = pose.wrist_rotation
            per_frame.append((i, str(labels[i]), canon.last_source, canon.last_vote,
                              canon.chirality, jump))
            if display:
                while True:
                    lines = [f"frame {i + 1}/{len(P)}   t={float(T[i]):.2f}s   {'PAUSED' if paused else ''}",
                             f"label {labels[i]}  chirality {'L' if canon.chirality < 0 else 'R'} "
                             f"by {canon.last_source} (vote {canon.last_vote:+.0f})",
                             f"palm jump {jump:5.2f} deg   sign margin {pose.sign_margin:.3f}   "
                             f"planarity {pose.planarity:.3f}",
                             ""]
                    for n in _hp.joint_names(fingers):
                        j = pose.joints[n]
                        if n == "palm":
                            lines.append(f"{n:<11} conf {j.confidence:.2f}")
                        elif not n.endswith("_tip"):
                            rv = pose.relative_rotvec(n)
                            lines.append(f"{n:<11} conf {j.confidence:.2f}  bend {pose.bend_angle_deg(n):5.1f}"
                                         f"  rotvec [{rv[0]:+6.1f} {rv[1]:+6.1f} {rv[2]:+6.1f}]")
                    canvas = _fv.render_replay(pose, lines, axis_thickness=args.axis_thickness,
                                               **view.kwargs)
                    cv2.imshow("frames", canvas)
                    key = cv2.waitKey(max(1, int(1000 / args.fps)) if not paused else 30) & 0xFF
                    if key in (ord("q"), 27):
                        raise KeyboardInterrupt
                    if key == ord(" "):
                        paused = not paused
                    elif view.handle_key(key):
                        pass
                    if not paused or key == ord("n"):
                        break
            i += 1
    except KeyboardInterrupt:
        print("\nstopped at frame", i)
    finally:
        if display:
            cv2.destroyAllWindows()

    report = stats.report()
    print(_fv.format_report(report))
    c = canon.counts
    print()
    print(f"canonicalization over {len(per_frame)} frames: geometry {c['geometry']}, continuity "
          f"{c['continuity']}, default {c['default']}; chirality switches {c['switches']}; "
          f"label disagreed with the decision in {c['label_disagreements']} frames")
    dis = [f for f in per_frame if f[1] not in ("?", "") and
           ((f[1].lower().startswith("l")) != (f[4] < 0))]
    if dis:
        print("  frames where the LABEL disagreed (idx, label, decided by, vote, palm jump vs previous frame):")
        for f in dis[:12]:
            print(f"    {f[0]:>5}  {f[1]:<6} {f[2]:<11} {f[3]:+7.0f}   {f[5]:6.2f} deg")
    big = [f for f in per_frame if f[5] > 45.0]
    print(f"  palm-frame jumps over 45 deg: {len(big)}" + (f"  at frames {[f[0] for f in big[:10]]}" if big else ""))
    if args.out:
        import json
        Path(args.out).write_text(json.dumps({"report": report, "canonicalization": c,
                                              "per_frame": per_frame}, default=float, indent=1))
        print(f"  written: {args.out}")
    return 0



HANDLERS = {"stepresponse": cmd_stepresponse, "frames": cmd_frames}
# `frames` replays a recording: no camera, no hand, no session lock needed.
NO_HARDWARE = ("frames",)
