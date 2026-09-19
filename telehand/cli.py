"""Argument parsing and command dispatch for ``run.py``.

Each subcommand lives in ``telehand.commands.<name>`` and is imported only
when invoked. Commands that drive the hand or the camera run inside the
one-instance session lock (see ``session.py``); the rest do not need it.
"""

from __future__ import annotations

import argparse
import importlib
import sys

from . import paths, session
from .hand import DEFAULT_BAUD, DEFAULT_PORT

# Commands that touch neither the hand nor a camera exclusively.
UNLOCKED_COMMANDS = {"stop", "handview", "cameras"}


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
    common.add_argument("--model", default=str(paths.HAND_LANDMARKER_MODEL))
    common.add_argument("--poses", default=str(paths.POSES_FILE),
                        help="pose library: robot joint angles per named pose")
    common.add_argument("--mirror", action="store_true", default=True,
                        help="mirror the camera image (default: on)")
    common.add_argument("--no-mirror", dest="mirror", action="store_false")
    common.add_argument("--no-display", action="store_true",
                        help="run without an OpenCV window")

    p = argparse.ArgumentParser(
        prog="telehand",
        description="Teleoperate the LHandPro dexterous hand with camera hand tracking.",
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
    t.add_argument("--profile", default=str(paths.PROFILE_FILE),
                   help="fk mode: operator hand profile from `run.py profile`")
    t.add_argument("--no-tactile", dest="tactile", action="store_false", default=True,
                   help="fk mode: disable closing pinches by feel (thumb-tip sensor / finger stall)")
    t.add_argument("--allow-estimates", action="store_true",
                   help="also blend poses whose robot joints are estimates, not jog-measured")
    t.add_argument("--mode", choices=["poses", "direct", "hybrid", "fk"], default="poses",
                   help="poses: pose-library blend (ours); direct: vendor-style "
                        "per-joint linear map; hybrid: direct fingers + pose-blend thumb")
    t.add_argument("--direct-calibration", default=str(paths.DIRECT_CALIBRATION_FILE))
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
    ts.add_argument("--out", default=str(paths.TOUCHSCAN_FILE))

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
    pf.add_argument("--out", default=str(paths.PROFILE_FILE))
    pf.add_argument("--name", default="")

    st = sub.add_parser("stop", help="stop the running telehand session cleanly")
    st.add_argument("--timeout", type=float, default=15.0,
                    help="seconds to wait for a clean shutdown before SIGTERM")
    return p



def run_command(name: str, args: argparse.Namespace) -> int:
    if name == "stop":
        return session.stop(timeout=args.timeout)
    module = importlib.import_module(f"telehand.commands.{name}")
    return module.run(args)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command in UNLOCKED_COMMANDS:
            return run_command(args.command, args)
        with session.hold(args.command, port=getattr(args, "port", None),
                          camera=getattr(args, "camera", None)):
            return run_command(args.command, args)
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
