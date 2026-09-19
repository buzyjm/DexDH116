"""``touchscan``: sweep fingers onto the thumb and record every tactile contact."""

from __future__ import annotations

from pathlib import Path

from .. import touchscan as _touchscan
from ..hand import Hand


def run(args) -> int:
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
        _touchscan.run_scan(hand, fingers, abd, flex, out, threshold=args.threshold,
                            step_deg=args.step, dwell_s=args.dwell)
    except KeyboardInterrupt:
        print("\ninterrupted - partial results are saved")
    finally:
        hand.disconnect()
    print()
    print(_touchscan.summarize(out))
    return 0
