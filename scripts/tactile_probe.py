#!/usr/bin/env python3
"""Read every tactile sensor once, read-only: no enable, no homing.

Press a fingertip while it runs to see contact values. See docs/tactile.md.

    python3 scripts/tactile_probe.py
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telehand.bootstrap import bootstrap  # noqa: E402

bootstrap()

from telehand.hand import Hand  # noqa: E402
from telehand.touchscan import SENSOR_IDS  # noqa: E402


def main() -> int:
    hand = Hand(dry_run=True)
    hand.connect(home=False)
    sdk = hand._sdk
    try:
        sdk.set_sensor_enable(True)
        time.sleep(1.0)
        sdk.set_finger_pressure_reset()
        time.sleep(0.6)
        for name, sid in SENSOR_IDS.items():
            pressure = sdk.get_finger_pressure(sid)
            normal = sdk.get_finger_normal_force(sid)
            direction = sdk.get_finger_force_direction(sid)
            print(f"{sid:2d} {name:10s} max_p={max(pressure):.2f} nf={normal:.2f} "
                  f"dir={direction:.0f} p={[round(x, 2) for x in pressure]}")
    finally:
        hand.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
