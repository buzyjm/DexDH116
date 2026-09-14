"""Read every tactile sensor once, read-only (no enable, no homing).

Press a fingertip while it runs to see contact values. See TACTILE.md.
"""
import sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "vendor"))
sys.path.insert(0, str(ROOT))
from dexdh116.hand import Hand

SENSORS = {1: "thumb_tip", 2: "thumb_pad", 3: "index_tip", 4: "index_pad",
           5: "middle_tip", 6: "middle_pad", 7: "ring_tip", 8: "ring_pad",
           9: "pinky_tip", 10: "pinky_pad", 11: "palm"}

hand = Hand(dry_run=True)
hand.connect(home=False)
sdk = hand._sdk
try:
    sdk.set_sensor_enable(True); time.sleep(1.0)
    sdk.set_finger_pressure_reset(); time.sleep(0.6)
    for sid, name in SENSORS.items():
        p = sdk.get_finger_pressure(sid)
        nf = sdk.get_finger_normal_force(sid)
        d = sdk.get_finger_force_direction(sid)
        print(f"{sid:2d} {name:10s} max_p={max(p):.2f} nf={nf:.2f} dir={d:.0f} p={[round(x, 2) for x in p]}")
finally:
    hand.disconnect()
