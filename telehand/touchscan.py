"""Let the hand measure its own kinematics with its tactile sensors.

The firmware's joint "angles" are converted from linear-actuator stroke by a
model the vendor does not publish, so they are not the URDF's joint angles and
no internal register reveals the relationship. Contact is observable, though:
the thumb-tip sensor reads a clean zero in free air and jumps on touch. So we
park the thumb on a grid of poses, close one finger at a time until the thumb
feels it, and record where that happened. Each contact is one geometric
constraint on the firmware->URDF map; a few hundred of them pin it down where
three hand-found pinches could not.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from .hand import Hand, JOINT_NAMES
from .paths import TOUCHSCAN_FILE

FINGER_JOINT = {"index": 2, "middle": 3, "ring": 4, "pinky": 5}
SENSOR_IDS = {
    "thumb_tip": 1, "thumb_pad": 2,
    "index_tip": 3, "index_pad": 4,
    "middle_tip": 5, "middle_pad": 6,
    "ring_tip": 7, "ring_pad": 8,
    "pinky_tip": 9, "pinky_pad": 10,
    "palm": 11,
}


@dataclass
class Contact:
    finger: str
    thumb_abd: float
    thumb_flex: float
    finger_angle: Optional[float]      # None = closed fully with no contact
    measured: List[float]              # what the hand reported at that moment
    sensors: Dict[str, float]          # max taxel pressure per pad


class TouchScanner:
    def __init__(self, hand: Hand, *, threshold: float = 0.05,
                 step_deg: float = 2.0, dwell_s: float = 0.12) -> None:
        self.hand = hand
        self.sdk = hand._sdk
        self.threshold = threshold
        self.step_deg = step_deg
        self.dwell_s = dwell_s

    # ------------------------------------------------------------ sensors

    def enable_sensors(self) -> None:
        self.sdk.set_sensor_enable(True)
        time.sleep(1.0)
        self.zero()

    def zero(self) -> None:
        self.sdk.set_finger_pressure_reset()
        time.sleep(0.6)

    def read_sensors(self) -> Dict[str, float]:
        out = {}
        for name, sid in SENSOR_IDS.items():
            try:
                vals = self.sdk.get_finger_pressure(sid)
                out[name] = float(max(vals)) if vals else 0.0
            except Exception:
                out[name] = float("nan")
        return out

    # ------------------------------------------------------------- motion

    def goto(self, angles: Sequence[float], settle: float = 0.6) -> None:
        """Command a pose and hold it until the rate limiter has delivered it."""
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            sent = self.hand.write_angles(angles)
            if max(abs(s - a) for s, a in zip(sent, angles)) < 0.05:
                break
            time.sleep(0.03)
        time.sleep(settle)

    def close_until_touch(self, finger: str, thumb_abd: float,
                          thumb_flex: float) -> Contact:
        j = FINGER_JOINT[finger]
        lim = self.hand.limits[j]
        pose = [thumb_abd, thumb_flex, 0.0, 0.0, 0.0, 0.0]
        self.goto(pose)
        # Re-zero with everything open so slow drift cannot masquerade as touch.
        self.zero()

        angle = lim.min_angle
        while angle <= lim.max_angle + 1e-6:
            pose[j] = angle
            self.goto(pose, settle=self.dwell_s)
            sensors = self.read_sensors()
            if sensors["thumb_tip"] >= self.threshold:
                # Back off at once so the finger does not keep loading the thumb.
                pose[j] = max(lim.min_angle, angle - 3 * self.step_deg)
                self.goto(pose, settle=0.1)
                return Contact(finger, thumb_abd, thumb_flex, angle,
                               self.hand.read_angles(), sensors)
            angle += self.step_deg

        pose[j] = lim.min_angle
        self.goto(pose, settle=0.1)
        return Contact(finger, thumb_abd, thumb_flex, None,
                       self.hand.read_angles(), self.read_sensors())


def run_scan(hand: Hand, fingers: List[str], abd_values: List[float],
             flex_values: List[float], out_path: Path, *, threshold: float,
             step_deg: float, dwell_s: float) -> List[Contact]:
    scanner = TouchScanner(hand, threshold=threshold, step_deg=step_deg, dwell_s=dwell_s)
    scanner.enable_sensors()

    contacts: List[Contact] = []
    if out_path.exists():
        try:
            prior = json.loads(out_path.read_text())
            contacts = [Contact(**c) for c in prior.get("contacts", [])]
            print(f"Resuming: {len(contacts)} contacts already in {out_path}")
        except Exception:
            contacts = []
    done = {(c.finger, c.thumb_abd, c.thumb_flex) for c in contacts}

    total = len(fingers) * len(abd_values) * len(flex_values)
    n = 0
    t0 = time.monotonic()
    try:
        for finger in fingers:
            for abd in abd_values:
                for flex in flex_values:
                    n += 1
                    if (finger, abd, flex) in done:
                        continue
                    c = scanner.close_until_touch(finger, abd, flex)
                    contacts.append(c)
                    _save(contacts, out_path)
                    hit = f"touch at {c.finger_angle:5.1f}" if c.finger_angle is not None else "no contact"
                    where = [k for k, v in c.sensors.items()
                             if k != "thumb_tip" and v >= threshold]
                    el = time.monotonic() - t0
                    print(f"[{n:3d}/{total}] {finger:<7} thumb=({abd:4.0f},{flex:3.0f})  "
                          f"{hit:<16} thumb_tip={c.sensors['thumb_tip']:.3f}  "
                          f"also:{where if where else '-'}  {el/60:4.1f}min", flush=True)
    finally:
        scanner.goto([0.0] * 6, settle=0.3)
        try:
            hand._sdk.set_sensor_enable(False)
        except Exception:
            pass
    return contacts


def _save(contacts: List[Contact], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({
        "version": 1,
        "joint_names": JOINT_NAMES,
        "contacts": [asdict(c) for c in contacts],
    }, indent=1) + "\n")


def summarize(path: Path = TOUCHSCAN_FILE) -> str:
    data = json.loads(Path(path).read_text())
    cs = [Contact(**c) for c in data["contacts"]]
    hits = [c for c in cs if c.finger_angle is not None]
    lines = [f"{path}: {len(cs)} sweeps, {len(hits)} contacts"]
    for f in ("index", "middle", "ring", "pinky"):
        fh = [c for c in hits if c.finger == f]
        if not fh:
            continue
        tip = sum(1 for c in fh if c.sensors.get(f"{f}_tip", 0) >= 0.05)
        pad = sum(1 for c in fh if c.sensors.get(f"{f}_pad", 0) >= 0.05)
        lines.append(f"  {f:<7} {len(fh):3d} contacts   finger_tip fired {tip:3d}   "
                     f"finger_pad fired {pad:3d}   (rest: thumb only)")
    return "\n".join(lines)
