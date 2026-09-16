"""Safe control layer over the LHandPro 6-DoF hand on RS485.

Everything that reaches the hardware goes through :meth:`Hand.write_angles`,
which clamps to the limits reported by the device and rate-limits how fast a
joint may move, so a glitch in the tracker cannot turn into a slammed finger.
"""

from __future__ import annotations

import ctypes
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

from . import sdk

# Motor ids for LAC_DOF_6, in the order the SDK enumerates them.
JOINT_NAMES = [
    "thumb_abduction",
    "thumb_flexion",
    "index_flexion",
    "middle_flexion",
    "ring_flexion",
    "pinky_flexion",
]
NUM_JOINTS = len(JOINT_NAMES)

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 500000
DEFAULT_NODE_ID = 1


@dataclass
class JointLimit:
    min_angle: float
    max_angle: float

    def clamp(self, value: float) -> float:
        return max(self.min_angle, min(self.max_angle, value))


class Hand:
    """A connected LHandPro hand.

    Parameters
    ----------
    dry_run:
        Run the whole pipeline but never command motion. The hand is still
        opened and read so limits and telemetry are real.
    max_deg_per_s:
        Ceiling on commanded joint change per second, applied host-side on top
        of the firmware's own velocity limit.
    """

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        node_id: int = DEFAULT_NODE_ID,
        *,
        dry_run: bool = False,
        keep_enabled: bool = False,
        max_deg_per_s: float = 180.0,
        angular_velocity: float = 150.0,
        max_current: int = 500,
        stall_kick: bool = True,
        skip_unchanged: bool = False,
        skip_epsilon: float = 0.1,
        resync_frames: int = 30,
        no_serial_flush: bool = False,
    ) -> None:
        self.port = port
        self.baud = baud
        self.node_id = node_id
        self.dry_run = dry_run
        self.keep_enabled = keep_enabled
        self.max_deg_per_s = max_deg_per_s
        self.angular_velocity = angular_velocity
        self.max_current = max_current
        # Worm-drive joints stick: a small position error may not produce
        # enough drive to break static friction, and the joint parks short of
        # its target (the thumb flexion sits at ~9 deg when asked for 15). If a
        # joint has been steady and short of a steady target, briefly overshoot
        # the command to unstick it, then restore.
        self.stall_kick = stall_kick
        self._kick_state = {}   # joint -> dict(since, until, cooldown_until)
        self.kick_count = [0] * NUM_JOINTS

        # Each set_target_angle is its own serial transaction, so a joint whose
        # command has not moved is a transaction bought for nothing. Comparison
        # is against the value last actually *sent*, not the last requested one,
        # so a stall kick (which sends something other than the target) is seen
        # as a change and so is its restoration.
        self.skip_unchanged = skip_unchanged
        self.skip_epsilon = skip_epsilon
        # RS485 here is fire-and-forget: a lost frame is never retransmitted.
        # If one is dropped and the command then never changes, that joint
        # would hold a stale target indefinitely. A periodic full resend bounds
        # that to resync_frames (~1 s at 30 Hz) at the cost of one expensive
        # frame in every resync_frames. Set 0 to disable.
        self.resync_frames = resync_frames
        self.no_serial_flush = no_serial_flush
        self._last_sent = None          # what each joint was last commanded with
        self._writes_since_resync = 0
        self.last_tx_count = 0          # serial transactions in the last write
        self.tx_total = 0

        self._controller = None
        self._sdk = None
        self.limits: List[JointLimit] = []
        self._commanded: Optional[List[float]] = None
        self._last_write: Optional[float] = None
        self.connected = False

    # ------------------------------------------------------------------ setup

    def connect(self, *, home: bool = True, home_wait: float = 6.0) -> None:
        """Open the bus, enable and (by default) home the motors.

        Homing physically drives every joint to find its reference, so the hand
        must be clear of obstructions before this is called.
        """
        RS485Controller, _ = sdk.load()
        # After load(), the vendor example dir is on sys.path, so serial_port
        # is importable and its class can be patched. Nothing on disk changes.
        if self.no_serial_flush:
            sdk.disable_serial_flush()
        self._controller = RS485Controller()

        ok = self._controller.connect(
            enable_motors=not self.dry_run,
            home_motors=home and not self.dry_run,
            home_wait_time=home_wait,
            rs485_port_name=self.port,
            rs485_baud_rate=self.baud,
            rs485_node_id=self.node_id,
        )
        if not ok:
            raise RuntimeError(
                f"Failed to connect to hand on {self.port} @ {self.baud}. "
                "Check the cable, power, and that no other process holds the port."
            )

        self._sdk = self._controller.sdk_handle
        self._sdk.start_monitor()
        time.sleep(0.5)

        self.limits = self._read_limits()
        self.connected = True

        if not self.dry_run:
            self._sdk.set_angular_velocity(0, self.angular_velocity)
            self._sdk.set_max_current(0, self.max_current)

        # Start tracking from wherever the hand actually is.
        self._commanded = self.read_angles()
        self._last_write = time.monotonic()

    def _read_limits(self) -> List[JointLimit]:
        """Ask the firmware for each joint's travel; fall back to a safe guess."""
        fn = self._sdk._lib.lhandprolib_get_limit_target_angle
        handle = self._sdk._handle
        limits = []
        for motor_id in range(1, NUM_JOINTS + 1):
            lo, hi = ctypes.c_float(), ctypes.c_float()
            rc = fn(handle, motor_id, ctypes.byref(lo), ctypes.byref(hi))
            if rc == 0 and hi.value > lo.value:
                limits.append(JointLimit(float(lo.value), float(hi.value)))
            else:
                limits.append(JointLimit(0.0, 30.0))
        return limits

    # ------------------------------------------------------------------- read

    def read_angles(self) -> List[float]:
        """Current measured joint angles in degrees."""
        out = []
        for motor_id in range(1, NUM_JOINTS + 1):
            try:
                out.append(float(self._sdk.get_now_angle(motor_id)))
            except Exception:
                out.append(0.0)
        return out

    def alarms(self) -> List[int]:
        out = []
        for motor_id in range(1, NUM_JOINTS + 1):
            try:
                out.append(int(self._sdk.get_now_alarm(motor_id)))
            except Exception:
                out.append(0)
        return out

    def clear_alarms(self) -> None:
        try:
            self._sdk.set_clear_alarm(0)
        except Exception:
            pass

    # ------------------------------------------------------------------ write

    def write_angles(self, angles: Sequence[float]) -> List[float]:
        """Clamp, rate-limit and command a full joint vector.

        Returns the angles actually commanded, which is what the caller should
        display -- they may differ from the request after limiting.
        """
        if len(angles) != NUM_JOINTS:
            raise ValueError(f"expected {NUM_JOINTS} angles, got {len(angles)}")

        now = time.monotonic()
        dt = now - (self._last_write or now)
        self._last_write = now
        max_step = self.max_deg_per_s * max(dt, 1e-3)

        if self._commanded is None:
            self._commanded = list(angles)

        target = []
        for i, want in enumerate(angles):
            want = self.limits[i].clamp(float(want))
            prev = self._commanded[i]
            delta = max(-max_step, min(max_step, want - prev))
            target.append(prev + delta)

        self._commanded = target

        if not self.dry_run:
            sent = self._apply_stall_kick(target, now) if self.stall_kick else target
            self._send(sent)

        return target

    def _send(self, sent: Sequence[float]) -> int:
        """Push the joint targets over RS485; returns transactions issued.

        Every set_target_angle is a separate serial transaction, and so is
        move_motors. With skip_unchanged off this is always seven.
        """
        resync = (self._last_sent is None
                  or (self.resync_frames and
                      self._writes_since_resync >= self.resync_frames))

        if self.skip_unchanged and not resync:
            changed = [i for i in range(NUM_JOINTS)
                       if abs(sent[i] - self._last_sent[i]) > self.skip_epsilon]
        else:
            changed = list(range(NUM_JOINTS))

        for i in changed:
            self._sdk.set_target_angle(i + 1, sent[i])
        tx = len(changed)
        # move_motors starts motion toward the targets. With nothing newly
        # written there is nothing to start, and any move already under way is
        # unaffected, so the call is skipped rather than spent.
        if changed:
            self._sdk.move_motors(0)
            tx += 1

        self._last_sent = list(sent)
        self._writes_since_resync = 0 if resync else self._writes_since_resync + 1
        self.last_tx_count = tx
        self.tx_total += tx
        return tx

    # Tunables for the unstick logic.
    KICK_ERROR_DEG = 3.0      # short of target by more than this ...
    KICK_STEADY_S = 0.25      # ... for this long, with the joint not moving ...
    KICK_DEG = 8.0            # ... then overshoot the command by at least this much
    KICK_DEG_MAX = 12.0       # grows with the error: a worm drive needs more to let go
    KICK_HOLD_S = 0.15
    KICK_COOLDOWN_S = 0.3     # short, so a joint that stays stuck gets kicked again

    def _apply_stall_kick(self, target, now):
        measured = self.read_angles()
        sent = list(target)
        for i, (want, have) in enumerate(zip(target, measured)):
            st = self._kick_state.setdefault(i, {"since": None, "until": 0.0, "cool": 0.0,
                                                 "last_have": have, "last_want": want})
            err = want - have
            moving = abs(have - st["last_have"]) > 0.5
            st["last_have"], st["last_want"] = have, want
            kick = min(self.KICK_DEG_MAX, self.KICK_DEG + 0.4 * abs(err))
            if now < st["until"]:
                if abs(err) <= 2.0:
                    # It let go and is nearly there: stop pushing before it overshoots.
                    st["until"] = now
                else:
                    sent[i] = self.limits[i].clamp(want + kick * (1 if err > 0 else -1))
                    continue
            # A changing target is no reason to wait: under teleop the target
            # moves every frame, and a joint that sits still while short of it
            # is stuck all the same.
            if abs(err) <= self.KICK_ERROR_DEG or moving or now < st["cool"]:
                st["since"] = None
                continue
            if st["since"] is None:
                st["since"] = now
            elif now - st["since"] >= self.KICK_STEADY_S:
                st["until"] = now + self.KICK_HOLD_S
                st["cool"] = st["until"] + self.KICK_COOLDOWN_S
                st["since"] = None
                self.kick_count[i] += 1
                sent[i] = self.limits[i].clamp(want + kick * (1 if err > 0 else -1))
        return sent

    def open_hand(self) -> None:
        """Command every joint to its minimum (fully open) angle."""
        self.write_angles([lim.min_angle for lim in self.limits])

    # --------------------------------------------------------------- teardown

    def stop(self) -> None:
        """Halt motion. The motors stay energized and holding."""
        if self._sdk is not None and not self.dry_run:
            try:
                self._sdk.stop_motors(0)
            except Exception:
                pass

    def release(self) -> None:
        """De-energize the motors so the hand goes limp.

        stop_motors() only ends the current move: the drivers stay enabled and
        keep drawing current to hold position, which persists after the serial
        link closes and even after the USB cable is pulled, since that cable
        carries data only. Disabling is what actually lets go.
        """
        if self._sdk is None or self.dry_run:
            return
        try:
            self._sdk.set_enable(0, False)
            time.sleep(0.2)  # let the frame reach the hand before the port shuts
        except Exception:
            pass

    def disconnect(self) -> None:
        if self._controller is None:
            return
        try:
            self.stop()
            if not self.keep_enabled:
                self.release()
            if self._sdk is not None:
                try:
                    self._sdk.stop_monitor()
                except Exception:
                    pass
            self._controller.disconnect()
        finally:
            self._controller = None
            self._sdk = None
            self.connected = False

    def __enter__(self) -> "Hand":
        return self

    def __exit__(self, *exc) -> None:
        self.disconnect()
