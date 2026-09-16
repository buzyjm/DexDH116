"""Step-response probe: measure the DH116's real actuator response.

Everything measured before this was host-side latency, which turned out to be
30 ms against a motor needing several hundred. This measures the motor: command
a step, sample the firmware's feedback, and characterise what the joint did.

Isolating the hand from our software
------------------------------------
Three limits sit between a teleop frame and a moving motor:

* ``Hand.write_angles`` rate-limits host-side (``max_deg_per_s``).
* The firmware has its own velocity limit (``set_angular_velocity``).
  ``Hand.connect`` writes 150; the vendor's own default is 200.
* The stall kick rewrites the commanded value when a joint sits short.

The ``raw`` drive path bypasses all three -- one ``set_target_angle`` plus one
``move_motors`` -- so it measures the firmware position loop and the mechanism.
The ``host`` path drives through ``write_angles`` with the rate limiter opened
up, so the stall kick runs as it does in teleop.

Measurement floor
-----------------
``get_now_angle`` reads a value the monitor thread refreshes over RS485. Two
short runs on joint 2 measured **64 ms between updates** during motion, flush
bypass or not, with every interval on a 16 ms multiple -- consistent with the
FTDI ``latency_timer`` (16) setting the cadence. Dead time and t50/t90
differences finer than that are not measurements.

Controller-state diagnostics
----------------------------
Two long high-velocity steps on joint 2 stopped part-way with no alarm code and
current falling to its idle level. To see what the controller believes when
that happens, every poll also reads the target angle, position-reached flag and
status word, and a sample is recorded whenever *any* of angle, target, reached,
status or alarm changes -- so a status flip that does not coincide with an
angle update is still timestamped.

One caution when reading ``get_target_angle``: it may return the SDK's cached
copy of what the host last wrote rather than a value reported by the drive. A
target that reads unchanged after a stop therefore does not by itself prove the
drive still holds it; a target that reads *changed* does prove something
replaced it, since the host did not.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

from ..hand import Hand, JOINT_NAMES, NUM_JOINTS
from ..kinematics import thumb_flex_max

MOTION_DEG = 0.5          # change treated as motion rather than sensor noise
SETTLE_DEG = 1.0
SETTLE_HOLD_S = 0.15
TARGET_MATCH_DEG = 1.0    # a target within this of a reference counts as equal
POSITION_VELOCITY = 150.0  # used for every un-measured positioning move


@dataclass
class StepResult:
    joint: int
    joint_name: str
    direction: str
    start_deg: float
    target_deg: float
    step_deg: float
    velocity_setting: float
    drive: str
    stall_kick: bool
    max_current: int

    dead_ms: float = float("nan")
    t50_ms: float = float("nan")
    t90_ms: float = float("nan")
    settle_ms: float = float("nan")
    peak_vel_deg_s: float = float("nan")
    sustained_vel_deg_s: float = float("nan")
    mean_vel_deg_s: float = float("nan")
    overshoot_deg: float = float("nan")
    final_deg: float = float("nan")
    final_err_deg: float = float("nan")
    peak_current: float = float("nan")
    reached: bool = False

    alarm_before: str = ""
    alarm_after: str = ""
    alarm_during: int = 0
    alarm_during_ms: float = float("nan")
    cleared: bool = False

    # Controller state for the probed joint.
    target_before: float = float("nan")    # read back just after the step command
    status_before: int = -1
    reached_before: int = -1
    statuses_seen: str = ""                # distinct status words, in order
    stop_ms: float = float("nan")          # when motion last ended
    stop_deg: float = float("nan")
    target_at_stop: float = float("nan")
    status_at_stop: int = -1
    reached_at_stop: int = -1
    target_after: float = float("nan")     # read back immediately after the window
    status_after: int = -1
    reached_after: int = -1
    stop_class: str = ""                   # interpretation, failed steps only
    # reached=1 asserted while the measured angle is still far from the target.
    # A label derived from the raw columns above, which are always kept.
    false_completion: bool = False         # the move ENDED that way
    early_reached_ms: float = float("nan")  # first such sample, even if it recovered
    early_reached_err_deg: float = float("nan")

    # Host-path command following. With --host-rate the probe drives through
    # write_angles at a teleop-like rate limit, so the joint is chased by a
    # moving commanded target instead of being handed one large step.
    host_rate: float = float("nan")        # deg/s ceiling applied by write_angles
    cmd_done_ms: float = float("nan")      # when the commanded target reached the endpoint
    cmd_lag_max: float = float("nan")      # worst commanded-minus-measured gap
    cmd_lag_med: float = float("nan")
    cmd_lag_final: float = float("nan")    # gap at the end of the window

    polls: int = 0
    updates: int = 0                       # angle updates, closing sample excluded
    update_p50_ms: float = float("nan")    # intervals between motion updates only
    update_p95_ms: float = float("nan")


def _alarm_str(codes: Sequence[int]) -> str:
    return ";".join(str(int(c)) for c in codes)


def _read(fn, *args, default=float("nan")):
    try:
        v = fn(*args)
        return default if v is None else v
    except Exception:
        return default


def measure_sampling(hand: Hand, seconds: float = 2.0) -> Dict[str, float]:
    """How often the firmware's angle feedback changes, with the hand at rest."""
    prev = hand.read_angles()
    changes: List[float] = []
    polls = 0
    t0 = time.perf_counter()
    last = t0
    while time.perf_counter() - t0 < seconds:
        now = hand.read_angles()
        polls += 1
        if any(abs(a - b) > 1e-6 for a, b in zip(now, prev)):
            t = time.perf_counter()
            changes.append((t - last) * 1000.0)
            last = t
            prev = now
    elapsed = time.perf_counter() - t0
    out = {"poll_rate_hz": polls / elapsed, "updates": float(len(changes)),
           "update_rate_hz": len(changes) / elapsed if elapsed > 0 else float("nan")}
    if changes:
        out["update_interval_p50_ms"] = float(np.median(changes))
        out["update_interval_p95_ms"] = float(np.percentile(changes, 95))
    return out


class StepProbe:
    def __init__(self, hand: Hand, *, settle_s: float = 1.2, window_s: float = 2.0,
                 poll_hz: float = 500.0, host_hz: float = 30.0) -> None:
        self.hand = hand
        self.settle_s = settle_s
        self.window_s = window_s
        self.poll_dt = 1.0 / poll_hz
        self.host_dt = 1.0 / host_hz
        self.traces: List[dict] = []

    # ------------------------------------------------------------- helpers

    def _safe_target(self, joint: int, value: float) -> float:
        value = self.hand.limits[joint].clamp(float(value))
        if joint == 1:
            value = min(value, thumb_flex_max(self.hand.read_angles()[0]))
        return value

    def _command(self, angles: Sequence[float], drive: str):
        """Issue one command. Returns what was actually commanded, per joint.

        The host path returns write_angles' own output, which is the request
        after clamping and rate limiting -- the instantaneous target the joint
        is chasing. (The stall kick may briefly send more than this; that is
        deliberate and is not reflected here.)
        """
        if drive == "raw":
            for i, a in enumerate(angles):
                self.hand._sdk.set_target_angle(i + 1, float(a))
            self.hand._sdk.move_motors(0)
            return list(angles)
        return list(self.hand.write_angles(list(angles)))

    def _goto(self, angles: Sequence[float], settle: Optional[float] = None) -> None:
        """Un-measured positioning move, always at a velocity known to complete.

        Positioning at the test velocity was a confound: a 0->80 deg move to the
        start of an 'open' step could itself stop part-way, so the open step
        began from wherever that happened to be.
        """
        self.hand._sdk.set_angular_velocity(0, POSITION_VELOCITY)
        self._command(angles, "raw")
        time.sleep(settle if settle is not None else self.settle_s)

    def recover(self) -> bool:
        """Clear alarms and park open after a failed step. True if a clear was needed."""
        had = any(self.hand.alarms())
        if had:
            self.hand.clear_alarms()
            time.sleep(0.3)
        self._goto([l.min_angle for l in self.hand.limits])
        return had

    def _state(self, joint: int) -> tuple:
        sdk = self.hand._sdk
        m = joint + 1
        return (float(_read(sdk.get_now_angle, m)),
                float(_read(sdk.get_target_angle, m)),
                int(_read(sdk.get_position_reached, m, default=-1)),
                int(_read(sdk.get_now_status, m, default=-1)),
                int(_read(sdk.get_now_alarm, m, default=0)),
                float(_read(sdk.get_now_current, m)))

    # ---------------------------------------------------------------- step

    def run_step(self, joint: int, start: float, target: float, *,
                 velocity: float, drive: str = "raw", stall_kick: bool = False,
                 max_current: int = 500) -> StepResult:
        hand = self.hand
        start = self._safe_target(joint, start)
        target = self._safe_target(joint, target)

        base = [lim.min_angle for lim in hand.limits]
        pose_start = list(base); pose_start[joint] = start
        pose_end = list(base); pose_end[joint] = target

        hand._sdk.set_max_current(0, int(max_current))
        hand.stall_kick = stall_kick
        hand._commanded = list(pose_start)
        hand._last_sent = None
        hand._kick_state = {}

        self._goto(pose_start)
        settled_start = hand.read_angles()[joint]
        alarm_before = hand.alarms()

        # The measured step is the only motion at the test velocity.
        hand._sdk.set_angular_velocity(0, float(velocity))
        time.sleep(0.05)

        res = StepResult(joint=joint, joint_name=JOINT_NAMES[joint],
                         direction="close" if target > start else "open",
                         start_deg=settled_start, target_deg=target,
                         step_deg=target - settled_start, velocity_setting=velocity,
                         drive=drive, stall_kick=stall_kick, max_current=max_current,
                         alarm_before=_alarm_str(alarm_before))

        # Samples are kept whenever anything observable changes, not on every
        # poll: angle, target, reached, status or alarm.
        rows: List[tuple] = []  # (t_ms, angle, target, reached, status, alarm, current, cmd)
        polls = 0
        peak_cur = 0.0
        prev_key = None
        # What the host currently asks of this joint. Constant on the raw path;
        # on the host path it ramps toward the endpoint at the rate limit.
        cmd_j = float(pose_start[joint])
        if drive == "host":
            res.host_rate = float(hand.max_deg_per_s)

        # write_angles sizes each step from the time since the previous write,
        # and _goto does not call it. The reset must happen here -- after the
        # settle sleeps, immediately before the first timed command -- or frame
        # one is allowed max_deg_per_s * (settle time), about 375 deg after a
        # 1.2 s settle, which delivers the whole step at once and turns the
        # host path back into the raw step it exists to replace.
        hand._last_write = time.monotonic()
        t0 = time.perf_counter()
        issued = self._command(pose_end, drive)
        if issued is not None:
            cmd_j = float(issued[joint])
        _, res.target_before, res.reached_before, res.status_before, _, _ = self._state(joint)
        next_host = t0 + self.host_dt
        while True:
            now = time.perf_counter()
            elapsed = now - t0
            if elapsed >= self.window_s:
                break
            ang, tgt, rch, sts, alm, cur = self._state(joint)
            polls += 1
            if cur == cur:
                peak_cur = max(peak_cur, cur)
            if alm and not res.alarm_during:
                res.alarm_during = alm
                res.alarm_during_ms = elapsed * 1000.0
            key = (round(ang, 4), round(tgt, 3), rch, sts, alm, round(cmd_j, 2))
            if key != prev_key:
                rows.append((elapsed * 1000.0, ang, tgt, rch, sts, alm, cur, cmd_j))
                prev_key = key
            if drive == "host" and now >= next_host:
                issued = self._command(pose_end, "host")
                if issued is not None:
                    cmd_j = float(issued[joint])
                next_host = now + self.host_dt
            time.sleep(self.poll_dt)

        # Read-back immediately after the window, before anything else touches
        # the bus: the probed joint's state, and every joint's target/status.
        ang, res.target_after, res.reached_after, res.status_after, alm, cur = self._state(joint)
        closing_t = (time.perf_counter() - t0) * 1000.0
        rows.append((closing_t, ang, res.target_after, res.reached_after,
                     res.status_after, alm, cur, cmd_j))
        readback = [{"target": float(_read(hand._sdk.get_target_angle, m)),
                     "status": int(_read(hand._sdk.get_now_status, m, default=-1)),
                     "reached": int(_read(hand._sdk.get_position_reached, m, default=-1)),
                     "angle": float(_read(hand._sdk.get_now_angle, m))}
                    for m in range(1, NUM_JOINTS + 1)]

        res.polls = polls
        res.peak_current = peak_cur
        res.alarm_after = _alarm_str(hand.alarms())
        seen: List[int] = []
        for r in rows:
            if not seen or seen[-1] != r[4]:
                seen.append(r[4])
        res.statuses_seen = ";".join(str(s) for s in seen)

        arr = np.array(rows, dtype=float)
        self._score(res, arr, closing_index=len(rows) - 1)
        self.traces.append({
            "result": asdict(res),
            "t_ms": [round(r[0], 2) for r in rows],
            "angle": [round(r[1], 4) for r in rows],
            "target": [round(r[2], 3) for r in rows],
            "reached": [int(r[3]) for r in rows],
            "status": [int(r[4]) for r in rows],
            "alarm": [int(r[5]) for r in rows],
            "current": [round(r[6], 1) for r in rows],
            "cmd": [round(r[7], 3) for r in rows],
            "closing_index": len(rows) - 1,
            "readback": readback,
        })
        return res

    # --------------------------------------------------------------- score

    def _score(self, res: StepResult, arr: np.ndarray, closing_index: int) -> None:
        if len(arr) < 3:
            return
        t_all, a_all = arr[:, 0], arr[:, 1]

        # Angle updates only: rows added for a target/status change carry a
        # repeated angle, and the closing row is a read-back, not an update.
        body = np.arange(len(arr)) != closing_index
        keep = [i for i in np.where(body)[0]
                if i == 0 or abs(a_all[i] - a_all[i - 1]) > 1e-9]
        t, a = t_all[keep], a_all[keep]
        res.updates = max(0, len(t) - 1)

        # Update intervals during motion only: the gap after settling, or the
        # gap to the closing read-back, measures how long nothing happened.
        if len(t) > 1:
            iv = np.diff(t)
            moving_iv = iv[np.abs(np.diff(a)) > MOTION_DEG]
            if len(moving_iv):
                res.update_p50_ms = float(np.median(moving_iv))
                res.update_p95_ms = float(np.percentile(moving_iv, 95))

        a0 = res.start_deg
        delta = res.target_deg - a0
        if abs(delta) < 1e-6:
            return
        # Timing and progress use every row so a final unchanged value still
        # counts as where the joint ended.
        progress_all = (a_all - a0) / delta

        moved = np.where(np.abs(a - a0) > MOTION_DEG)[0]
        if len(moved):
            res.dead_ms = float(t[moved[0]])
        for frac, name in ((0.5, "t50_ms"), (0.9, "t90_ms")):
            hit = np.where(progress_all >= frac)[0]
            if len(hit):
                setattr(res, name, float(t_all[hit[0]]))

        inside = np.abs(a_all - res.target_deg) <= SETTLE_DEG
        for i in range(len(t_all)):
            if inside[i] and t_all[-1] - t_all[i] >= SETTLE_HOLD_S * 1000 and inside[i:].all():
                res.settle_ms = float(t_all[i])
                break

        if len(t) > 1:
            dt_s = np.diff(t) / 1000.0
            da = np.diff(a)
            ok = dt_s > 0
            v = np.abs(da[ok] / dt_s[ok])
            moving = v[np.abs(da[ok]) > MOTION_DEG]
            if len(moving):
                res.peak_vel_deg_s = float(moving.max())
                res.sustained_vel_deg_s = float(np.median(moving))

        band = np.where((progress_all >= 0.1) & (progress_all <= 0.9))[0]
        if len(band) > 1:
            span = (t_all[band[-1]] - t_all[band[0]]) / 1000.0
            if span > 0:
                res.mean_vel_deg_s = float(abs(a_all[band[-1]] - a_all[band[0]]) / span)

        res.overshoot_deg = float(max(0.0, (progress_all.max() - 1.0) * abs(delta)))
        res.final_deg = float(a_all[-1])
        res.final_err_deg = float(res.target_deg - res.final_deg)
        res.reached = bool(abs(res.final_err_deg) <= SETTLE_DEG)

        # When did motion end? The earliest row after which the angle never
        # moves more than MOTION_DEG again. State is taken from that row.
        stop = len(a_all) - 1
        for i in range(len(a_all)):
            if np.all(np.abs(a_all[i:] - a_all[i]) <= MOTION_DEG):
                stop = i
                break
        # Command-following: how far the host's instantaneous target ran ahead
        # of the joint. On the raw path this is just the step itself.
        cmd = arr[:, 7]
        lag = np.abs(cmd - a_all)
        res.cmd_lag_max = float(lag.max())
        res.cmd_lag_med = float(np.median(lag))
        res.cmd_lag_final = float(lag[-1])
        done = np.where(np.abs(cmd - res.target_deg) <= 0.05)[0]
        if len(done):
            res.cmd_done_ms = float(t_all[done[0]])

        res.stop_ms = float(t_all[stop])
        res.stop_deg = float(a_all[stop])
        res.target_at_stop = float(arr[stop, 2])
        res.reached_at_stop = int(arr[stop, 3])
        res.status_at_stop = int(arr[stop, 4])

        # Any sample, after the move has begun, where the drive asserts reached
        # while the joint is outside the settle band. Before the move begins,
        # reached=1 is just the previous move's state and is not counted.
        rch = arr[:, 3]
        began = np.where(rch == 0)[0]
        if len(began):
            err_all = np.abs(res.target_deg - a_all)
            early = [i for i in range(began[0], len(arr))
                     if rch[i] == 1 and err_all[i] > SETTLE_DEG]
            if early:
                res.early_reached_ms = float(t_all[early[0]])
                res.early_reached_err_deg = float(err_all[early[0]])

        if not res.reached:
            res.false_completion = bool(res.reached_after == 1)
            res.stop_class = classify_stop(res)


def classify_stop(r: StepResult) -> str:
    """Interpretive label for a failed step. The raw columns remain the evidence."""
    requested = r.target_deg
    tgt = r.target_after if r.target_after == r.target_after else r.target_at_stop
    err = abs(requested - r.final_deg)
    if tgt != tgt:
        target_note = "target unreadable"
    elif abs(tgt - requested) <= TARGET_MATCH_DEG:
        target_note = "read-back target unchanged"
    elif abs(tgt - r.final_deg) <= TARGET_MATCH_DEG:
        target_note = "read-back target replaced by the stopped position"
    else:
        target_note = f"read-back target changed to {tgt:.1f}"

    if r.reached_after == 1 and err > SETTLE_DEG:
        # Checked first: a drive that asserts reached is not still chasing the
        # target, whatever the (possibly cached) target read-back says.
        kind = f"early/false completion: reached=1 while {err:.1f} deg from target"
    elif tgt == tgt and abs(tgt - requested) <= TARGET_MATCH_DEG:
        kind = "stopped short, reached not asserted"
    else:
        kind = "stopped short"
    status = f"status {r.status_before}->{r.status_at_stop}->{r.status_after}"
    return f"{kind}; {target_note}; {status}"


def save(results: List[StepResult], traces: List[dict], csv_path: Path,
         trace_path: Optional[Path] = None) -> None:
    if not results:
        return
    cols = list(asdict(results[0]).keys())

    def fmt(v):
        if isinstance(v, float):
            return f"{v:.3f}"
        return str(v).replace(",", ";")

    with open(csv_path, "w") as fh:
        fh.write(",".join(cols) + "\n")
        for r in results:
            d = asdict(r)
            fh.write(",".join(fmt(d[c]) for c in cols) + "\n")
    if trace_path is not None:
        Path(trace_path).write_text(json.dumps(traces))


def summarize(results: List[StepResult], sampling: Optional[Dict[str, float]] = None,
              traces: Optional[List[dict]] = None) -> str:
    if not results:
        return "no results"
    out: List[str] = []
    ok = [r for r in results if r.updates >= 2]

    out.append("")
    out.append("feedback resolution (motion updates only)")
    if sampling and "update_interval_p50_ms" in sampling:
        out.append(f"  at rest:      p50 {sampling['update_interval_p50_ms']:.1f} ms "
                   f"({sampling.get('update_rate_hz', float('nan')):.1f} Hz)")
    p50 = [r.update_p50_ms for r in ok if r.update_p50_ms == r.update_p50_ms]
    p95 = [r.update_p95_ms for r in ok if r.update_p95_ms == r.update_p95_ms]
    if p50:
        out.append(f"  during moves: p50 {np.median(p50):.1f} ms, worst step p95 {max(p95):.1f} ms, "
                   f"median {np.median([r.updates for r in ok]):.0f} updates per step")
        out.append(f"  => timings finer than ~{np.median(p50):.0f} ms are NOT resolved.")

    out.append("")
    out.append("per-step results")
    out.append(f"  {'dir':>5}{'step':>6}{'vset':>5}{'upd':>5}{'t90':>6}{'peakv':>7}{'sust':>6}"
               f"{'err':>7}{'cur':>5}{'alm':>5}{'tgt0':>7}{'stat0':>6}{'stop':>6}"
               f"{'tgt@st':>7}{'st@st':>6}{'tgtEnd':>7}{'stEnd':>6}{'rchEnd':>7}{'ok':>4}")
    out.append("  " + "-" * 118)
    for r in ok:
        out.append(f"  {r.direction:>5}{r.step_deg:>6.0f}{r.velocity_setting:>5.0f}{r.updates:>5}"
                   f"{r.t90_ms:>6.0f}{r.peak_vel_deg_s:>7.0f}{r.sustained_vel_deg_s:>6.0f}"
                   f"{r.final_err_deg:>7.1f}{r.peak_current:>5.0f}{r.alarm_during:>5}"
                   f"{r.target_before:>7.1f}{r.status_before:>6}{r.stop_ms:>6.0f}"
                   f"{r.target_at_stop:>7.1f}{r.status_at_stop:>6}{r.target_after:>7.1f}"
                   f"{r.status_after:>6}{r.reached_after:>7}{'Y' if r.reached else 'NO':>4}")

    fails = [r for r in results if not r.reached]
    early = [r for r in results if r.early_reached_ms == r.early_reached_ms]
    out.append("")
    out.append(f"early/false completion events: {sum(r.false_completion for r in results)} terminal, "
               f"{sum(1 for r in early if not r.false_completion)} transient (reached=1 outside the "
               f"{SETTLE_DEG:.0f} deg band, then recovered)")
    if fails:
        out.append("")
        out.append(f"FAILED STEPS ({len(fails)})")
        for r in fails:
            out.append(f"  {r.joint_name} {r.direction} {r.step_deg:+.0f} deg at v={r.velocity_setting:.0f}"
                       f": stopped at {r.stop_deg:.1f} deg, {r.stop_ms:.0f} ms, "
                       f"{r.final_err_deg:+.1f} deg short")
            out.append(f"     target  sent {r.target_deg:.1f} | read after command {r.target_before:.1f}"
                       f" | at stop {r.target_at_stop:.1f} | after window {r.target_after:.1f}")
            out.append(f"     status  after command {r.status_before} | at stop {r.status_at_stop}"
                       f" | after window {r.status_after} | sequence [{r.statuses_seen}]")
            out.append(f"     reached after command {r.reached_before} | at stop {r.reached_at_stop}"
                       f" | after window {r.reached_after}")
            out.append(f"     alarms  before [{r.alarm_before}] after [{r.alarm_after}]"
                       + (f" during {r.alarm_during} at {r.alarm_during_ms:.0f} ms"
                          if r.alarm_during else " none during"))
            out.append(f"     current peak {r.peak_current:.0f} of {r.max_current}")
            out.append(f"     => {r.stop_class}")
        out.append("  NOTE: get_target_angle may echo the host's cached write. An unchanged")
        out.append("  target is weak evidence; a changed one is strong evidence of replacement.")

    def grouped(key):
        g: Dict[object, List[StepResult]] = {}
        for r in ok:
            g.setdefault(key(r), []).append(r)
        return g

    out.append("")
    out.append("velocity by setting and step")
    out.append(f"  {'v_set':>6}{'step':>6}{'n':>4}{'peak':>7}{'sust':>7}{'t90':>7}{'reached':>9}")
    for (v, s), rs in sorted(grouped(lambda r: (r.velocity_setting,
                                               round(abs(r.target_deg - r.start_deg if r.direction == 'close'
                                                         else r.start_deg - r.target_deg) / 10) * 10)).items()):
        pk = [r.peak_vel_deg_s for r in rs if r.peak_vel_deg_s == r.peak_vel_deg_s]
        su = [r.sustained_vel_deg_s for r in rs if r.sustained_vel_deg_s == r.sustained_vel_deg_s]
        t9 = [r.t90_ms for r in rs if r.t90_ms == r.t90_ms]
        out.append(f"  {v:>6.0f}{s:>6.0f}{len(rs):>4}"
                   f"{(np.median(pk) if pk else float('nan')):>7.0f}"
                   f"{(np.median(su) if su else float('nan')):>7.0f}"
                   f"{(np.median(t9) if t9 else float('nan')):>7.0f}"
                   f"{sum(r.reached for r in rs):>5}/{len(rs):<3}")

    out.append("")
    out.append("controller state on successful steps (for comparison with failures)")
    succ = [r for r in ok if r.reached]
    if succ:
        seqs: Dict[str, int] = {}
        for r in succ:
            k = (f"status after cmd {r.status_before} -> seq [{r.statuses_seen}] -> end {r.status_after},"
                 f" reached end {r.reached_after}, target end matches request "
                 f"{abs(r.target_after - r.target_deg) <= TARGET_MATCH_DEG}")
            seqs[k] = seqs.get(k, 0) + 1
        for k, c in sorted(seqs.items(), key=lambda kv: -kv[1]):
            out.append(f"  {c:>3} x  {k}")
    return "\n".join(out)
