"""Per-stage latency instrumentation for the teleop loop.

The point of this module is to answer one question with numbers rather than
feel: when you move your finger, how long until the motor is asked to move,
and which stage ate the time?

Stages are timed with ``time.perf_counter`` (~50 ns per call), so leaving the
timing on costs nothing measurable -- roughly 1 us per frame against a 33 ms
frame period. Only the CSV write is optional.

``frame_age`` is how long a frame sat between the capture thread receiving it
and the loop consuming it. Note what it does *not* include: the stamp is taken
after ``VideoCapture.read()`` returns, so exposure, USB transfer and MJPG
decode have already happened. Camera-internal latency cannot be measured from
inside the process at all -- that needs an external reference, such as filming
a millisecond clock together with the hand.

So ``frame_age`` near zero means the queue is healthy, not that the camera is
instant. Read it together with ``wait``: a long wait and a fresh frame means
the host has headroom and the camera is pacing the loop; a short wait and a
stale frame means the host is the bottleneck and frames are backing up.
"""

from __future__ import annotations

import contextlib
import time
from pathlib import Path
from typing import Dict, List, Optional

# Ordered for display. `loop` is the whole iteration; the rest are its parts.
# `wait` and `frame_age` together say who is the constraint: a long wait with
# fresh frames means the host has headroom and the camera paces us; a short
# wait with stale frames means the host is the bottleneck and frames are
# queueing behind it.
STAGES = ["wait", "frame_age", "track", "retarget", "write", "overlay", "display", "loop"]

# What the operator actually feels: photons in the camera to bytes on RS485.
# Display cost is deliberately excluded -- it happens after the command is sent.
COMMAND_PATH = ["frame_age", "track", "retarget", "write"]


class FrameTimer:
    """Stage timings for one frame.

    Used as a context manager per stage::

        with timer.stage("track"):
            reading = tracker.process(frame, ts)
    """

    __slots__ = ("times",)

    def __init__(self) -> None:
        self.times: Dict[str, float] = {}

    @contextlib.contextmanager
    def stage(self, name: str):
        t0 = time.perf_counter()
        try:
            yield
        finally:
            # += so a stage entered twice in one frame accumulates rather than
            # overwriting; `retarget` can be re-entered by the pinch closer.
            self.times[name] = self.times.get(name, 0.0) + (time.perf_counter() - t0) * 1000.0

    def mark(self, name: str, ms: float) -> None:
        """Record a duration measured elsewhere (e.g. frame age)."""
        self.times[name] = float(ms)

    @property
    def command_latency(self) -> float:
        return sum(self.times.get(s, 0.0) for s in COMMAND_PATH)


def _pct(values: List[float], q: float) -> float:
    """Percentile without numpy, so the summary works on any recorded CSV."""
    if not values:
        return float("nan")
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    pos = (len(s) - 1) * q
    lo = int(pos)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (pos - lo)


class LatencyLog:
    """Collects per-frame stage timings; writes CSV and prints a summary.

    Rows are held in memory as well as streamed to disk so the summary works
    whether or not a CSV path was given. A teleop session is minutes long at
    30 Hz, so a few thousand rows of floats is nothing.
    """

    def __init__(self, path: Optional[Path] = None, extra: Optional[List[str]] = None) -> None:
        self.extra = list(extra or [])
        self.columns = ["t", "frame"] + STAGES + ["command_latency"] + self.extra
        self.rows: List[Dict[str, float]] = []
        self._fh = None
        if path is not None:
            self._fh = open(path, "w")
            self._fh.write(",".join(self.columns) + "\n")
        self.path = path

    def add(self, t: float, frame_no: int, timer: FrameTimer, **extra) -> None:
        row = {"t": t, "frame": float(frame_no), "command_latency": timer.command_latency}
        for s in STAGES:
            row[s] = timer.times.get(s, float("nan"))
        for k in self.extra:
            row[k] = float(extra.get(k, float("nan")))
        self.rows.append(row)
        if self._fh is not None:
            self._fh.write(",".join(f"{row[c]:.3f}" for c in self.columns) + "\n")

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None

    # ------------------------------------------------------------- summary

    def summary(self) -> str:
        return format_summary(self.rows, len(self.rows))


def format_summary(rows: List[Dict[str, float]], n_frames: int) -> str:
    """Render the p50/p95 table. Shared by live runs and `run.py latency`."""
    if not rows:
        return "latency: no frames recorded"

    def col(name):
        return [r[name] for r in rows
                if name in r and r[name] == r[name]]   # drop NaN

    out = []
    out.append("")
    out.append(f"latency over {n_frames} frames (ms)")
    out.append(f"  {'stage':<16}{'p50':>8}{'p95':>8}{'p99':>8}{'max':>8}")
    out.append("  " + "-" * 48)
    for s in STAGES:
        v = col(s)
        if not v:
            continue
        label = s
        if s == "loop":
            out.append("  " + "-" * 48)
            label = "loop (period)"
        elif s == "wait":
            label = "wait for frame"
        elif s == "overlay":
            label = "3D hand view"
        out.append(f"  {label:<16}{_pct(v, .50):>8.1f}{_pct(v, .95):>8.1f}"
                   f"{_pct(v, .99):>8.1f}{max(v):>8.1f}")

    cmd = col("command_latency")
    if cmd:
        out.append("  " + "-" * 48)
        out.append(f"  {'CAMERA->RS485':<16}{_pct(cmd, .50):>8.1f}{_pct(cmd, .95):>8.1f}"
                   f"{_pct(cmd, .99):>8.1f}{max(cmd):>8.1f}")

    # Serial transactions per frame: what optimization A actually saves.
    tx = col("tx")
    if tx:
        engaged = [r["tx"] for r in rows
                   if r.get("engaged", 0) == 1 and "tx" in r and r["tx"] == r["tx"]]
        pool = engaged or tx
        out.append("")
        out.append(f"  serial transactions/frame (engaged): p50 {_pct(pool, .50):.0f}"
                   f"  mean {sum(pool) / len(pool):.2f}  max {max(pool):.0f}"
                   f"   total {sum(tx):.0f}")
        out.append(f"  (baseline is 7: six set_target_angle + one move_motors)")

    # Rate from elapsed wall-clock, not from median loop: the loop period is
    # bimodal once serial stalls enter the picture, and a median then overstates
    # the real rate (33.2 fps reported against 29.3 actually delivered).
    times = col("t")
    loop = col("loop")
    if len(times) > 1 and times[-1] > times[0]:
        span = times[-1] - times[0]
        out.append("")
        out.append(f"  frame rate: {(len(times) - 1) / span:.1f} fps "
                   f"({len(times)} frames in {span:.1f} s)")
        if loop:
            out.append(f"  loop period: median {_pct(loop, .50):.1f} ms, "
                       f"mean {sum(loop) / len(loop):.1f} ms"
                       + ("   <- bimodal: medians flatter than the mean here"
                          if sum(loop) / len(loop) > _pct(loop, .50) * 1.1 else ""))

    # The command path is what the operator feels; name the biggest contributor
    # so the number comes with a direction to act in.
    parts = [(s, _pct(col(s), .50)) for s in COMMAND_PATH if col(s)]
    if parts:
        worst, worst_ms = max(parts, key=lambda p: p[1])
        total = sum(p[1] for p in parts)
        if total > 0:
            out.append(f"  dominant stage: {worst} ({worst_ms:.1f} ms, "
                       f"{100 * worst_ms / total:.0f}% of the command path)")
    out.append("")
    out.append("  NOTE: this measures host-side latency only. The motor still has")
    out.append("  to travel: at 150 deg/s a full 80 deg finger close adds 533 ms.")
    return "\n".join(out)


def summarize_csv(path: Path) -> str:
    """Re-summarize a CSV written by a previous run."""
    path = Path(path)
    lines = path.read_text().strip().splitlines()
    if len(lines) < 2:
        return f"{path}: no data rows"
    header = lines[0].split(",")
    rows = []
    for line in lines[1:]:
        vals = line.split(",")
        if len(vals) != len(header):
            continue
        row = {}
        for k, v in zip(header, vals):
            try:
                row[k] = float(v)
            except ValueError:
                pass
        rows.append(row)
    return format_summary(rows, len(rows))
