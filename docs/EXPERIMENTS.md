# Experiments and findings (2026-09-15/16)

Short record of what the experimental tools measured on this rig, so the
numbers behind the recommended settings are not lost. Raw outputs are in
`data/benchmarks` and `data/logs` (git-ignored); `tools/` summarises them.

## Host latency (`--latency-csv`, `run.py latency`)

Baseline command path (camera → RS485) was 44 ms p50 at 27 fps. Two findings:

- **Smoothing was filtering noise that isn't there.** Solver output jitter is
  0.17° at `--smoothing 0.35` and 0.30° with smoothing off — both under the
  0.8° deadband. `--smoothing 0.7` cuts EMA lag from 63 ms to 15 ms for free.
- **`write` cost 12 ms regardless of how many joints were sent.** The vendor
  serial layer calls `tcdrain` after every write; removing it
  (`--no-serial-flush`, patched at runtime in `sdk.py`, ordering preserved by
  the write lock and the tty FIFO) took the command path from 44.3 to 30.7 ms
  p50 and restored 29.4 fps. Sending only changed joints (`--skip-unchanged`)
  gained ~3 % and is kept optional. Reading angles is free (served from the
  monitor thread's cache): removing six reads per frame changed nothing.

## Actuator characterisation (`run.py stepresponse`)

- Feedback resolution is **64 ms per update** (~16 Hz), consistent with the
  FTDI `latency_timer` at its 16 ms default; dead time and fine t50/t90
  differences are under-resolved. `latency_timer` needs root to change.
- Firmware velocity 150 is binding: index/middle sustain ~140 °/s at 150 and
  ~185 °/s at 200 (+31 %), completing 80° moves in ~600 vs ~700 ms. At 250 an
  80° move on joint 2 **stops at ~40° every time**.
- Those stops are **early/false completions**: the drive asserts `reached=1`
  with the joint 21–41° short, status runs its normal 0→1→0, current drops to
  idle, no alarm code. Not torque-limited (≤ 300 of 500 ‰).
- Under teleop-style incremental commanding (`--drive host --host-rate 300`)
  the finger failures did not reproduce, but the **thumb ones did, identically**:
  thumb abduction stops at 22.56° of a 60° move and thumb flexion at ~9° of
  30°, at 150 °/s, with or without the stall kick. `FIRMWARE_MAX_DEG` commands
  the thumb into that region routinely.
- Pinky at 200 froze mid-close for ~1 s on every 80° move before completing.
- Per-joint conclusion: index/middle would be clean at 200; ring, pinky and
  thumbs should stay at 150. **Teleop still sets 150 for all joints**; per-joint
  firmware velocities were not adopted.

## Hand frames (`run.py frames`, `docs/FRAMES.md`)

- MediaPipe's handedness label is unreliable per frame in both directions;
  chirality is therefore decided from the geometry (curled fingertips lie on
  the palmar side), with 0 disagreements over 533 confident frames.
- Reconstructed frames: det = +1 to 1e-15, rigid-transform invariant to 1e-14,
  bend angles equal the existing `_angle_between` to 1e-13, no 180° flips
  under plausible input, jitter p50 ≈ 1–2°.

## Frame-based control (`--pose-mode frames`)

Thumb abduction/flexion and index flexion from the frames, FK for the rest,
with FK fallback on implausible geometry. Offline, the frame-mapped commands
were not steadier than the FK solver's (0.51/0.33/0.46° vs 0.46/0.16/0.30°
high-frequency residual); live, it felt less stable. Kept for comparison,
not recommended.
