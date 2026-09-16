# telehand

Camera-driven teleoperation of the DH116 (LHandPro LAC_DOF_6) dexterous hand.
Hold your right hand up to a webcam; the robot hand copies it.

```
webcam ──▶ MediaPipe HandLandmarker ──▶ FK retargeter ──▶ clamp / rate-limit / stall kick ──▶ RS485
           21 world landmarks           6 joint angles      Hand.write_angles()               DH116
                     │
                     └──▶ phantom 3-D hand view (visualisation only, never touches control)
```

The stable path is **scalar FK teleop**: `--mode fk`. Everything under
`telehand/experimental/` is research tooling and is neither imported nor
constructed by that path unless an explicitly experimental flag or subcommand
asks for it. See [ARCHITECTURE.md](ARCHITECTURE.md) for the data flow and
[docs/EXPERIMENTS.md](docs/EXPERIMENTS.md) for what the experiments found.

Stable baseline: git tag `stable-scalar-fk-2026-09-16`.

## Hardware and software assumptions

| | |
|---|---|
| Hand | DH116 / LHandPro `LAC_DOF_6`, **right** hand, 6 active DoF, RS485 @ 500 000 baud on `/dev/ttyUSB0` (FTDI FT231X), node id 1 |
| Joint limits | read from the firmware at startup: thumb abduction 0–60°, thumb flexion 0–30°, four finger flexions 0–80° |
| Camera | any V4L2 webcam at 640×480 MJPG ~30 fps; `run.py cameras` lists them, `--camera` accepts an index, `/dev` node, `by-path` link or name fragment |
| Operator | one **right** hand, seen through a mirrored (selfie) camera image (`--mirror`, on by default) |
| Python | 3.12, no system packages: dependencies live in the project-local `vendor/` (mediapipe 1.0 Tasks API, opencv, numpy, scipy, pyserial, mujoco) |
| Display | an X display for the OpenCV window (`export DISPLAY=:1` when working over SSH) |

This checkout is a parallel copy of the original project. The large read-only
trees are **symlinks** to it and are not tracked:

```
vendor/  models/  LHandProLib-API-Linux-20260727/  DH116_URDF_MJCF_Files/  DH116_URDF_Files_2025_11_24/
```

`run.py` puts `vendor/` (and the Pinocchio `cmeel` layout inside it) on
`sys.path`; nothing is installed system-wide.

## Running teleop (stable, recommended)

```bash
export DISPLAY=:1 && cd ~/telehand-lat
python3 run.py teleop --mode fk --smoothing 0.7 --deadband 0.4 --max-speed 300 --no-serial-flush
```

With the phantom 3-D hand view beside the camera image:

```bash
python3 run.py teleop --mode fk --show-3d-pose --smoothing 0.7 --deadband 0.4 --max-speed 300 --no-serial-flush
```

Add `--show-joint-frames` to draw every articulated joint's local XYZ frame on
the phantom, `--latency-csv lat.csv` to keep the per-frame profile.

The hand **homes first** (drives every joint through its range — keep it clear),
then teleop starts **paused**. Keys in the window:

| key | |
|---|---|
| `SPACE` | engage / pause |
| `o` | open the hand and disengage |
| `c` | clear alarms |
| `p` / `f` | toggle the phantom view / its joint frames |
| `[` `]` | rotate the phantom view |
| drag, wheel, `hjkl`, `r`, `v` | orbit / zoom / reset / save the robot view |
| `q` or `Esc` | quit |

Why those settings: measured on this rig, `--smoothing 0.7` removes 48 ms of
EMA lag while output jitter stays far below the deadband; `--no-serial-flush`
removes the vendor's per-write `tcdrain` (−27 % command latency, ordering
preserved); `--max-speed 300` opens the host rate limit so the firmware
velocity is what governs. `--mode fk` is the default, so `--mode fk` may be
omitted.

Before the first session on a machine:

```bash
python3 run.py check                  # camera, tracking, hand link; never moves the hand
python3 run.py cameras --use icspring # remember the camera
python3 run.py profile --name you     # ~10 s, no motors: per-finger scaling and pinch gap
```

`run.py stop` ends a running session cleanly from another terminal (motors
stopped, de-energized, serial closed). Only one session can hold the hardware;
a second one is refused.

## Latency profiling

Every teleop run prints a p50/p95/p99 table of per-stage timings at exit:
`wait`, `frame_age`, `track` (MediaPipe), `retarget`, `write` (RS485),
`3D hand view` (when on), `display`, `loop`, and the command path
`CAMERA->RS485`. `--latency-csv PATH` keeps the per-frame rows;
`run.py latency PATH` re-prints the summary later. Reference numbers for the
recommended settings: command path ≈ 30 ms p50, 29 fps. Host-side only — the
camera's own exposure/transfer delay and the motor's travel time (200–530 ms
for a full 80° finger close) are not in it.

## Safety

- Every command is clamped to the limits the firmware reports, rate-limited
  host-side (`--max-speed`), and current-limited (`--max-current`, 500 ‰).
- The stall kick briefly overshoots a worm-drive joint parked short of its
  target; `--no-stall-kick` disables it.
- Teleop starts paused; losing tracking holds the last pose.
- Quitting, `Ctrl+C`, `SIGTERM`, `run.py stop` or any error stops the motors,
  **de-energizes** them and closes the bus. Pulling the USB cable is not a
  substitute: it carries data only, and the motors would keep holding.
  `--keep-enabled` opts out of de-energizing.
- `--dry-run` connects and tracks but never enables or moves the motors.
- Known hardware behaviour (see docs/EXPERIMENTS.md): thumb abduction sticks
  on the way back; large thumb moves stop short (~22° of a 60° abduction,
  ~9° of a 30° flexion) and the firmware reports them as reached.

## Layout

```
run.py                      entry point (vendor/ on sys.path)
telehand/                   stable package
  teleop.py                 CLI and the teleop loop
  tracker.py                MediaPipe -> 21 world landmarks + 6 scalar signals
  fkretarget.py             the FK retargeter (fingertip geometry -> 6 angles) + pinch closer
  kinematics.py             DH116 URDF geometry, coupling, measured drive curves
  hand.py                   clamp, rate limit, stall kick, RS485; de-energize on exit
  sdk.py                    vendor SDK loader; runtime --no-serial-flush patch
  session.py                one-instance lock, clean stop
  latency.py                per-stage profiler
  viz.py, handview.py       robot UI panel (MuJoCo / hull render of the commanded pose)
  phantom.py                phantom 3-D hand view (visualisation only)
  handpose.py               canonical right-hand landmarks + wrist-local joint frames (used by phantom)
  poses.py, direct.py       alternative retargeters (--mode poses / direct / hybrid)
  profile.py, touchscan.py  operator profile; tactile contact mapping
  experimental/             not on the stable path (see docs/EXPERIMENTS.md)
    framemap.py             frame-based joint mapping   (--pose-mode frames)
    frameviz.py             hand-frame replay viewer     (run.py frames)
    stepresponse.py         actuator characterisation    (run.py stepresponse, moves the hand)
    cli.py                  their subcommands
tests/                      python3 tests/test_*.py  (no hardware; fixtures in tests/data/)
tools/                      summaries over benchmark outputs in data/benchmarks
docs/                       ARCHITECTURE details, FRAMES.md (frame maths), EXPERIMENTS.md
data/                       generated CSVs, traces, logs, screenshots (git-ignored)
poses.json, touch_grid.json, abd_envelope.json   measured calibration data (tracked)
```

Run the tests:

```bash
for t in tests/test_*.py; do python3 "$t" || break; done
```

`tests/test_control_replay.py` is the regression guard: the FK retargeter's
output on a bundled recording is frozen in `tests/data/control_reference.npz`,
and any change to the control path that alters a commanded angle fails there
before it can reach the hand.

## Experimental tools

Isolated under `telehand/experimental/`, labelled `[experimental]` in
`run.py --help`, and never loaded by plain teleop:

- `run.py teleop --pose-mode frames` — frame-based control of thumb and index
  (kept for comparison; it felt less stable than the FK solver).
- `run.py frames --replay rec.npz` — replay a `--record` recording as
  wrist-local joint frames with stability metrics.
- `run.py stepresponse` — **moves the hand**; measures per-joint step response.
- `--skip-unchanged` — fewer serial transactions per frame (~3 % gain).
- `tools/` — summaries over the benchmark outputs in `data/benchmarks`.
