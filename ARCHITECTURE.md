# Architecture

## The stable path

```
              Camera (threaded V4L2, newest frame only)
                 │  BGR frame + capture timestamp
                 ▼
          HandTracker.process()            tracker.py
                 │  HandReading: 21 world landmarks (m), 21 image landmarks (px),
                 │               6 scalar signals (deg), handedness label
                 ▼
          FKRetargeter.__call__()          fkretarget.py
                 │  fingertip-vector targets in the palm frame
                 │  damped Gauss-Newton over the DH116 kinematics (kinematics.py)
                 │  EMA smoothing (--smoothing) + deadband (--deadband)
                 │  6 firmware angles
                 ▼
          teleop loop                      teleop.py  (cmd_teleop / compute_angles)
                 │  thumb-clearing gate, tactile pinch closer (fk mode)
                 ▼
          Hand.write_angles()              hand.py
                 │  clamp to firmware limits → host rate limit (--max-speed)
                 │  → stall kick → set_target_angle ×6 + move_motors
                 ▼
          RS485 @ 500 kbaud  (sdk.py; --no-serial-flush patches out the vendor tcdrain)
                 ▼
               DH116
```

Everything above the hand runs once per camera frame (~30 Hz). `latency.py`
timestamps each stage; `CAMERA->RS485` = frame_age + track + retarget + write.

## The visualisation side path

```
   HandReading ──▶ PhantomHand.render()   phantom.py     ──▶ third column of the window
                      │
                      ├─ Canonicalizer            handpose.py   chirality from the geometry
                      ├─ EMA on canonical points               (label flicker cannot flip it)
                      ├─ HandPose3D.from_landmarks (joint frames, when --show-joint-frames)
                      └─ 2.5-D projection, painter's ordering, fog, floor shadow
```

It runs **after** `write_angles()` in the same iteration, owns its own state,
returns only an image, and `render()` never raises (a failing frame yields a
placeholder panel and is counted). `tests/test_control_replay.py` asserts that
rendering it between frames leaves every commanded angle bit-identical.

`handpose.py` is a geometry module (canonical right-hand landmarks, palm frame,
minimal-rotation chain frames, wrist-local expression — maths in
`docs/FRAMES.md`). It is stable and tested; on the stable path it is used
only by the phantom view.

## Safety layers

| layer | where | default |
|---|---|---|
| firmware joint limits, read at connect | `Hand._read_limits` | — |
| host rate limit | `Hand.write_angles` | 180 °/s (`--max-speed`) |
| firmware velocity | `Hand.connect` | 150 °/s (see docs/EXPERIMENTS.md before raising) |
| current limit | `Hand.connect` | 500 ‰ (`--max-current`) |
| stall kick | `Hand._apply_stall_kick` | on |
| start paused, hold last pose on tracking loss | `cmd_teleop` | — |
| one process holds the hardware | `session.py` (`.telehand.lock`) | — |
| stop + de-energize on every exit path | `Hand.disconnect` | on (`--keep-enabled` opts out) |

## Experimental components

`telehand/experimental/` is imported by the stable path only through
`experimental/cli.py`, a light module that registers the `stepresponse` and
`frames` subcommands; the modules behind them load when those commands run.
`--pose-mode frames` / `--pose-log` import `framemap` lazily inside
`cmd_teleop`. Importing `telehand.teleop` therefore loads no experimental
module beyond `cli` (checked in the cleanup verification).

```
experimental/framemap.py      FrameMapper: thumb/index joints from HandPose3D, FK fallback
experimental/frameviz.py      FrameStats + replay renderer for `run.py frames`
experimental/stepresponse.py  StepProbe: per-joint step response with controller-state logging
experimental/cli.py           subcommands
```

## Where things are written

| what | where | tracked |
|---|---|---|
| pose library, tactile maps, envelope | `poses.json`, `touch_grid.json`, `abd_envelope.json` | yes |
| per-machine state | `.camera.json`, `.handview_view.json`, `.telehand.lock`, `*calibration.json`, `handprofile*.json` | no |
| profiles, recordings, benchmarks, logs | `data/` | no |
| test fixtures | `tests/data/*.npz` | yes |
