# telehand

Camera-driven teleoperation for the LHandPro 6-DoF dexterous hand.

Hold your hand up to the webcam; the robot hand copies it.

```
webcam ──▶ MediaPipe HandLandmarker ──▶ 6 raw signals ──▶ calibrated
           (21 world landmarks)          (degrees)         retarget
                                                              │
                                       RS485 @ 500 kbaud ◀────┘
                                       /dev/ttyUSB0, node 1
```

## Detected hardware

Probed on this machine, not assumed:

| | |
|---|---|
| Hand | `/dev/ttyUSB0` (FTDI FT231X), RS485 @ 500000 baud, node id 1 |
| Model | `LAC_DOF_6` — 6 active DoF of 11 total, **right** hand |
| Camera | **`/dev/video2` icSpring** (default) — 640x480 MJPG, measured 29.8 fps |
| Camera (alt) | `/dev/video0` Innomaker U20CAM-1080p — 29.5 fps |

`/dev/ttyACM0` and `/dev/ttyACM2` (CH343 serial) and `/dev/ttyACM1` (Paxini
tactile-sensor adapter) are also present but do not answer the LHandPro
protocol and stream nothing on their own — they are something else.

Joint limits, read from the firmware at startup rather than hardcoded:

| id | joint | range |
|---|---|---|
| 1 | thumb_abduction | 0–60° |
| 2 | thumb_flexion | 0–30° |
| 3 | index_flexion | 0–80° |
| 4 | middle_flexion | 0–80° |
| 5 | ring_flexion | 0–80° |
| 6 | pinky_flexion | 0–80° |

## Setup

Already done in this directory, but to reproduce: this machine has no `pip`,
no `venv`, and `sudo` needs a password, so dependencies live in a project-local
`vendor/` directory installed via pip's standalone zipapp.

```bash
curl -sSL -o scripts/pip.pyz https://bootstrap.pypa.io/pip/pip.pyz
python3 scripts/pip.pyz install --target vendor mediapipe opencv-python numpy pyserial
curl -sSL -o models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
```

The MuJoCo hand view additionally needs `mujoco`, `glfw` and `PyOpenGL`;
without them the panel falls back to a convex-hull renderer. Nothing is
installed system-wide. `rm -rf vendor scripts/pip.pyz` undoes all of it.

`run.py` puts `vendor/` on `sys.path` before importing anything
([bootstrap.py](telehand/bootstrap.py)). A standalone script does the same by
calling `telehand.bootstrap.bootstrap()` first; see
[scripts/tactile_probe.py](scripts/tactile_probe.py) for the pattern.

## Use

**1. Check everything, without moving the hand:**

```bash
python3 run.py check
```

Opens a preview window showing the tracked skeleton and running detection rate.
Reports camera, how many frames found a hand, the live joint limits, current
angles and any alarms. Safe to run any time — it never commands motion.

If it reports a different hand than you expected: MediaPipe labels handedness
assuming a mirrored, selfie-style view, which `--mirror` (on by default)
provides — so the label matches the hand you actually held up. The robot is a
**right** hand; driving it with your left works (only per-finger curl is used,
which has no handedness) but the thumb feels mirrored. Use `--hand Left` to
track your left hand deliberately.

**2. Calibrate to your hand** (recommended; takes about 20 seconds):

```bash
python3 run.py calibrate
```

It walks through the pose plan (open, the four thumb-to-finger pinches, fist):
hold each pose and press SPACE. The tracker signals for every pose are stored
in `calibration/poses.json` beside the robot joint angles for that pose, and
the retargeter blends between them. It refuses to save when two poses look too
alike to the tracker.

**3. Teleoperate:**

```bash
python3 run.py teleop
```

The hand **homes first** — it drives every joint through its full range to find
its reference. Make sure it is clear of obstructions and not gripping anything.

Teleop starts **paused**. Press SPACE in the video window to engage.

The window is split: your tracked hand on the left, the robot hand's state on
the right.

```
+---------------------------+--------------------------+
|                           | ENGAGED         29.8 fps |
|   camera + tracked        |                          |
|   hand skeleton           |   COMMANDED POSE         |
|                           |     (schematic hand,     |
|                           |      curls as you curl)  |
|                           |                          |
|                           | thumb abduction   53/60  |
|                           | ################----     |
|                           | thumb flexion     15/30  |
|                           | ##########-----------    |
|                           | ...                      |
|                           | hand tracked             |
+---------------------------+--------------------------+
```

The panel is driven by the **commanded** angles -- what actually went out over
RS485 after clamping and rate limiting -- not by the raw tracker output. If the
hand cannot keep up with you, or a joint is pinned at its limit, the panel shows
that rather than hiding it. Bars turn red on an alarm.

`check` also opens a preview window with the live detection rate, so you can fix
your framing before calibrating.

| key | |
|---|---|
| `SPACE` | engage / pause following |
| `o` | open the hand and disengage |
| `c` | clear alarms |
| `q` or `Esc` | quit |

Useful flags:

```bash
python3 run.py teleop --dry-run        # track and display, never move the hand
python3 run.py teleop --no-home        # skip homing (only if already homed)
python3 run.py teleop --hand Left      # track your left hand instead
python3 run.py teleop --camera 0       # use the Innomaker instead
python3 run.py teleop --max-current 300  # gentler grip force (per-mille)
python3 run.py teleop --smoothing 0.2  # smoother but laggier
```

## Adding a pose

Every pose in `POSE_PLAN` ([poses.py](telehand/poses.py)) is one anchor: the
robot's joint angles for it, and your hand's tracker signals for it. Poses added
to the plan after `calibration/poses.json` was written appear automatically as unmeasured,
uncalibrated anchors. To bring one to life:

```bash
python3 run.py jog          # n/N to the pose, adjust joints, s to save its robot side
python3 run.py calibrate    # records ONLY poses still missing signals (--all redoes every one)
```

The current plan: open, the four thumb-to-finger pinches, and fist -- six
anchors, every robot side measured with `jog`. Poses whose robot joints are only
estimates are excluded from teleop by default (`--allow-estimates` to include).

## Retargeting modes

`teleop --mode` selects how your hand becomes joint angles. Both are kept so
they can be compared on the same hand.

| mode | what it does | strengths |
|---|---|---|
| `poses` (default) | pose library: your recorded poses blended by similarity ([poses.py](telehand/poses.py)) | recorded poses (pinches!) are reproduced exactly |
| `direct` | vendor-style: smooth the 21 landmarks (EMA + Kalman), 2D bend angle per joint, linear map to each actuator ([direct.py](telehand/direct.py)) | 1:1, responsive, no anchors needed |
| `hybrid` | `direct` for the four fingers, `poses` for the two thumb joints | responsive fingers + a thumb that still pinches |
| `fk` | kinematic: your fingertip layout is expressed in the robot's palm frame, scaled by palm width, and a damped Gauss-Newton solve finds the six actuator angles whose forward kinematics match it ([fkretarget.py](telehand/fkretarget.py)) | any pose has geometric meaning; no anchors to record |

`fk` rests on [kinematics.py](telehand/kinematics.py): vendor URDF geometry, the
MJCF coupling polynomials, a finger stroke-to-angle curve measured from photos
(0/20/40/60/80 -> 0/19.4/35.4/50.1/62.6 deg), a thumb curve fitted to tactile
contacts (`data/touch_grid.json`), and the measured abduction-dependent flexion
envelope. Matched quantities are thumb-to-finger, neighbouring-finger and
palm-to-finger vectors plus the thumb's pointing direction; there is
deliberately no palm-to-thumb term, because the two thumbs sit differently on
their palms. `--record` in any mode also writes the 21x3 world landmarks to a
`.npz` next to the CSV.

**Per-operator profile.** Hands differ in proportion, and a single palm-width
scale cannot absorb that. Before an operator's first session:

```bash
python3 run.py profile --name zhang            # ~10 s, no motors
python3 run.py profile --name li --out calibration/handprofile_li.json
python3 run.py teleop --mode fk --hand Right --profile calibration/handprofile_li.json
```

An open flat hand gives each finger's length (the solver then scales every
finger by its own ratio to the robot's), and a real thumb-index pinch gives
the tracker's residual gap for this hand (MediaPipe leaves ~2-3 cm of air),
which sets the pinch-closure intent threshold. `teleop` loads
`calibration/handprofile.json` automatically when it exists.

Three behaviours sit on top of the solve in `fk` mode:

- **Tactile pinch closure** (`--no-tactile` to disable): when your thumb is
  within ~35 mm of a fingertip, that finger keeps curling past the solution
  until the thumb-tip sensor feels contact or the finger stalls against the
  thumb; a stall with no pressure means it is jammed on the thumb's side, and
  it backs off. MediaPipe leaves a centimetre of air in a real pinch, and this
  closes it. Sensor ids, layout and API notes: [docs/tactile.md](docs/tactile.md).
- **Thumb extended**: a straight thumb clear of every fingertip (open hand,
  thumbs-up) is sent to `[0, 0]` outright -- the hardware's best rendering of
  both, measured with `jog` -- instead of the solver's compromise.
- **Fingers wait for the thumb**: the abduction joint is slow to let go. While
  it still sits more than 6 deg across from where it was sent, the fingers hold
  their curl rather than closing into its path.

Known hardware limit: thumb abduction sticks on the way back (60 -> 0 can stop
at 15-25 deg). Raising the current limit does not help -- the drive never asks
for more than ~250 mA -- so this is the firmware position loop, not torque. The
stall kick recovers most cases; the rest needs the vendor's drive parameters.

`direct` is a re-implementation of the pipeline inside Leadtron's own
`hand_detector` (recovered from the shipped Windows bundle): the same landmark
triplets, `bend = 180 - angle`, and the same smoothing constants. It knows
nothing about fingertip geometry, which is why it feels immediate and why it
cannot promise a pinch closes.

In `direct`/`hybrid`, press **`1`** with an open hand and **`2`** with a fist
(20 frames each) to set the per-joint ranges; they are saved to
`calibration/direct_calibration.json`. Tuning: `--lm-alpha` (EMA, vendor 0.3),
`--lm-deadband` (vendor 0.02), `--no-kalman`.

## Choosing a camera

```bash
python3 run.py cameras
```

Lists every capture device with a measured frame rate and a path that survives
replugging. `--camera` takes an index, a `/dev` node, a `by-path` link, or a
name fragment (the default is `icspring`):

```bash
python3 run.py teleop --camera icspring
python3 run.py teleop --camera /dev/v4l/by-path/pci-0000:13:00.4-usb-0:2:1.0-video-index0
```

Two things make bare indices unreliable. Device numbers shuffle when cameras are
plugged in a different order -- the icSpring has been `/dev/video2` and
`/dev/video0` on different days -- and two identical Innomakers report the same
serial, so `by-id` cannot separate them. `by-path` is keyed on the USB port and
stays unique; a name fragment is refused when it matches more than one camera.

Opening a camera also waits ~1.5 s for auto-exposure to settle. Without it the
first frames come back blown out or black, which reads as a dead device.

## Hand view

The panel draws the hand from the vendor model, driven by the same angles that
go out over RS485. Two backends, picked automatically:

| backend | how | cost |
|---|---|---|
| MuJoCo (preferred) | renders the vendor MJCF with real meshes and lighting; needs a GL context | 0.15 ms/frame |
| convex hulls | each link's STL reduced to a sampled point cloud, drawn as its projected hull; pure numpy + cv2 | 2.4 ms/frame |

```bash
python3 run.py handview --pose 10,15,45,0,0,0     # no camera, no motors
python3 run.py handview --poses calibration/poses.json   # n/N walks the anchors, hjkl orbits
python3 run.py teleop --mode fk --show-measured   # outline where the hand actually is
```

`--show-measured` renders the measured pose as well and keeps only its
silhouette, drawn over the commanded one. A joint that has not arrived -- thumb
abduction sticking on its way back, a finger blocked by the thumb -- shows up as
two outlines pulling apart, instead of having to be read off the numbers.

Each joint's value is printed beside the finger it drives (the thumb shows
`abduction/flexion`), with one compact row of vertical bars underneath for the
position within each joint's travel.

The view is driven with the mouse, inside the rendered rectangle only, so
dragging over the camera image never moves it:

| | |
|---|---|
| drag | orbit |
| wheel | zoom |
| double click | back to the default view |
| `v` | save the current angle to `.telehand/view.json` |
| `hjkl` / `r` | keyboard equivalents |

Both `teleop` and `jog` share the saved viewpoint, so an angle found while
measuring a pose in `jog` is the one teleop starts with.

The MJCF is the same hand as the URDF, rotated 180 deg about z: link-to-link
distances agree to 0.0 mm. Fingertips need care, though. MuJoCo recentres mesh
vertices on the centroid, so the vertex furthest from the mesh origin is
whichever end of the pad is further from its middle -- often the root. Tips are
therefore measured from the body origin, which brings the two models into exact
agreement; before that fix they disagreed by up to 40 mm and the tip markers sat
mid-phalanx.

**Self-contact.** The MJCF ships with collisions off; the renderer turns them
on. The meshes are rigid while the real fingertips are rubber, so wherever the
hand is genuinely pressing on itself the two links overlap. That overlap is
marked with a ring on the view -- larger and redder with depth -- and listed in
the side column as `thumb-middle 8mm`.

It is not a modelling error worth removing. Both pinch poses come out at exactly
0 mm; only `fist` and `point` overlap, and those are poses where the real thumb
does press on the fingers. A sweep of 90 thumb-parameter combinations could only
improve the worst case from -9.0 mm to -4.3 mm and never reached zero, while
making the pinch region worse -- so the overlap is the missing rubber, not a bad
calibration. Clamping it away would be worse still: backing the fist off until
nothing overlaps drives the middle finger from 80 deg to 0.

The framing solves itself per viewpoint: a fixed camera distance frames well
from one direction and badly from the next, so the hand is rendered onto an
oversized canvas, measured where it actually lands, then scaled by distance and
centred by cropping. Solved once per viewpoint and cached, so a steady view
costs 0.2 ms a frame.

## Stopping a session

The hand and camera are single-access devices, and a teleop loop that is still
engaged keeps driving motors whether or not anyone is watching. Only one
telehand session may hold them at a time.

```bash
python3 run.py stop      # ends the running session cleanly
```

Pressing `q` in the window does the same thing. Both take the normal shutdown
path: motors stopped, monitor thread joined, serial closed.

Starting a second session while one is running is refused, naming the one that
holds the hardware:

```
ERROR: another telehand session is already running: `teleop` (pid 47170, started 14:53:52).
  Stop it with:  python3 run.py stop
```

The running pid is recorded in `.telehand/session.lock`. It has to be the pid of the
Python process itself: signalling a wrapper shell does not reach the process
underneath, which leaves the hand running with nobody supervising it. A lock
left behind by a crash (`kill -9`) is detected as stale and cleared on the next
run.

## Unplugging the hand

Shut down in software first:

```bash
python3 run.py stop      # or press q in the window
```

That now **de-energizes the motors**: motion is halted, the drivers are disabled,
and current drops to zero. The hand goes limp, so anything it was gripping will
be released.

Pulling the USB cable is not a substitute. That cable carries RS485 data only --
the hand is powered separately -- so unplugging it leaves the motors enabled and
still holding force, with the firmware receiving no stop command. The SDK
documents no watchdog or link-loss behaviour, so what the firmware does after
that is undefined.

`stop_motors()` alone is not enough either: it ends the current move but leaves
the drivers energized. Measured after what used to be a clean shutdown, all six
motors still read `enable=True` drawing 18-117 units of current. Disabling is
what actually lets go, and `disconnect()` now does it.

Pass `--keep-enabled` to `teleop` or `jog` to leave the motors holding on exit.

## Safety

Layered, so a tracking glitch cannot become a slammed finger:

- Every command is **clamped** to the limits the firmware reports.
- Joint motion is **rate-limited** host-side (`--max-speed`, default 180°/s) on
  top of the firmware's own 150°/s velocity limit.
- **Current limited** to 500‰ by default, which caps grip force.
- **Stall kick**: the worm-drive joints stick -- asked for 15°, the thumb flexion
  parked at 9° with every pinch missing. If a joint sits short of a steady target
  without moving, the command is briefly overshot by 8° to break static friction,
  then restored. `--no-stall-kick` disables it.
- Teleop **starts paused**; you engage deliberately.
- **Losing tracking holds the last pose** rather than snapping anywhere.
- Quitting, `Ctrl+C`, `SIGTERM`, or any error stops motors and closes the bus
  cleanly. Only a forced `kill -9` skips that, and it leaves motors enabled.
- Only one session can hold the hand at a time; `run.py stop` ends it.
- Disconnecting de-energizes the motors, so exiting always leaves the hand limp
  rather than silently gripping. `--keep-enabled` opts out.

## Layout

```
run.py                   entry point: puts vendor/ on sys.path, runs the CLI
telehand/
  bootstrap.py           vendor/ and cmeel path setup, native log quieting
  paths.py               every file and directory the project uses
  cli.py                 argument parser and command dispatch
  commands/              one module per subcommand (check, calibrate, teleop, jog, ...)
  sdk.py                 loads the vendor LHandProLib .so
  hand.py                clamping, rate limiting, stall kick; the only path to RS485
  camera.py              camera discovery and newest-frame V4L2 capture
  tracker.py             MediaPipe landmarks -> 6 raw signals
  poses.py               pose library and blended retargeting   (--mode poses)
  direct.py              vendor-style per-joint linear map       (--mode direct)
  kinematics.py          DH116 forward kinematics, measured drive curves
  fkretarget.py          kinematic retargeting, tactile pinch closure (--mode fk)
  profile.py             per-operator hand measurements for fk mode
  touchscan.py           tactile contact sweeps
  handview.py            MuJoCo / convex-hull renderer of the hand
  viz.py                 camera overlay and the side panel
  session.py             one-instance lock and clean stop
calibration/             poses.json (pose library), handprofile.json, direct_calibration.json
data/                    measurements: touch_grid.json, abd_envelope.json, photos/, recordings/
docs/                    tactile.md (sensor notes), the DH116 user manual
models/                  hand_landmarker.task
scripts/                 tactile_probe.py, pip.pyz
third_party/             LHandProLib SDK, DH116 URDF and MJCF packages
vendor/                  project-local Python dependencies
.telehand/               runtime state: camera choice, view angle, mesh cache, session lock
somehand/                sibling retargeting projects adapted to this hand (below)
AnyDexRetarget/
```

Every path above is defined once, in [paths.py](telehand/paths.py). `data/`
has its own [README](data/README.md) describing each measurement file.

## Related projects

Two other retargeting stacks are checked out here so that all three can be
compared on the same hand. Each is a git clone carrying a small local
adaptation for the DH116; keep the upstream diff minimal (`git -C <dir> diff`).

| directory | what it is | where to start |
|---|---|---|
| `AnyDexRetarget/` | optimisation-based retargeting (Pinocchio + NLopt) | `python3 AnyDexRetarget/dh116.py env`, then [README_DH116.md](AnyDexRetarget/README_DH116.md) |
| `somehand/` | MediaPipe + MuJoCo retargeting with an RS485 backend for this hand | `somehand/configs/retargeting/{base,right}/dh116*.yaml` and its own README |

Both reach the hardware through the vendor SDK under `third_party/`, and
AnyDexRetarget drives the hand through `telehand.hand.Hand` itself, so the
safety layer is shared.

## Implementation notes

### Camera capture

Both cameras were benchmarked at 640x480 MJPG. The icSpring (`/dev/video2`) is
the default at 29.8 fps; the Innomaker (`/dev/video0`) manages 29.5.

Two non-obvious things about the capture path:

- `CAP_PROP_BUFFERSIZE = 1` is the usual advice for avoiding stale frames, but
  on the V4L2 backend it **halves** the delivered rate — measured 15 fps on both
  cameras, against 30 without it. It is not set.
- Leaving the queue at its default means a slow consumer falls behind and you
  end up teleoperating from frames a few hundred ms old. A reader thread that
  keeps only the newest frame solves that without costing frame rate.

Both cameras default to aperture-priority auto-exposure, which can also drop the
rate in dim light. If frame rate sags, add more light on your hand rather than
forcing a manual exposure — the tracker needs the contrast anyway.

### How a finger angle is derived

Signals come from MediaPipe's `hand_world_landmarks` (metric and hand-relative)
rather than image coordinates, so readings hold steady as you move toward or
away from the camera.

- **Finger flexion** — the total bend along the finger: the turn at the PIP
  joint plus the turn at the DIP joint. ~0° straight, ~170° fully curled.
- **Thumb abduction** — the angle at the wrist between the thumb MCP and the
  index MCP. Measured at the MCP, not the tip, so thumb *curl* (which happens
  further out) does not leak into the abduction reading.

Calibration stores the raw value at both the open and fist pose per signal and
interpolates between them, so the mapping self-corrects for direction — it
works even where a larger raw angle means a smaller joint angle.
