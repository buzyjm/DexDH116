# DH116 tactile sensor output

Measured on the DH116 (LHandPro `LAC_DOF_6`, right hand) over RS485 at
`/dev/ttyUSB0`, 500000 baud, with SDK `LHandProLib-API-Linux-20260727` and its
Python wrapper `RS485_python/lhandprolib_python_sdk`. Data was read on
2026-09-11 on this machine in read-only mode (motors not enabled, no homing).

## 1. Sensor overview

The SDK exposes 11 tactile sensors, each a small array of taxels. The ids and
the names used in this project are in `SENSOR_IDS` in
[touchscan.py](../telehand/touchscan.py).

| id | enum (C++ / C) | location | taxels |
|---|---|---|---|
| 1 | `LSS_FINGER_1_1` / `C_LSS_FINGER_1_1` | thumb tip `thumb_tip` | 5 |
| 2 | `LSS_FINGER_1_2` | thumb pad `thumb_pad` | 9 |
| 3 | `LSS_FINGER_2_1` | index tip `index_tip` | 9 |
| 4 | `LSS_FINGER_2_2` | index pad `index_pad` | 9 |
| 5 | `LSS_FINGER_3_1` | middle tip `middle_tip` | 9 |
| 6 | `LSS_FINGER_3_2` | middle pad `middle_pad` | 9 |
| 7 | `LSS_FINGER_4_1` | ring tip `ring_tip` | 9 |
| 8 | `LSS_FINGER_4_2` | ring pad `ring_pad` | 9 |
| 9 | `LSS_FINGER_5_1` | pinky tip `pinky_tip` | 5 |
| 10 | `LSS_FINGER_5_2` | pinky pad `pinky_pad` | 9 |
| 11 | `LSS_HAND_PALM` | palm `palm` | 26 |

108 taxels in total.

## 2. Taxel layout (`get_finger_sensor_pos`)

Coordinates are normalised to 0-1 and are constant; they only describe the
relative position of the taxels within an array. There are three array shapes.

5 taxels (ids 1 and 9):

```
x = [0.50, 0.41, 0.41, 0.58, 0.58]
y = [0.18, 0.50, 0.62, 0.62, 0.50]
```

9 taxels (ids 2-8 and 10):

```
x = [0.50, 0.38, 0.38, 0.56, 0.56, 0.36, 0.36, 0.57, 0.57]
y = [0.08, 0.30, 0.21, 0.21, 0.30, 0.82, 0.69, 0.69, 0.82]
```

26-taxel palm (id 11): five rows from y=0.90 down to y=0.10, with 4/4/6/6/6
taxels per row:

```
x = [0.74, 0.58, 0.41, 0.25,
     0.74, 0.58, 0.41, 0.25,
     0.90, 0.74, 0.58, 0.41, 0.25, 0.10,
     0.90, 0.74, 0.58, 0.41, 0.25, 0.10,
     0.90, 0.74, 0.58, 0.41, 0.25, 0.10]
y = [0.90 x4, 0.70 x4, 0.50 x6, 0.30 x6, 0.10 x6]
```

## 3. Enable sequence

```python
sdk.set_sensor_enable(True)      # start background parsing of tactile frames
time.sleep(1.0)                  # let the first frames arrive
sdk.set_finger_pressure_reset()  # zero the baseline; pressure is relative to this moment
time.sleep(0.6)
```

`set_sensor_enable` only controls whether the background thread parses tactile
frames; it is off by default. Before the reset a few taxels drift by 0.01-0.02
with nothing touching them; after the reset every one reads zero. The pinch
closure in fk-mode teleop and `touchscan` both use this sequence; `touchscan`
also resets again after each time the hand opens, so slow drift cannot be
mistaken for contact.

## 4. Interface and return types

### Python wrapper (`lhandprolib_wrapper.py`)

| method | returns | notes |
|---|---|---|
| `get_finger_sensor_pos(id)` | `Tuple[List[float], List[float]]` | x list and y list, length = taxel count |
| `get_finger_pressure(id)` | `List[float]` | one pressure per taxel, length = taxel count, 0-1 |
| `get_finger_normal_force(id)` | `float` | normal force, 0-1 |
| `get_finger_normal_force_ex(id)` | `List[float]` | always 2 elements in practice |
| `get_finger_tangential_force(id)` | `float` | tangential force, 0-1 |
| `get_finger_tangential_force_ex(id)` | `List[float]` | always 2 elements in practice |
| `get_finger_force_direction(id)` | `float` | force direction, 0-360 degrees; 65535 when invalid |
| `get_finger_force_direction_ex(id)` | `List[float]` | always 2 elements in practice |
| `get_finger_proximity(id)` | `float` | proximity, 0-1 |
| `get_finger_proximity_ex(id)` | `List[float]` | always 2 elements in practice |
| `set_sensor_enable(bool)` / `set_finger_pressure_reset()` | `None` | raise on error |

Elements are Python `float` converted from C 32-bit `float`, so the precision
is single (coordinates show tails such as `0.4099999964237213`).

### C interface (`LHandProLib.h`)

Every function returns an `int` error code: 0 on success, otherwise one of
`C_LER_*` (1 bad argument, 2 not initialised, 4 bad data, 5-8 communication
errors, 11 not homed). Data comes back through out-parameters; the array
variants allocate inside the library and the caller must `free`:

```c
int lhandprolib_get_finger_sensor_pos(handle, int sensor_id, float** x, float** y, int* count);
int lhandprolib_get_finger_pressure(handle, int sensor_id, float** list, int* count);
int lhandprolib_get_finger_normal_force(handle, int sensor_id, float* value);
int lhandprolib_get_finger_normal_force_ex(handle, int sensor_id, float** list, int* count);
/* tangential_force, force_direction, proximity follow the same two shapes */
```

The C++ `get_finger_pressure(int, float*, int* io_count)` takes a caller-owned
buffer: pass `nullptr` first to get the count, then the array. The Python
wrapper hides the two-step call and the free.

## 5. One raw frame (thumb tip, nothing touching)

```
get_finger_sensor_pos(1)
  -> tuple: ([0.5, 0.4099999964237213, 0.4099999964237213, 0.5799999833106995, 0.5799999833106995],
             [0.18000000715255737, 0.5, 0.6200000047683716, 0.6200000047683716, 0.5])

get_finger_pressure(1)            -> list: [0.0, 0.0, 0.0, 0.0, 0.0]
get_finger_normal_force(1)        -> float: 0.0
get_finger_normal_force_ex(1)     -> list: [0.0, 0.0]
get_finger_tangential_force(1)    -> float: 0.0
get_finger_tangential_force_ex(1) -> list: [0.0, 0.0]
get_finger_force_direction(1)     -> float: 65535.0
get_finger_force_direction_ex(1)  -> list: [65535.0, 65535.0]
get_finger_proximity(1)           -> float: 0.0
get_finger_proximity_ex(1)        -> list: [0.0, 0.0]

get_finger_pressure(11)           -> list len=26: [0.0, 0.0, ..., 0.0]
```

In the same frame the direction channel of the other sensors: tip arrays (1, 5,
7, 9) give 65535, pads and palm give 0, and the index tip gives a value that
changes even with no contact (308 and 298 on two runs).

## 6. What the numbers mean

- Pressure is a **difference from the baseline**, not an absolute value; the
  baseline is whatever `set_finger_pressure_reset()` saw.
- The direction channel returns 65535 (a uint16 sentinel) with no contact,
  not 0, so contact detection has to treat it separately.
- The four `_ex` arrays are always length 2 regardless of taxel count; their
  meaning is not documented in the manual.
- There is no timestamp and no frame counter. Each call returns the latest
  frame cached by the SDK's background thread, so the return value cannot tell
  you whether the data has been updated.
- Reads are non-blocking: 396 consecutive reads in 2 seconds caused no
  problems. The sensor's own refresh rate was not measured (no contact in this
  session).

## 7. How the project uses it

Only `max(get_finger_pressure(1))`, the thumb tip (id 1), is used:

- `PinchCloser` in [fkretarget.py](../telehand/fkretarget.py), fk-mode teleop:
  contact is declared when thumb-tip pressure is at or above `pressure_on`
  (default 0.03) and the finger is carrying extra closure.
- [touchscan.py](../telehand/touchscan.py): threshold `threshold` (default
  0.05); sweeps each finger onto the thumb and records where it touches.

From the historical record `data/touch_grid.json` (46 entries, 10 contacts):

| state | thumb-tip pressure |
|---|---|
| contact | 0.06 - 1.00 |
| no contact | <= 0.04 |

## 8. Not yet verified

- Actual values of normal force, tangential force, direction and proximity
  during contact.
- The effect of `set_sensor_data_format(1)` (7-sensor channel mode) and
  `set_sensor_order`; the project has always used the default format 0.

## 9. Reproducing

[scripts/tactile_probe.py](../scripts/tactile_probe.py) connects read-only (no
enable, no homing) and reads each of the 11 sensors once. Press a fingertip
while it runs to see contact values.

```bash
cd ~/telehand && python3 scripts/tactile_probe.py
```

The core is just:

```python
hand = Hand(dry_run=True)
hand.connect(home=False)
sdk = hand._sdk
sdk.set_sensor_enable(True); time.sleep(1.0)
sdk.set_finger_pressure_reset(); time.sleep(0.6)
for name, sid in SENSOR_IDS.items():
    p = sdk.get_finger_pressure(sid)
    nf = sdk.get_finger_normal_force(sid)
    d = sdk.get_finger_force_direction(sid)
    print(f"{sid:2d} {name:10s} max_p={max(p):.2f} nf={nf:.2f} dir={d:.0f} p={[round(x, 2) for x in p]}")
hand.disconnect()
```

Output with nothing touching:

```
 1 thumb_tip  max_p=0.00 nf=0.00 dir=65535 p=[0.0, 0.0, 0.0, 0.0, 0.0]
 2 thumb_pad  max_p=0.00 nf=0.00 dir=0 p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 3 index_tip  max_p=0.00 nf=0.00 dir=298 p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 ...
11 palm       max_p=0.00 nf=0.00 dir=0 p=[0.0, ... x26]
```
