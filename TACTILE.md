# DH116 Tactile Module Output Specification

Tested environment: DH116 (LHandPro `LAC_DOF_6`, right hand), RS485 `/dev/ttyUSB0` @ 500,000 baud,
SDK `LHandProLib-API-Linux-20260727`, Python wrapper `RS485_python/lhandprolib_python_sdk`.
Data collected on 2026-09-11 in read-only mode (without motor enable or homing).

## 1. Sensor Overview

The SDK exposes 11 tactile sensors, each consisting of a small contact array. Sensor IDs and their project naming are defined in `SENSOR_IDS` within `dexdh116/touchscan.py`.

| ID | Enum (C++ / C) | Location | Contact Points |
|---|---|---|---|
| 1 | `LSS_FINGER_1_1` / `C_LSS_FINGER_1_1` | Thumb tip `thumb_tip` | 5 |
| 2 | `LSS_FINGER_1_2` | Thumb pad `thumb_pad` | 9 |
| 3 | `LSS_FINGER_2_1` | Index tip `index_tip` | 9 |
| 4 | `LSS_FINGER_2_2` | Index pad `index_pad` | 9 |
| 5 | `LSS_FINGER_3_1` | Middle tip `middle_tip` | 9 |
| 6 | `LSS_FINGER_3_2` | Middle pad `middle_pad` | 9 |
| 7 | `LSS_FINGER_4_1` | Ring tip `ring_tip` | 9 |
| 8 | `LSS_FINGER_4_2` | Ring pad `ring_pad` | 9 |
| 9 | `LSS_FINGER_5_1` | Pinky tip `pinky_tip` | 5 |
| 10 | `LSS_FINGER_5_2` | Pinky pad `pinky_pad` | 9 |
| 11 | `LSS_HAND_PALM` | Palm `palm` | 26 |

Total: 108 contact points.

## 2. Contact Point Layout (`get_finger_sensor_pos`)

Coordinates are normalized to 0–1 and constant, describing relative contact positions within each sensor array. Three array types:

5-point array (IDs 1, 9):

```
x = [0.50, 0.41, 0.41, 0.58, 0.58]
y = [0.18, 0.50, 0.62, 0.62, 0.50]
```

9-point array (IDs 2–8, 10):

```
x = [0.50, 0.38, 0.38, 0.56, 0.56, 0.36, 0.36, 0.57, 0.57]
y = [0.08, 0.30, 0.21, 0.21, 0.30, 0.82, 0.69, 0.69, 0.82]
```

26-point palm array (ID 11), 5 rows from y=0.90 down to y=0.10 with 4/4/6/6/6 points per row:

```
x = [0.74, 0.58, 0.41, 0.25,
     0.74, 0.58, 0.41, 0.25,
     0.90, 0.74, 0.58, 0.41, 0.25, 0.10,
     0.90, 0.74, 0.58, 0.41, 0.25, 0.10,
     0.90, 0.74, 0.58, 0.41, 0.25, 0.10]
y = [0.90 x4, 0.70 x4, 0.50 x6, 0.30 x6, 0.10 x6]
```

## 3. Activation Sequence

```python
sdk.set_sensor_enable(True)      # start background parsing of tactile frames
time.sleep(1.0)                  # let the first frames arrive
sdk.set_finger_pressure_reset()  # zero the baseline; pressure is relative to this moment
time.sleep(0.6)
```

`set_sensor_enable` controls whether the background thread parses tactile frames (disabled by default). Before reset with no load, individual contacts may drift around 0.01–0.02; after reset, all zero out.
Both `teleop.py` pinch closure and `touchscan.py` use this sequence; `touchscan` resets pressure again each time the hand re-opens to prevent slow drift from registering as contact.

## 4. API & Return Types

### Python Wrapper (`lhandprolib_wrapper.py`)

| Method | Return Type | Description |
|---|---|---|
| `get_finger_sensor_pos(id)` | `Tuple[List[float], List[float]]` | x-list, y-list, length = point count |
| `get_finger_pressure(id)` | `List[float]` | One pressure value per point, length = point count, 0–1 |
| `get_finger_normal_force(id)` | `float` | Normal force, 0–1 |
| `get_finger_normal_force_ex(id)` | `List[float]` | Fixed 2 elements in practice |
| `get_finger_tangential_force(id)` | `float` | Tangential force, 0–1 |
| `get_finger_tangential_force_ex(id)` | `List[float]` | Fixed 2 elements in practice |
| `get_finger_force_direction(id)` | `float` | Force direction, 0–360°; 65535 when invalid |
| `get_finger_force_direction_ex(id)` | `List[float]` | Fixed 2 elements in practice |
| `get_finger_proximity(id)` | `float` | Proximity, 0–1 |
| `get_finger_proximity_ex(id)` | `List[float]` | Fixed 2 elements in practice |
| `set_sensor_enable(bool)` / `set_finger_pressure_reset()` | `None` | Raises exception on error |

Elements are Python `float` converted from C 32-bit `float` (single precision; coordinates may appear with trailing decimals like `0.4099999964237213`).

### C Interface (`LHandProLib.h`)

Each function returns an `int` error code (0 on success; non-zero codes in `C_LER_*`: 1 invalid argument, 2 uninitialized, 4 data exception, 5–8 communication error, 11 not homed).
Data is returned via out-parameters; array variants are allocated internally by the library and caller is responsible for calling `free`:

```c
int lhandprolib_get_finger_sensor_pos(handle, int sensor_id, float** x, float** y, int* count);
int lhandprolib_get_finger_pressure(handle, int sensor_id, float** list, int* count);
int lhandprolib_get_finger_normal_force(handle, int sensor_id, float* value);
int lhandprolib_get_finger_normal_force_ex(handle, int sensor_id, float** list, int* count);
/* tangential_force, force_direction, proximity follow the same two shapes */
```

The C++ version `get_finger_pressure(int, float*, int* io_count)` expects caller-allocated buffers: pass `nullptr` first to query count, then pass buffer.
The Python wrapper encapsulates both call steps and memory deallocation.

## 5. Raw Output Frame Sample (Thumb Tip, Unloaded)

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

Direction channels of other sensors in the same frame: fingertip sensors (1, 5, 7, 9) return 65535, finger pads and palm return 0. The index tip returned a non-zero value even without contact (308 and 298 across two runs).

## 6. Value Semantics & Important Notes

- Pressure is a **relative difference** from baseline, not an absolute quantity; baseline is established by `set_finger_pressure_reset()`.
- The direction channel returns 65535 (uint16 sentinel) when no contact is detected, not 0. Handle this separately when checking for contact.
- The four `_ex` arrays have a fixed length of 2 regardless of contact point count; their exact semantics are not detailed in vendor documentation.
- No timestamp or frame index is provided. Each call returns the latest frame cached by the SDK background thread; return values alone do not indicate whether data has updated.
- Reads are non-blocking: tested at 396 consecutive reads in 2 seconds without issues. Sensor intrinsic refresh rate was not measured under zero load.

## 7. Project Usage

Only the thumb tip (ID 1) `max(get_finger_pressure(1))` is actively used:

- `dexdh116/teleop.py` in `fk` mode (`PinchCloser`): contact detected when thumb tip pressure >= `pressure_on` (default 0.03) and the finger has commanded additional closure.
- `dexdh116/touchscan.py`: threshold `threshold` (default 0.05), scans contact positions where each finger touches the thumb.

Historical `touch_grid.json` record (46 entries, 10 contacts):

| State | Thumb Tip p |
|---|---|
| In contact | 0.06 – 1.00 |
| No contact | <= 0.04 |

## 8. Unverified Features

- Actual values of normal force, tangential force, direction, and proximity under active contact.
- Effects of `set_sensor_data_format(1)` (7-sensor channel mode) and `set_sensor_order`; the project currently uses default format 0 throughout.

## 9. Reproduction Script

`tactile_probe.py` in project root: read-only connection (no motor enable, no homing) that reads each of the 11 sensors once. Press a fingertip while running to see contact readings:

```bash
cd ~/DexDH116 && python3 tactile_probe.py
```

Core logic:

```python
hand = Hand(dry_run=True)
hand.connect(home=False)
sdk = hand._sdk
sdk.set_sensor_enable(True); time.sleep(1.0)
sdk.set_finger_pressure_reset(); time.sleep(0.6)
for sid, name in SENSORS.items():
    p = sdk.get_finger_pressure(sid)
    nf = sdk.get_finger_normal_force(sid)
    d = sdk.get_finger_force_direction(sid)
    print(f"{sid:2d} {name:10s} max_p={max(p):.2f} nf={nf:.2f} dir={d:.0f} p={[round(x, 2) for x in p]}")
hand.disconnect()
```

Output under unloaded conditions:

```
 1 thumb_tip  max_p=0.00 nf=0.00 dir=65535 p=[0.0, 0.0, 0.0, 0.0, 0.0]
 2 thumb_pad  max_p=0.00 nf=0.00 dir=0 p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 3 index_tip  max_p=0.00 nf=0.00 dir=298 p=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 ...
11 palm       max_p=0.00 nf=0.00 dir=0 p=[0.0, ... x26]
```
