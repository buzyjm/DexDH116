# Measurement data

Everything here was measured on the DH116 in this lab; nothing is required at
runtime. The calibration the CLI reads back lives in `../calibration/`.

| file | what it is | used by |
|---|---|---|
| `touch_grid.json` | 46 thumb-flexion sweeps at fixed abduction / finger positions, with thumb-tip pressure and a `hit` flag (10 confirmed contacts) | fitting the thumb drive curve in `telehand/kinematics.py` (`THUMB_FLEX_FULL_DEG`, `THUMB_ABD_GAIN`) |
| `touch_events.json` | single logged contact event from the same session | reference |
| `abd_envelope.json` | `[commanded abduction, measured flexion limit, measured abduction]` rows; the abduction-dependent flexion envelope | `_FLEX_ENVELOPE` in `telehand/kinematics.py` |
| `poses_backup_2026-09-07.json` | the pose library before `pinch_pinky` was measured with `jog` | history; the live copy is `../calibration/poses.json` |
| `photos/` | side-view photos of the index finger and thumb at known firmware angles, taken with `run.py photostep` | the finger stroke-to-angle curve (0/20/40/60/80 -> 0/19.4/35.4/50.1/62.6 deg) |
| `recordings/fk_session.csv` | `teleop --record` log: per-frame signals, nearest anchor, commanded angles (header only in this copy) | |
| `recordings/fk_session.npz` | the 21x3 world landmarks for the same session, 727 frames, left hand | replay in AnyDexRetarget (`dh116.py convert`) |
| `recordings/fk_session_solved.npz` | the fk-mode solve for that recording | comparison between retargeting modes |
| `touchscan.json` | written by `run.py touchscan` (not present until a scan has been run) | `run.py touchscan` resumes from it |
