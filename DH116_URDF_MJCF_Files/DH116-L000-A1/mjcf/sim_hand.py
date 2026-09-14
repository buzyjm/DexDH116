#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DH116-L000-A1 Dexterous Hand MuJoCo Interactive Simulation
===========================================================
Drag sliders in the right-side Actuators panel to control active joints.
Passive joints automatically follow active joints via equality constraints.

Usage:
    python sim_hand.py
"""

import os, sys, time
import numpy as np

if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    sys.exit("[ERROR] Please install mujoco: pip install mujoco")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, ".", "DH116-L000-A1.xml")

ACTIVE_DESC = {
    "act_finger11": "Thumb ab/adduction (MCP ab/adduction)",
    "act_finger12": "Thumb flexion     (MCP flexion)",
    "act_finger21": "Index flexion     (MCP flexion)",
    "act_finger31": "Middle flexion    (MCP flexion)",
    "act_finger41": "Ring flexion      (MCP flexion)",
    "act_finger51": "Pinky flexion     (MCP flexion)",
}
PASSIVE_DESC = [
    ("finger13", "finger12", "Thumb IP   <- Thumb MCP"),
    ("finger22", "finger21", "Index PIP  <- Index MCP"),
    ("finger32", "finger31", "Middle PIP <- Middle MCP"),
    ("finger42", "finger41", "Ring PIP   <- Ring MCP"),
    ("finger52", "finger51", "Pinky PIP  <- Pinky MCP"),
]


def print_info(model: "mujoco.MjModel") -> None:
    print("=" * 58)
    print(f"  {os.path.splitext(os.path.basename(MODEL_PATH))[0]} Hand MuJoCo Interactive Simulation")
    print("=" * 58)
    print(f"  MuJoCo {mujoco.__version__}")
    print(f"  Joints: {model.njnt}  Actuators: {model.nu}  Equality: {model.neq}")
    print()

    print("  -- Active Joints (drag sliders in right Actuators panel) --")
    for i in range(model.nu):
        aname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        jid   = model.actuator_trnid[i, 0]
        lo, hi = model.jnt_range[jid]
        desc  = ACTIVE_DESC.get(aname, "")
        print(f"    {aname:<16} [{np.degrees(lo):6.1f}° ~ {np.degrees(hi):6.1f}°]  {desc}")

    print()
    print("  -- Passive Joints (equality constraints) --")
    for _, _, desc in PASSIVE_DESC:
        print(f"    {desc}")

    print()
    print("  -- Controls --")
    print("    Actuator sliders    -> Control finger flexion/abduction angles")
    print("    Left mouse drag     -> Orbit viewpoint")
    print("    Right mouse drag    -> Pan viewpoint")
    print("    Scroll wheel        -> Zoom")
    print("    Space               -> Pause / Resume simulation")
    print("    Backspace           -> Reset to initial state")
    print("    Close window        -> Exit program")
    print()


def run(model: "mujoco.MjModel", data: "mujoco.MjData") -> None:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth   = 140
        viewer.cam.elevation = -25
        viewer.cam.distance  = 0.35
        viewer.cam.lookat[:] = [0.0, 0.0, 0.07]

        print("  [Simulation Running] Drag sliders in right Actuators panel to control fingers.")
        print("  Close window to exit.")

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


def main() -> None:
    if not os.path.exists(MODEL_PATH):
        sys.exit(f"[ERROR] Model file not found: {MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)

    print_info(model)
    run(model, data)


if __name__ == "__main__":
    main()
