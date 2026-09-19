#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DH116-L000-A1 灵巧手 MuJoCo 交互仿真
========================================
启动后通过窗口右侧 Actuators 面板拖动滑块控制各主动关节。
被动关节通过 equality 约束自动跟随主动关节。

运行:
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
    sys.exit("[ERROR] 请先安装: pip install mujoco")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, ".", "DH116-L000-A1.xml")

ACTIVE_DESC = {
    "act_finger11": "拇指侧摆  (MCP ab/adduction)",
    "act_finger12": "拇指弯曲  (MCP flexion)",
    "act_finger21": "食指弯曲  (MCP flexion)",
    "act_finger31": "中指弯曲  (MCP flexion)",
    "act_finger41": "无名指弯曲(MCP flexion)",
    "act_finger51": "小拇指弯曲(MCP flexion)",
}
PASSIVE_DESC = [
    ("finger13", "finger12", "拇指 IP   ← 拇指 MCP"),
    ("finger22", "finger21", "食指 PIP  ← 食指 MCP"),
    ("finger32", "finger31", "中指 PIP  ← 中指 MCP"),
    ("finger42", "finger41", "无名 PIP  ← 无名 MCP"),
    ("finger52", "finger51", "小指 PIP  ← 小指 MCP"),
]


def print_info(model: "mujoco.MjModel") -> None:
    print("=" * 58)
    print(f"  {os.path.splitext(os.path.basename(MODEL_PATH))[0]}  灵巧手  MuJoCo 交互仿真")
    print("=" * 58)
    print(f"  MuJoCo {mujoco.__version__}")
    print(f"  关节: {model.njnt}  执行器: {model.nu}  equality: {model.neq}")
    print()

    print("  ── 主动关节（右侧 Actuators 面板拖动滑块控制）──")
    for i in range(model.nu):
        aname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        jid   = model.actuator_trnid[i, 0]
        lo, hi = model.jnt_range[jid]
        desc  = ACTIVE_DESC.get(aname, "")
        print(f"    {aname:<16} [{np.degrees(lo):6.1f}° ~ {np.degrees(hi):6.1f}°]  {desc}")

    print()
    print("  ── 被动关节（equality 约束自动跟随）──")
    for _, _, desc in PASSIVE_DESC:
        print(f"    {desc}")

    print()
    print("  ── 操作提示 ──")
    print("    Actuators 面板滑块  → 控制各手指弯曲/侧摆角度")
    print("    鼠标左键拖动        → 旋转视角")
    print("    鼠标右键拖动        → 平移视角")
    print("    滚轮                → 缩放")
    print("    Space               → 暂停 / 继续仿真")
    print("    Backspace           → 重置到初始状态")
    print("    关闭窗口            → 退出程序")
    print()


def run(model: "mujoco.MjModel", data: "mujoco.MjData") -> None:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth   = 140
        viewer.cam.elevation = -25
        viewer.cam.distance  = 0.35
        viewer.cam.lookat[:] = [0.0, 0.0, 0.07]

        print("  [仿真运行中] 请在窗口右侧 Actuators 面板拖动滑块来控制手指")
        print("  关闭窗口退出。")

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


def main() -> None:
    if not os.path.exists(MODEL_PATH):
        sys.exit(f"[ERROR] 找不到模型文件: {MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)

    print_info(model)
    run(model, data)


if __name__ == "__main__":
    main()
