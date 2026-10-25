#!/usr/bin/env python
"""EVT-VLA3 仿真场景交互查看器。默认 diana7_dual,可传其他场景 xml 路径。"""
import sys
from pathlib import Path

import mujoco
import mujoco.viewer

DEFAULT_SCENE = Path(__file__).resolve().parent / "diana7_description" / "diana7_dual.xml"

xml = sys.argv[1] if len(sys.argv) > 1 else str(DEFAULT_SCENE)
m = mujoco.MjModel.from_xml_path(xml)
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
print(f"场景: {xml}")
print(f"加载 OK: {m.nq} 个自由度 / {m.nu} 个执行器;关闭窗口即退出")
mujoco.viewer.launch(m, d)
print("查看器已关闭")
