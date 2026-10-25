#!/usr/bin/env python
"""Diana7 双臂位置控制演示:MuJoCo 实时窗口,双臂绕 home 做平滑正弦动作。
关窗退出。ctrl = home + A·sin(ωt+φ),全部限幅在关节行程内。"""
from pathlib import Path

import numpy as np
import mujoco
import mujoco.viewer

XML = Path(__file__).resolve().parent / "diana7_description" / "diana7_dual.xml"
AMP = np.array([0.15, 0.25, 0.20, 0.40, 0.10, 0.25, 0.15])   # 每关节幅度 rad
OMEGA = np.array([0.6, 0.8, 0.5, 1.0, 0.7, 1.2, 0.9])       # rad/s
PHI = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0])


def main():
    m = mujoco.MjModel.from_xml_path(str(XML))
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    home = d.qpos.copy()
    lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
    print(f"模型: {m.nq} qpos / {m.nu} 执行器;home 姿态启动,双臂演示动作")

    with mujoco.viewer.launch_passive(m, d) as v:
        while v.is_running():
            t = d.time
            for arm in range(2):
                off = arm * 7
                sign = 1.0 if arm == 0 else -1.0   # 右臂镜像
                for j in range(7):
                    k = off + j
                    d.ctrl[k] = np.clip(
                        home[k] + sign * AMP[j] * np.sin(OMEGA[j] * t + PHI[j]),
                        lo[k], hi[k])
            mujoco.mj_step(m, d)
            v.sync()
    print("演示结束")


if __name__ == "__main__":
    main()
