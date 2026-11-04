#!/usr/bin/env python3
"""Diana7 双臂与左右五指手的平滑位置控制演示。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np


XML = Path(__file__).resolve().parent / "diana7_description" / "diana7_dual.xml"
ARM_AMP = np.array([0.15, 0.25, 0.20, 0.40, 0.10, 0.25, 0.15])
ARM_OMEGA = np.array([0.6, 0.8, 0.5, 1.0, 0.7, 1.2, 0.9])
ARM_PHI = np.array([0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
HAND_PHASE = np.array([0.0, 0.15, 0.30, 0.45, 0.10, 0.28, 0.18, 0.36, 0.26, 0.44, 0.34, 0.52])


def actuator_home(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Return joint positions in actuator order, independent of qpos tree order."""
    targets = np.empty(model.nu, dtype=np.float64)
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        targets[actuator_id] = data.qpos[model.jnt_qposadr[joint_id]]
    return targets


def compute_control_targets(
    model: mujoco.MjModel,
    home: np.ndarray,
    time_s: float,
) -> np.ndarray:
    """Compute bounded arm and finger targets in actuator order."""
    if home.shape != (model.nu,):
        raise ValueError(f"home must have shape ({model.nu},), got {home.shape}")
    targets = home.astype(np.float64, copy=True)
    arm_wave = ARM_AMP * np.sin(ARM_OMEGA * time_s + ARM_PHI)
    targets[:7] = home[:7] + arm_wave
    targets[7:14] = home[7:14] - arm_wave

    for offset, phase_offset in ((14, 0.0), (26, np.pi / 3.0)):
        wave = 0.5 * (1.0 + np.sin(0.9 * time_s + HAND_PHASE + phase_offset))
        lo = model.actuator_ctrlrange[offset : offset + 12, 0]
        hi = model.actuator_ctrlrange[offset : offset + 12, 1]
        targets[offset : offset + 12] = lo + 0.65 * (hi - lo) * wave

    return np.clip(
        targets,
        model.actuator_ctrlrange[:, 0],
        model.actuator_ctrlrange[:, 1],
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=None, help="maximum control steps")
    parser.add_argument("--headless", action="store_true", help="run without a viewer")
    args = parser.parse_args()
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be at least 1")
    return args


def main() -> int:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    home = actuator_home(model, data)
    data.ctrl[:] = home
    completed = 0

    if args.headless:
        limit = args.steps if args.steps is not None else 1000
        for _ in range(limit):
            data.ctrl[:] = compute_control_targets(model, home, data.time)
            mujoco.mj_step(model, data)
            completed += 1
    else:
        from mujoco import viewer as mujoco_viewer

        with mujoco_viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and (args.steps is None or completed < args.steps):
                data.ctrl[:] = compute_control_targets(model, home, data.time)
                mujoco.mj_step(model, data)
                viewer.sync()
                completed += 1

    finite = bool(
        all(np.all(np.isfinite(values)) for values in (data.qpos, data.qvel, data.qacc, data.ctrl))
    )
    print(
        json.dumps(
            {"steps": completed, "finite": finite, "nq": model.nq, "nu": model.nu, "ncam": model.ncam},
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0 if finite else 1


if __name__ == "__main__":
    raise SystemExit(main())
