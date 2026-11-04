from pathlib import Path
import json
import subprocess
import sys

import mujoco
import numpy as np

from diana7_control_demo import actuator_home, compute_control_targets


ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "diana7_description" / "diana7_dual.xml"


def load_home():
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data, actuator_home(model, data)


def test_control_targets_are_finite_bounded_and_animate_both_hands():
    model, _, home = load_home()
    samples = [compute_control_targets(model, home, time_s) for time_s in (0.0, 0.5, 1.0, 10.0)]
    lo = model.actuator_ctrlrange[:, 0]
    hi = model.actuator_ctrlrange[:, 1]

    assert all(target.shape == (38,) for target in samples)
    assert all(np.all(np.isfinite(target)) for target in samples)
    assert all(np.all(target >= lo) and np.all(target <= hi) for target in samples)
    assert not np.allclose(samples[0][14:26], samples[1][14:26])
    assert not np.allclose(samples[0][26:38], samples[1][26:38])
    assert not np.allclose(samples[0][14:26], samples[0][26:38])


def test_arm_targets_keep_original_mirrored_sinusoid():
    model, _, home = load_home()
    targets = compute_control_targets(model, home, 0.75)

    left_delta = targets[:7] - home[:7]
    right_delta = targets[7:14] - home[7:14]
    np.testing.assert_allclose(left_delta, -right_delta, atol=1e-12)


def test_combined_arm_and_hand_control_stays_finite_for_1000_steps():
    model, data, home = load_home()
    for _ in range(1000):
        data.ctrl[:] = compute_control_targets(model, home, data.time)
        mujoco.mj_step(model, data)

    for values in (data.qpos, data.qvel, data.qacc, data.ctrl):
        assert np.all(np.isfinite(values))
    for name in (f"{side}_joint_{index}" for side in ("L", "R") for index in range(1, 8)):
        joint_id = model.joint(name).id
        assert np.isfinite(data.qpos[model.jnt_qposadr[joint_id]])


def test_headless_cli_runs_from_an_unrelated_working_directory(tmp_path):
    result = subprocess.run(
        [sys.executable, str(ROOT / "diana7_control_demo.py"), "--headless", "--steps", "20"],
        cwd=tmp_path,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    summary = json.loads(result.stdout)
    assert summary == {"finite": True, "ncam": 3, "nq": 38, "nu": 38, "steps": 20}
