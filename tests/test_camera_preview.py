from pathlib import Path
import json
import os
import subprocess
import sys
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

import camera_preview
from diana7_control_demo import actuator_home, compute_control_targets


ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "diana7_description" / "diana7_dual.xml"


def load_scene():
    model = mujoco.MjModel.from_xml_path(str(XML))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=256, width=256)
    return model, data, renderer


def test_render_camera_grid_has_three_distinct_nonblank_rgb_panels():
    _, data, renderer = load_scene()
    try:
        grid = camera_preview.render_camera_grid(renderer, data)
    finally:
        renderer.close()

    assert grid.shape == (256, 768, 3)
    assert grid.dtype == np.uint8
    panels = np.split(grid, 3, axis=1)
    assert all(float(panel.var()) > 100.0 for panel in panels)
    assert all(not np.array_equal(panels[index], panels[index + 1]) for index in range(2))


def test_fixed_camera_keeps_both_hand_assemblies_inside_the_frame():
    model, data, renderer = load_scene()
    home = actuator_home(model, data)
    hand_geom_ids = {}
    for side in ("L", "R"):
        geom_ids = {model.geom(f"{side}_palm_visual").id}
        for geom_id, body_id in enumerate(model.geom_bodyid):
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(body_id)) or ""
            if body_name.startswith(f"{side}_hand"):
                geom_ids.add(geom_id)
        hand_geom_ids[side] = tuple(geom_ids)

    try:
        renderer.enable_segmentation_rendering()
        for step in range(101):
            if step % 10 == 0:
                renderer.update_scene(data, camera="fixed_camera")
                object_ids = renderer.render()[..., 0]
                for side in ("L", "R"):
                    mask = np.isin(object_ids, hand_geom_ids[side])
                    rows, columns = np.where(mask)
                    assert int(mask.sum()) > 20
                    assert columns.min() > 5 and columns.max() < 250
                    assert rows.min() > 5 and rows.max() < 250
            data.ctrl[:] = compute_control_targets(model, home, data.time)
            mujoco.mj_step(model, data)
    finally:
        renderer.close()


def test_finger_motion_changes_both_wrist_camera_images():
    model, data, renderer = load_scene()
    try:
        before = np.split(camera_preview.render_camera_grid(renderer, data), 3, axis=1)
        arm_qpos_before = {
            name: float(data.qpos[model.jnt_qposadr[model.joint(name).id]])
            for name in (f"{side}_joint_{index}" for side in ("L", "R") for index in range(1, 8))
        }
        for side in ("L", "R"):
            for suffix in ("hand1_2", "hand1_3", "hand1_4", "hand2_1", "hand2_2", "hand3_1", "hand3_2", "hand4_1", "hand4_2", "hand5_1", "hand5_2"):
                joint_id = model.joint(f"{side}_{suffix}").id
                data.qpos[model.jnt_qposadr[joint_id]] = 1.0
        mujoco.mj_forward(model, data)
        after = np.split(camera_preview.render_camera_grid(renderer, data), 3, axis=1)
        arm_qpos_after = {
            name: float(data.qpos[model.jnt_qposadr[model.joint(name).id]])
            for name in arm_qpos_before
        }
    finally:
        renderer.close()

    assert arm_qpos_after == arm_qpos_before
    assert np.mean(np.abs(after[1].astype(float) - before[1].astype(float))) > 0.5
    assert np.mean(np.abs(after[2].astype(float) - before[2].astype(float))) > 0.5


def test_missing_opencv_has_preview_only_install_message(monkeypatch):
    def missing_cv2(name):
        raise ImportError(name)

    monkeypatch.setattr(camera_preview.importlib, "import_module", missing_cv2)
    with pytest.raises(RuntimeError, match="pip install opencv-python"):
        camera_preview.load_cv2()


def test_headless_cli_renders_from_an_unrelated_working_directory(tmp_path):
    env = os.environ.copy()
    env["MUJOCO_GL"] = "egl"
    result = subprocess.run(
        [sys.executable, str(ROOT / "camera_preview.py"), "--headless", "--steps", "5"],
        cwd=tmp_path,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    summary = json.loads(result.stdout)
    assert summary["steps"] == 5
    assert summary["finite"] is True
    assert summary["image_shape"] == [256, 768, 3]
    assert all(value > 100.0 for value in summary["panel_variances"])


def test_live_window_close_releases_renderer_and_opencv_once(monkeypatch, tmp_path):
    class FakeRenderer:
        def __init__(self):
            self.close_calls = 0

        def close(self):
            self.close_calls += 1

    class FakeCv2:
        COLOR_RGB2BGR = 0
        FONT_HERSHEY_SIMPLEX = 0
        LINE_AA = 0
        WND_PROP_VISIBLE = 0

        def __init__(self):
            self.destroy_calls = 0

        @staticmethod
        def cvtColor(image, _code):
            return image

        @staticmethod
        def putText(*_args):
            return None

        @staticmethod
        def imshow(*_args):
            return None

        @staticmethod
        def waitKey(_delay):
            return 0

        @staticmethod
        def getWindowProperty(*_args):
            return 0.0

        def destroyAllWindows(self):
            self.destroy_calls += 1

    renderer = FakeRenderer()
    cv2 = FakeCv2()
    model = object()
    data = SimpleNamespace(
        ctrl=np.zeros(1),
        qpos=np.zeros(1),
        qvel=np.zeros(1),
        time=0.0,
    )
    fake_mujoco = SimpleNamespace(
        MjModel=SimpleNamespace(from_xml_path=lambda _path: model),
        MjData=lambda _model: data,
        Renderer=lambda *_args, **_kwargs: renderer,
        mj_resetDataKeyframe=lambda *_args: None,
        mj_forward=lambda *_args: None,
        mj_step=lambda *_args: None,
    )
    grid = (np.indices((256, 768, 3)).sum(axis=0) % 256).astype(np.uint8)

    monkeypatch.setattr(
        camera_preview,
        "parse_args",
        lambda: SimpleNamespace(steps=None, headless=False, xml=tmp_path / "scene.xml"),
    )
    monkeypatch.setattr(camera_preview, "mujoco", fake_mujoco)
    monkeypatch.setattr(camera_preview, "actuator_home", lambda *_args: np.zeros(1))
    monkeypatch.setattr(camera_preview, "compute_control_targets", lambda *_args: np.zeros(1))
    monkeypatch.setattr(camera_preview, "render_camera_grid", lambda *_args: grid)
    monkeypatch.setattr(camera_preview, "load_cv2", lambda: cv2)

    assert camera_preview.main() == 0
    assert renderer.close_calls == 1
    assert cv2.destroy_calls == 1
