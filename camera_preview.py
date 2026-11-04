#!/usr/bin/env python3
"""Preview fixed, left-wrist, and right-wrist MuJoCo cameras side by side."""

from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path

import mujoco
import numpy as np

from diana7_control_demo import XML, actuator_home, compute_control_targets


CAMERA_NAMES = ("fixed_camera", "left_wrist_camera", "right_wrist_camera")
PANEL_SIZE = 256


def render_camera_grid(
    renderer: mujoco.Renderer,
    data: mujoco.MjData,
    camera_names: tuple[str, ...] = CAMERA_NAMES,
) -> np.ndarray:
    """Render named cameras and return one horizontal RGB uint8 grid."""
    frames = []
    for camera_name in camera_names:
        renderer.update_scene(data, camera=camera_name)
        frames.append(renderer.render().copy())
    return np.concatenate(frames, axis=1)


def panel_variances(grid: np.ndarray, panel_width: int = PANEL_SIZE) -> list[float]:
    return [
        float(grid[:, start : start + panel_width].var())
        for start in range(0, grid.shape[1], panel_width)
    ]


def load_cv2():
    try:
        return importlib.import_module("cv2")
    except ImportError as exc:
        raise RuntimeError(
            "Live camera preview requires OpenCV; install it with: pip install opencv-python"
        ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=None, help="maximum control steps")
    parser.add_argument("--headless", action="store_true", help="render without a window")
    parser.add_argument("--xml", type=Path, default=XML, help="optional scene XML")
    args = parser.parse_args()
    if args.steps is not None and args.steps < 1:
        parser.error("--steps must be at least 1")
    return args


def main() -> int:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(str(args.xml.expanduser().resolve()))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    home = actuator_home(model, data)
    renderer = mujoco.Renderer(model, height=PANEL_SIZE, width=PANEL_SIZE)
    cv2 = None
    completed = 0
    grid = None
    limit = args.steps if args.steps is not None else (100 if args.headless else None)

    try:
        if not args.headless:
            cv2 = load_cv2()
        while limit is None or completed < limit:
            data.ctrl[:] = compute_control_targets(model, home, data.time)
            mujoco.mj_step(model, data)
            grid = render_camera_grid(renderer, data)
            completed += 1
            if cv2 is None:
                continue

            display = cv2.cvtColor(grid, cv2.COLOR_RGB2BGR)
            for index, name in enumerate(CAMERA_NAMES):
                cv2.putText(
                    display,
                    name,
                    (index * PANEL_SIZE + 8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            cv2.imshow("Diana7 cameras - q/ESC to quit", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if cv2.getWindowProperty("Diana7 cameras - q/ESC to quit", cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        renderer.close()
        if cv2 is not None:
            cv2.destroyAllWindows()

    if grid is None:
        raise RuntimeError("camera preview completed without rendering a frame")
    finite = bool(all(np.all(np.isfinite(values)) for values in (data.qpos, data.qvel, data.ctrl)))
    summary = {
        "steps": completed,
        "finite": finite,
        "image_shape": list(grid.shape),
        "panel_variances": [round(value, 3) for value in panel_variances(grid)],
    }
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0 if finite and all(value > 0 for value in summary["panel_variances"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
