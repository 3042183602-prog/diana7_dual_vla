# Diana7 Dexterous Hands and Three Cameras Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the standalone Diana7 dual-arm MuJoCo package with two animated five-finger hands, one fixed camera, two inward wrist cameras, and a three-view preview.

**Architecture:** Keep `diana7_dual.xml` as the single runtime scene so the package remains copy-and-run. Copy the selected Robotis KS hand meshes into a local `hands/` asset directory, attach prefixed left/right articulated hand trees below `L_link_7` and `R_link_7`, and append hand actuators after the unchanged 14 arm actuators. Put reusable control-target generation in the existing demo module and keep multi-camera rendering in a separate preview script.

**Tech Stack:** Python 3.9+, MuJoCo 3.x, NumPy, optional OpenCV for the live tiled preview, pytest.

**Spec:** `docs/superpowers/specs/2026-11-04-diana7-dexterous-hands-three-cameras-design.md`

## Global Constraints

- Preserve the existing first 14 qpos values and first 14 actuator entries as left arm `j1..j7`, then right arm `j1..j7`.
- Use the existing Robotis KS hands only as a visual and kinematic approximation; hand geoms must have `contype="0"` and `conaffinity="0"` in this iteration.
- The package must not reference files outside `evt_diana7_sim` at runtime.
- Camera names are exactly `fixed_camera`, `left_wrist_camera`, and `right_wrist_camera`.
- Default rendered camera images are RGB `uint8`, 256×256, with wrist cameras mounted as mirrored inward-facing children of the two terminal links.
- Normal viewer use continues to require only `mujoco` and `numpy`; OpenCV is optional and imported only by the tiled camera preview.
- The directory is not a Git repository, so each task ends with a fresh test checkpoint instead of a commit.

## Review Focus

- Loading from an unrelated current directory must resolve every mesh relative to the XML, covered by Task 1's arbitrary-CWD load test.
- Adding hand qpos and actuators must not shift or rename the existing arm control interface, covered by Task 1's exact prefix-order assertions.
- A missing OpenCV installation must not break model loading or the normal control demo and must produce a clear preview-only error, covered by Task 4.
- Headless rendering without an X/Wayland window must work with `MUJOCO_GL=egl`, covered by Task 4's subprocess test.
- Window closure or `Ctrl+C` must release both MuJoCo and OpenCV resources, covered by Task 4's mocked cleanup test and manual smoke check.

---

### Task 1: Import and Attach the Articulated Hands

**Files:**
- Create: `diana7_description/hands/l-hand1-1.STL` through `l-hand5-2.STL`
- Create: `diana7_description/hands/r-hand1-1.STL` through `r-hand5-2.STL`
- Modify: `diana7_description/diana7_dual.xml`
- Create: `tests/test_diana7_model.py`

**Interfaces:**
- Consumes: the left/right hand body hierarchies and meshes from `../asset/robotis_ks/ROBOT_asm2.xml` only during implementation.
- Produces: 24 named hand joints (`L_hand1_1`…`L_hand5_2`, `R_hand1_1`…`R_hand5_2`, including thumb intermediate joints), 24 same-prefix position actuators, and a self-contained loadable scene.

- [ ] **Step 1: Write failing model tests**

Add tests that load `diana7_description/diana7_dual.xml` from a temporary unrelated working directory and assert:

- the original 14 joint names and actuator names are still the exact prefix of model order;
- all 24 prefixed hand joints and actuators exist and are unique;
- `model.nq == 38` and `model.nu == 38`;
- every geom belonging to a hand body has collision masks disabled;
- all mesh file references resolve inside `evt_diana7_sim`.

- [ ] **Step 2: Run the focused tests and confirm the expected failure**

Run: `.venv/bin/python -m pytest evt_diana7_sim/tests/test_diana7_model.py -v`

Expected: FAIL because the hand joints, actuators, and mesh files do not exist.

- [ ] **Step 3: Copy the 24 selected hand meshes into the standalone package**

Copy only the `l-hand*.STL` and `r-hand*.STL` files referenced by the chosen hand trees into `diana7_description/hands/`; preserve source files unchanged and reference only relative `hands/<name>.STL` paths in the XML.

- [ ] **Step 4: Add hand assets and prefixed hand body trees**

In `diana7_dual.xml`, add uniquely prefixed mesh assets and attach mirrored left/right hand roots below `L_link_7` and `R_link_7`. Retain source joint axes, ranges, body transforms, inertials, and visual colors; set all hand geom collision masks to zero. Choose the two root transforms so open palms extend away from each wrist and face the central work area without intersecting the arm mesh.

- [ ] **Step 5: Append hand actuators and extend the home keyframe**

Append 24 limited position actuators after the 14 arm actuators, using the source joint ranges with stable visualization gains (`kp=50`, `kv=5`). Extend the `home` keyframe with 24 open-hand joint values while leaving its first 14 qpos values byte-for-byte unchanged.

- [ ] **Step 6: Run the Task 1 tests**

Run: `.venv/bin/python -m pytest evt_diana7_sim/tests/test_diana7_model.py -v`

Expected: PASS for arbitrary-CWD loading, ordering, names, local assets, and disabled hand collisions.

### Task 2: Add the Fixed and Mirrored Wrist Cameras

**Files:**
- Modify: `diana7_description/diana7_dual.xml`
- Modify: `tests/test_diana7_model.py`

**Interfaces:**
- Consumes: `L_link_7`, `R_link_7`, and the attached hand roots from Task 1.
- Produces: named MuJoCo cameras `fixed_camera`, `left_wrist_camera`, and `right_wrist_camera` with approximately 70–75° vertical FOV.

- [ ] **Step 1: Add failing camera identity and attachment tests**

Assert that:

- `model.ncam == 3` and model camera names match the required order;
- the fixed camera is world-attached;
- left/right wrist cameras are descendants of the corresponding terminal links;
- the two wrist camera local positions are mirror-symmetric about the dual-arm centerline;
- changing only one arm's qpos changes only its wrist camera world pose after `mj_forward`.

- [ ] **Step 2: Run the camera tests and confirm failure**

Run: `.venv/bin/python -m pytest evt_diana7_sim/tests/test_diana7_model.py -k camera -v`

Expected: FAIL with missing camera names.

- [ ] **Step 3: Add the fixed camera**

Place `fixed_camera` in `worldbody` above and in front of the table, using `mode="targetbody"` against a small non-colliding target body/site at the central work area. Set `fovy` to 70–75° and frame both arms, both hands, and the tabletop.

- [ ] **Step 4: Add mirrored wrist camera bodies**

Create a small invisible camera mount under each terminal hand/wrist body. Use mirrored lateral offsets toward the space between the two arms and orientations aimed from the inner wrist across the palm toward the fingertips; name the cameras exactly as specified.

- [ ] **Step 5: Render calibration stills and tune approximate extrinsics**

Render the home pose from all three cameras at 256×256. Adjust only the wrist camera mount offsets/orientations until each image contains its palm and the majority of all five fingertips; adjust the fixed camera until both complete hands are visible.

- [ ] **Step 6: Run all model tests**

Run: `.venv/bin/python -m pytest evt_diana7_sim/tests/test_diana7_model.py -v`

Expected: PASS with exactly three cameras and correct attachment behavior.

### Task 3: Animate Arms and Dexterous Fingers Safely

**Files:**
- Modify: `diana7_control_demo.py`
- Create: `tests/test_diana7_control.py`

**Interfaces:**
- Consumes: the 38-actuator order established in Task 1.
- Produces: `compute_control_targets(model: mujoco.MjModel, home: np.ndarray, time_s: float) -> np.ndarray` and CLI options `--steps N` and `--headless`.

- [ ] **Step 1: Write failing control-target tests**

Test that `compute_control_targets`:

- returns shape `(38,)` with finite values;
- preserves the existing mirrored seven-joint arm sinusoid behavior;
- generates smooth bounded finger flexion with left/right phase offset;
- keeps every value inside `model.actuator_ctrlrange` at representative times `0`, `0.5`, `1`, and `10` seconds.

Add an integration test that resets the home keyframe, applies targets for 1000 control steps, and asserts finite `qpos`, `qvel`, `qacc`, and `ctrl` while the first 14 qpos entries remain the arm block.

- [ ] **Step 2: Run the tests and confirm failure**

Run: `.venv/bin/python -m pytest evt_diana7_sim/tests/test_diana7_control.py -v`

Expected: FAIL because `compute_control_targets` and CLI modes do not exist.

- [ ] **Step 3: Extract deterministic target generation**

Implement `compute_control_targets` in `diana7_control_demo.py`. Retain the current arm amplitudes/frequencies/phases, drive all finger joints between open and approximately 65% flexion using smooth sinusoids, and clip the combined vector to actuator limits.

- [ ] **Step 4: Add bounded and headless execution**

Add `argparse` options `--steps` (default unlimited in viewer mode) and `--headless`. Headless mode must step without importing or launching the viewer and print a compact JSON summary containing `steps`, `finite`, `nq`, `nu`, and `ncam`.

- [ ] **Step 5: Run control tests and a short headless smoke test**

Run: `.venv/bin/python -m pytest evt_diana7_sim/tests/test_diana7_control.py -v`

Run: `.venv/bin/python evt_diana7_sim/diana7_control_demo.py --headless --steps 100`

Expected: tests PASS; JSON reports `steps=100`, `finite=true`, `nq=38`, `nu=38`, `ncam=3`.

### Task 4: Build the Three-Camera Preview

**Files:**
- Create: `camera_preview.py`
- Create: `tests/test_camera_preview.py`

**Interfaces:**
- Consumes: `compute_control_targets(...)` from Task 3 and the three camera names from Task 2.
- Produces: `render_camera_grid(renderer: mujoco.Renderer, data: mujoco.MjData, camera_names: tuple[str, ...]) -> np.ndarray`, returning a `(256, 768, 3)` RGB `uint8` array, plus CLI options `--steps N` and `--headless`.

- [ ] **Step 1: Write failing renderer and CLI tests**

Test that:

- rendering the three required cameras returns RGB `uint8` shape `(256, 768, 3)`;
- each 256-pixel panel has nonzero variance and differs from the other two;
- finger motion changes the matching wrist panel;
- importing `diana7_control_demo` and running its headless path succeeds when `cv2` import is blocked;
- preview startup without `cv2` raises a concise message instructing `pip install opencv-python`;
- mocked viewer/window closure runs renderer cleanup and `cv2.destroyAllWindows()` exactly once.

- [ ] **Step 2: Run preview tests and confirm failure**

Run with EGL: `MUJOCO_GL=egl .venv/bin/python -m pytest evt_diana7_sim/tests/test_camera_preview.py -v`

Expected: FAIL because `camera_preview.py` does not exist.

- [ ] **Step 3: Implement off-screen three-camera rendering**

Create one reusable `mujoco.Renderer` at 256×256, update and render each camera in fixed order, then concatenate the three RGB arrays horizontally. Keep rendering logic independent of OpenCV so headless tests need no GUI.

- [ ] **Step 4: Implement live and headless preview loops**

In live mode, convert the grid to BGR only at the OpenCV boundary, add camera-name labels, display one window, and stop on `q`, Escape, window close, `Ctrl+C`, or `--steps`. In headless mode, render the requested number of steps and print JSON with image shape, per-panel variance, step count, and finite-state status.

- [ ] **Step 5: Run preview tests and the headless subprocess**

Run: `MUJOCO_GL=egl .venv/bin/python -m pytest evt_diana7_sim/tests/test_camera_preview.py -v`

Run: `MUJOCO_GL=egl .venv/bin/python evt_diana7_sim/camera_preview.py --headless --steps 20`

Expected: tests PASS; JSON reports shape `[256, 768, 3]`, positive variance for all three panels, and `finite=true`.

### Task 5: Documentation and End-to-End Verification

**Files:**
- Modify: `README.md`
- Test: `tests/test_diana7_model.py`
- Test: `tests/test_diana7_control.py`
- Test: `tests/test_camera_preview.py`

**Interfaces:**
- Consumes: completed model, control demo, and preview CLI.
- Produces: copy-and-run instructions for normal viewer, animated hands, three-camera live preview, and EGL headless verification.

- [ ] **Step 1: Update the package inventory and dependency notes**

Document the new `hands/` assets, 38-qpos/38-actuator ordering, the three exact camera names, `opencv-python` as an optional live-preview dependency, and the limitation that hand collisions/grasp dynamics are intentionally disabled.

- [ ] **Step 2: Add exact run commands**

Document commands using the project virtual environment for:

- the free-view viewer;
- the dual-arm plus finger animation;
- the live three-camera grid;
- the EGL headless smoke test.

- [ ] **Step 3: Run the full automated suite from an unrelated directory**

Run from `/tmp`:

`MUJOCO_GL=egl /srv/drone/Desktop/Diana双臂资料/EVT-VLA3-50/.venv/bin/python -m pytest /srv/drone/Desktop/Diana双臂资料/EVT-VLA3-50/evt_diana7_sim/tests -v`

Expected: all tests PASS with no missing assets or working-directory dependence.

- [ ] **Step 4: Run fresh headless end-to-end checks**

Run the control demo for 1000 steps and the camera preview for 100 steps. Expected: both exit zero, report finite state, and the camera grid reports positive variance in all panels.

- [ ] **Step 5: Perform a manual GUI smoke check**

Launch the normal animation and the three-camera preview. Confirm that the fixed view contains both hands, each wrist view contains its corresponding palm and moving fingertips, window closure terminates cleanly, and no hand mesh visibly intersects its forearm at the home pose.
