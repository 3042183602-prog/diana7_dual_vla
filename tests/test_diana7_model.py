from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCENE_XML = PACKAGE_ROOT / "diana7_description" / "diana7_dual.xml"

ARM_NAMES = tuple(
    [f"L_joint_{index}" for index in range(1, 8)]
    + [f"R_joint_{index}" for index in range(1, 8)]
)
HAND_SUFFIXES = (
    "hand1_1",
    "hand1_2",
    "hand1_3",
    "hand1_4",
    "hand2_1",
    "hand2_2",
    "hand3_1",
    "hand3_2",
    "hand4_1",
    "hand4_2",
    "hand5_1",
    "hand5_2",
)
HAND_NAMES = tuple(
    f"{side}_{suffix}" for side in ("L", "R") for suffix in HAND_SUFFIXES
)


def load_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(SCENE_XML))


def named_objects(model: mujoco.MjModel, object_type: mujoco.mjtObj, count: int):
    return tuple(mujoco.mj_id2name(model, object_type, index) for index in range(count))


def test_scene_loads_with_all_assets_from_unrelated_working_directory(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)

    model = load_model()

    assert model.nq == 38
    assert model.nu == 38


def test_arm_order_is_preserved_and_hand_controls_are_appended():
    model = load_model()
    joint_names = named_objects(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    actuator_names = named_objects(model, mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)

    assert tuple(name for name in joint_names if name in ARM_NAMES) == ARM_NAMES
    assert actuator_names[:14] == ARM_NAMES
    assert tuple(name for name in joint_names if name in HAND_NAMES) == HAND_NAMES
    assert actuator_names[14:] == HAND_NAMES
    assert len(set(joint_names)) == len(joint_names)
    assert len(set(actuator_names)) == len(actuator_names)


def test_hand_geometries_are_visual_only():
    model = load_model()
    hand_body_ids = {
        index
        for index in range(model.nbody)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index) or "").startswith(
            ("L_hand", "R_hand")
        )
    }
    hand_geom_ids = [
        index for index, body_id in enumerate(model.geom_bodyid) if int(body_id) in hand_body_ids
    ]

    assert len(hand_body_ids) == 24
    assert len(hand_geom_ids) == 24
    assert all(model.geom_contype[index] == 0 for index in hand_geom_ids)
    assert all(model.geom_conaffinity[index] == 0 for index in hand_geom_ids)

def test_wrist_flange_adapters_overlap_link7_and_palm_visuals():
    model = load_model()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    for side in ("L", "R"):
        flange_id = model.geom(f"{side}_wrist_flange").id
        link_body_id = model.body(f"{side}_link_7").id
        palm_id = model.geom(f"{side}_palm_visual").id
        link_geom_id = next(
            geom_id
            for geom_id in range(model.ngeom)
            if model.geom_bodyid[geom_id] == link_body_id
            and model.geom_dataid[geom_id] >= 0
            and mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, int(model.geom_dataid[geom_id]))
            == "link_7_v"
        )

        assert model.geom_bodyid[flange_id] == link_body_id
        assert model.geom_contype[flange_id] == 0
        assert model.geom_conaffinity[flange_id] == 0
        assert mujoco.mj_geomDistance(model, data, flange_id, link_geom_id, 0.1, None) < 0.0
        assert mujoco.mj_geomDistance(model, data, flange_id, palm_id, 0.1, None) < 0.0

def test_wrist_six_axis_force_torque_sensors_are_attached_and_finite():
    model = load_model()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    assert model.nsensor == 4
    assert model.nsensordata == 12
    for side in ("L", "R"):
        site_id = model.site(f"{side}_wrist_fts").id
        assert model.site_bodyid[site_id] == model.body(f"{side}_palm_mount").id
        for kind, dimension in (("force", 3), ("torque", 3)):
            sensor_id = model.sensor(f"{side}_wrist_{kind}").id
            assert model.sensor_dim[sensor_id] == dimension
            assert model.sensor_adr[sensor_id] >= 0

    assert np.all(np.isfinite(data.sensordata))
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.sensordata))


def test_all_mesh_references_stay_inside_the_standalone_package():
    description_root = SCENE_XML.parent.resolve()
    xml_root = ET.parse(SCENE_XML).getroot()
    mesh_files = [element.attrib["file"] for element in xml_root.findall("./asset/mesh")]

    assert any(filename.startswith("hands/") for filename in mesh_files)
    for filename in mesh_files:
        resolved = (description_root / filename).resolve()
        assert resolved.is_relative_to(description_root)
        assert resolved.is_file(), filename


def body_ancestors(model: mujoco.MjModel, body_id: int) -> set[str]:
    names = set()
    while body_id > 0:
        names.add(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id))
        body_id = int(model.body_parentid[body_id])
    return names


def test_three_cameras_have_fixed_names_and_expected_attachments():
    model = load_model()
    camera_names = named_objects(model, mujoco.mjtObj.mjOBJ_CAMERA, model.ncam)

    assert camera_names == ("fixed_camera", "left_wrist_camera", "right_wrist_camera")
    fixed_id = model.camera("fixed_camera").id
    left_id = model.camera("left_wrist_camera").id
    right_id = model.camera("right_wrist_camera").id
    assert model.cam_bodyid[fixed_id] == 0
    assert "L_link_7" in body_ancestors(model, int(model.cam_bodyid[left_id]))
    assert "R_link_7" in body_ancestors(model, int(model.cam_bodyid[right_id]))
    np.testing.assert_allclose(
        model.cam_pos[left_id] * np.array([1.0, -1.0, 1.0]),
        model.cam_pos[right_id],
        atol=1e-9,
    )

    palm_geoms = (model.geom("L_palm_visual"), model.geom("R_palm_visual"))
    assert all(geom.contype[0] == 0 and geom.conaffinity[0] == 0 for geom in palm_geoms)


def test_wrist_camera_extrinsics_are_rigid_to_the_wrist():
    model = load_model()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    camera_ids = (model.camera("left_wrist_camera").id, model.camera("right_wrist_camera").id)
    positions_before = data.cam_xpos[list(camera_ids)].copy()
    rotations_before = data.cam_xmat[list(camera_ids)].copy()

    for name in ("L_hand3_1", "R_hand3_1"):
        joint_id = model.joint(name).id
        data.qpos[model.jnt_qposadr[joint_id]] += 0.5
    mujoco.mj_forward(model, data)

    np.testing.assert_allclose(data.cam_xpos[list(camera_ids)], positions_before, atol=1e-12)
    np.testing.assert_allclose(data.cam_xmat[list(camera_ids)], rotations_before, atol=1e-12)


def test_each_wrist_camera_moves_only_with_its_own_arm():
    model = load_model()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    left_camera = model.camera("left_wrist_camera").id
    right_camera = model.camera("right_wrist_camera").id
    left_before = data.cam_xpos[left_camera].copy()
    right_before = data.cam_xpos[right_camera].copy()

    left_joint = model.joint("L_joint_1").id
    data.qpos[model.jnt_qposadr[left_joint]] += 0.4
    mujoco.mj_forward(model, data)

    assert np.linalg.norm(data.cam_xpos[left_camera] - left_before) > 1e-3
    np.testing.assert_allclose(data.cam_xpos[right_camera], right_before, atol=1e-12)
