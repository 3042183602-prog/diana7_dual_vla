import mujoco
import numpy as np
from pathlib import Path

xml = Path("tasks/diana7_dual_sensors.xml")
m = mujoco.MjModel.from_xml_path(str(xml))
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

print("nq =", m.nq, "nu =", m.nu)
print("cameras:", [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(m.ncam)])
print("sensors:")
for i in range(m.nsensor):
    print("  ", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i), "dim=", m.sensor_dim[i])

# 力/力矩传感器初始读数
for name in ["L_force","L_torque","R_force","R_torque"]:
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name)
    adr = m.sensor_adr[sid]
    dim = m.sensor_dim[sid]
    print(name, "adr", adr, "dim", dim, "data", d.sensordata[adr:adr+dim])

out = Path("tasks/preview")
out.mkdir(exist_ok=True)
for cam in ["head_camera","L_hand_camera","R_hand_camera"]:
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam)
    renderer = mujoco.Renderer(m, height=480, width=640)
    renderer.update_scene(d, camera=cid)
    img = renderer.render()
    p = out / f"{cam}.png"
    from PIL import Image
    Image.fromarray(img).save(p)
    print("saved", p, img.shape, img.mean(), img.min(), img.max())
