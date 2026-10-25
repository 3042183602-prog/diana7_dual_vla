from pathlib import Path
src = Path("diana7_description/diana7_dual.xml").read_text(encoding="utf-8")
# 新场景位于 tasks/ 目录，mesh 路径需要指回 diana7_description/
src = src.replace('file="', 'file="../diana7_description/')

LINK7_GEOM = '                  <geom type="mesh" mesh="link_7_v" group="2" contype="0" conaffinity="0" />\n'
assert src.count(LINK7_GEOM) == 2, src.count(LINK7_GEOM)

left_insert = '''                    <site name="L_tool_mount" pos="0 0 0.045" size="0.012" rgba="0.95 0.25 0.15 1"/>
                    <camera name="L_hand_camera" pos="0 0 0.075" xyaxes="-0.24 0 -0.97 0 1 0" fovy="70"/>
'''
right_insert = '''                    <site name="R_tool_mount" pos="0 0 0.045" size="0.012" rgba="0.95 0.25 0.15 1"/>
                    <camera name="R_hand_camera" pos="0 0 0.075" xyaxes="-0.24 0 -0.97 0 1 0" fovy="70"/>
'''
src = src.replace(LINK7_GEOM, left_insert + LINK7_GEOM, 1)
src = src.replace(LINK7_GEOM, right_insert + LINK7_GEOM, 1)

head_cam = '    <camera name="head_camera" mode="targetbody" target="R_link_1" pos="0.4 -0.3 0.6" fovy="70"/>\n'
anchor = '    <light pos="0 0 3.5" dir="0 0 -1" directional="true" diffuse="0.9 0.9 0.9" />\n'
assert anchor in src
src = src.replace(anchor, anchor + head_cam, 1)

sensors = '''  <sensor>
    <force name="L_force" site="L_tool_mount" noise="0"/>
    <torque name="L_torque" site="L_tool_mount" noise="0"/>
    <force name="R_force" site="R_tool_mount" noise="0"/>
    <torque name="R_torque" site="R_tool_mount" noise="0"/>
  </sensor>
</mujoco>
'''
src = src.replace('</mujoco>', sensors, 1)

out = Path("tasks")
out.mkdir(exist_ok=True)
(out / "diana7_dual_sensors.xml").write_text(src, encoding="utf-8")
print("wrote", out / "diana7_dual_sensors.xml")
