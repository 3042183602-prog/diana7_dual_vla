from pathlib import Path
src = Path("tasks/diana7_dual_sensors.xml").read_text(encoding="utf-8")

# 在 worldbody 中插入孔底座（固定）
hole_base = '''    <body name="hole_base" pos="0.20 0.10 0.7725">
      <inertial pos="0 0 0.01" mass="0.2" diaginertia="0.0005 0.0005 0.0003"/>
      <geom name="hole_base_geom" type="box" pos="0 0 0.01" size="0.06 0.06 0.01" rgba="0.40 0.45 0.60 1"/>
      <geom name="hole_wall_n" type="box" pos="0 0.014 0.02" size="0.004 0.004 0.02" rgba="0.30 0.35 0.50 1"/>
      <geom name="hole_wall_s" type="box" pos="0 -0.014 0.02" size="0.004 0.004 0.02" rgba="0.30 0.35 0.50 1"/>
      <geom name="hole_wall_e" type="box" pos="0.014 0 0.02" size="0.004 0.004 0.02" rgba="0.30 0.35 0.50 1"/>
      <geom name="hole_wall_w" type="box" pos="-0.014 0 0.02" size="0.004 0.004 0.02" rgba="0.30 0.35 0.50 1"/>
      <site name="hole_center" pos="0 0 0.02" size="0.005" rgba="0.10 0.90 0.30 1"/>
    </body>
'''
anchor = '    <camera name="head_camera"'
idx = src.index(anchor)
src = src[:idx] + hole_base + src[idx:]

# 在右臂 link7 内插入轴（第一版：轴固定在右末端，无需夹爪）
peg = '''                    <body name="R_peg" pos="0 0 0.045" quat="1 0 0 0">
                      <inertial pos="0 0 -0.025" mass="0.03" diaginertia="0.00002 0.00002 0.00002"/>
                      <geom name="R_peg_geom" type="cylinder" pos="0 0 -0.025" size="0.008 0.045" rgba="0.90 0.65 0.10 1"/>
                    </body>
'''
cam_anchor = '<camera name="R_hand_camera"'
idx2 = src.index(cam_anchor)
# 找到该 camera 行的行尾
line_end = src.index(chr(10), idx2) + 1
src = src[:line_end] + peg + src[line_end:]

out = Path("tasks/diana7_dual_peg_hole.xml")
out.write_text(src, encoding="utf-8")
print("wrote", out)
