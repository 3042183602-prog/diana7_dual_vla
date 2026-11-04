# Diana7 双臂、灵巧手与三相机 MuJoCo 仿真

这个目录是一套可独立运行的近似可视化模型，包含：

- 两台 Diana7 七轴机械臂；
- 两只 Robotis KS 外形的 12 关节灵巧手，共 24 个手指关节；
- 一台固定相机，以及安装在两只手内侧的左右腕部相机；
- 双臂和手指的示例动画；
- 三路相机的并排预览程序。

灵巧手的手掌外壳与手指网格已经随模型放在 `diana7_description/hands/`，运行时不依赖原始素材目录。

## 环境

必需依赖：

```bash
python3 -m pip install mujoco numpy
```

三路相机实时窗口还需要 OpenCV：

```bash
python3 -m pip install opencv-python
```

测试使用 `pytest`。如果项目上一级已经有 `.venv`，可把下文的 `python3` 换成 `../.venv/bin/python`。

## 运行

打开 MuJoCo 交互窗口：

```bash
python3 open_sim_viewer.py
```

播放双臂和灵巧手动画：

```bash
python3 diana7_control_demo.py
```

不打开窗口，运行有限步数并输出 JSON 状态：

```bash
python3 diana7_control_demo.py --headless --steps 1000
```

打开三路相机并排预览（固定、左腕、右腕，每路 256×256）：

```bash
python3 camera_preview.py
```

在无显示器环境中检查三路相机：

```bash
MUJOCO_GL=egl python3 camera_preview.py --headless --steps 100
```

脚本会根据自身位置寻找模型，因此可以从任意工作目录启动。实时相机窗口按 `q`、`Esc` 或关闭窗口退出。

## 模型接口

模型共有 38 个单自由度关节和 38 个位置执行器：14 个机械臂关节，加上 24 个手指关节。

执行器顺序保持原机械臂接口在前：

1. `L_joint_1` 至 `L_joint_7`；
2. `R_joint_1` 至 `R_joint_7`；
3. 左手 12 个关节；
4. 右手 12 个关节。

MuJoCo 的 `qpos` 按刚体树遍历排列，因此它的顺序是：左臂 7 个、左手 12 个、右臂 7 个、右手 12 个。外部程序不要假定 `qpos[7:14]` 是右臂；应通过关节名称取得 `qpos` 地址。示例控制程序已经按名称映射。

相机名称固定为：

- `fixed_camera`
- `left_wrist_camera`
- `right_wrist_camera`

左右腕部相机以固定外参安装在手内侧，朝向掌心和指尖运动区域；手指弯曲不会改变相机相对手腕的姿态。

## 腕部力/力矩传感器

左右腕部各有一个六轴力/力矩传感器：

- `L_wrist_force` / `L_wrist_torque`
- `R_wrist_force` / `R_wrist_torque`

四个传感器共 12 个数据分量，数据位于 `data.sensordata`，坐标系为对应腕部传感器 site 的局部坐标系：

```python
def read_sensor(model, data, name):
    sensor_id = model.sensor(name).id
    address = model.sensor_adr[sensor_id]
    return data.sensordata[address:address + model.sensor_dim[sensor_id]].copy()

left_force = read_sensor(model, data, "L_wrist_force")
left_torque = read_sensor(model, data, "L_wrist_torque")
```

传感器数值是近似仿真输出，未做真实传感器量程、零点、噪声和温漂标定。

## 当前近似与限制

- 灵巧手的外形和运动链来自现有 Robotis KS 网格，但与真实硬件的安装变换、尺寸和相机内参尚未做标定。
- 左右 `link_7` 远端与手掌壳之间增加了深灰色视觉转接法兰，用于近似机械连接；法兰不参与碰撞，也不改变手部运动链。
- 手部网格目前用于运动与视觉展示，手部碰撞已关闭；模型不用于验证抓取接触力或稳定性。
- 固定相机与腕部相机的位置和视场角是为了清楚观察动作而调出的近似值，不代表真实标定结果。

## 测试

```bash
MUJOCO_GL=egl PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests -v
```
