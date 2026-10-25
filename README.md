# Diana7 双臂精密装配仿真环境

基于 evt_diana7_sim 的 Diana7 双臂 MuJoCo 模型搭建的精密装配仿真环境。

## 当前进度（M1 第一步）

- 双臂 Diana7 模型（左右各 7 关节）
- 三个摄像机：
  - head_camera：头部/全局视角
  - L_hand_camera：左腕相机
  - R_hand_camera：右腕相机
- 两个末端六维力/力矩传感器（左右各一个 force + torque）：
  - L_force / L_torque
  - R_force / R_torque

## 目录结构

    diana7_dual_vla/
    ├── diana7_description/            # 原始双臂模型与 mesh
    ├── tasks/
    │   ├── diana7_dual_sensors.xml    # 带三相机+力传感器场景
    │   └── preview/                   # 三相机渲染预览图
    ├── build_sensor_scene.py          # 从原始模型生成场景
    ├── preview_sensors.py             # 加载/渲染/读传感器测试脚本
    └── README.md

## 使用方法

依赖：mujoco >= 3.2、numpy、Pillow

    # 查看场景
    python open_sim_viewer.py tasks/diana7_dual_sensors.xml

    # 验证相机与力传感器
    python preview_sensors.py

## 下一步

- 加装配任务物体（轴/孔）
- 加灵巧手模型
- 接入 Gymnasium 环境
