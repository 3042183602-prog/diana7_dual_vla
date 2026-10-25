# Diana7 双臂精密装配 仿真训练计划（v6）

## 0. 题目

《基于深度强化学习的零部件精密装配策略训练与仿真验证》

## 1. 总体路线

1. BC / VLA 预训练（SmolVLA / ACT 等）
2. DRL 后训练（PPO / SAC，力传感器反馈）
3. 课程学习 + 域随机化
4. Sim-to-Real → Jetson AGX Thor 端侧部署

## 2. 资产边界

- 我们自己的机械臂：evt_diana7_sim 的 Diana7 双臂（左右各 7 关节）。
- 博士的 asset/scene_table_ks：仅作相机/场景参考，不作为我们的基础。
- 真机相机、灵巧手未装；仿真先行。

## 3. 灵巧手（真机资料已查）

来自 手windows相关文件_python3(1)/Prosthesis_Hand_Demo：
- 型号：Prosthesis Hand（假手）
- 自由度：6 个指节 J0–J5（joint_id 0–5），每个电机一个关节
- 通信：CAN 2.0A，1Mbps
- 模式：电流模式（current_loop）、位置模式（pos_loop）
- 状态机：init → preop → op
- 控制：send_ctrl(cmd, joint_id, values)，int16 补码
- 资料中暂无 3D/CAD 模型，只有控制协议与用户手册

**仿真建模策略**：
- 第一版先做简化手/夹持器（或末端工具），保证轴孔装配环境先跑通。
- 后续若有手部 3D 模型，再按 J0–J5 六关节建模替换。

## 4. 仿真环境（M1）

- 基于 evt_diana7_sim Diana7 双臂。
- 任务：轴孔插入（peg-in-hole），支持公差与初始位姿随机化。
- 感知：六维力/力矩传感器 + 关节状态 + 可选相机。
- 动作：末端六维增量（或关节位置增量）。
- 成功判定：插入深度 + 姿态误差 + 接触力峰值约束。

## 5. MDP 建模

- 状态：关节角度、末端位姿、六维力/力矩（+ 可选视觉特征）。
- 奖励：稀疏成功 + 密集（接近孔、对准、插入深度）+ 接触力惩罚。

## 6. 算法

- BC/VLA：SmolVLA 先行，ACT / pi0.5 等对比。
- DRL：PPO、SAC（Stable-Baselines3 或自研），服务器 RTX PRO 6000。

## 7. 里程碑

- M1：MuJoCo 轴孔装配环境 + 力传感器 + 成功判定。
- M2：演示数据生成 + LeRobot v3 + 回放校验。
- M3：BC/VLA 预训练。
- M4：DRL 后训练（PPO/SAC）。
- M5：课程学习/域随机化/对比评估。
- M6：Jetson AGX Thor 端侧验证。

## 8. 下一步

1. 开始 M1：搭建 Diana7 双臂轴孔装配 MuJoCo 环境（含力传感器）。
2. 渲染校验后进入 M2。
