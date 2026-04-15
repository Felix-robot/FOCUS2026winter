# FOCUS 2026 冬令营 - 视觉与仿真赛道成果提交

**作者**：杨子昊
**赛道**：具身智能与仿真 (Embodied AI & Simulation)

## 项目概述
本项目基于 Unitree Go1 机器狗仿真平台，实现了从底层 ROS 接口改造到上层自主逻辑控制的全链路开发。

## 已完成任务
- **阶段一**：完成 ROS Noetic 环境搭建与 Turtlesim 验证。
- **阶段二**：审计 `unitree_guide` 源码，通过修改 `IOROS` 类成功引入 `/cmd_vel` 订阅接口，实现了外部代码对接。
- **阶段三**：编写 Python 脚本控制机器狗完成闭合正方形轨迹；初步构建了基于颜色分割的视觉追踪系统。

## 运行说明
1. 启动仿真：`roslaunch unitree_guide gazeboSim.launch`
2. 启动控制器：`rosrun unitree_guide junior_ctrl` (按键 2 站立，按键 4 进入 Trotting)
3. 运行自动化脚本：`python3 src/square_move.py`