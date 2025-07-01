# srs-ik

逆运动学（Inverse Kinematics）相关的 ROS2 项目，实现 SRS 构型机械臂的逆解。

## 🚀 特性

- 基于 ROS2 构建，支持实时控制
- 实现双机械臂逆运动学求解
- 易于扩展和二次开发

## 📦 安装

```bash
# 克隆仓库
git clone https://github.com/11ym/srs-ik.git

# 进入工作空间
cd srs-ik

# 安装依赖（根据项目实际情况修改）
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# 编译工作空间
colcon build

## 🔧 使用方法
# 加载环境
source install/setup.bash

# 启动机械臂描述
ros2 launch right_tech_arm_description robot_description.launch.py

# 启动机械臂控制
ros2 launch right_tech_arm_description robot_arm_control_launch.py
