# srs-ik

逆运动学（Inverse Kinematics）相关的 ROS2 项目，实现 SRS 构型机械臂的逆解。

## 🚀 特性

- 基于 ROS2 构建，支持实时控制
- 实现双机械臂逆运动学求解
- 易于扩展和二次开发

srs-ik/
├── src/                # 源代码，主要实现基于臂角参数法的逆运动学求解
├── launch/             # 启动文件，包含机械臂可视化及逆运动学节点的启动脚本
├── urdf/               # 机械臂的 URDF 模型文件
├── README.md           # 项目说明文档
└── package.xml         # ROS2 包描述文件




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
