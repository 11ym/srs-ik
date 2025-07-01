# launch/robot_arm_control_launch.py

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='right_tech_arm_description',  # 包名
            executable='robot_arm_control_node.py',   # Python 脚本名称
            name='robot_arm_control_node',         # 给节点指定一个名字
            output='screen',                       # 输出到屏幕
        ),
    ])
