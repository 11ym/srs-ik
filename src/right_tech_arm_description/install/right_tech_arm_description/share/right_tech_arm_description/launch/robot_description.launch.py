#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包目录
    pkg_share = get_package_share_directory('right_tech_arm_description')
    
    # 设置 URDF 文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'right_arm.urdf')
    
    # 读取 URDF 文件
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 返回 LaunchDescription 对象
    return LaunchDescription([
        # 启动 joint_state_publisher_gui 节点
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        
        # 启动 robot_state_publisher 节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 启动 rviz2 节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),

        # # 启动 robot_arm_control_node 节点
        # Node(
        #     package='right_tech_arm_description',
        #     executable='robot_arm_control_node',  # 确保此处是正确的可执行文件名
        #     name='robot_arm_control_node',
        #     output='screen'
        # ),
    ])
