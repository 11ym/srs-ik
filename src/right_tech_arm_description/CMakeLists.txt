cmake_minimum_required(VERSION 3.5)
project(right_tech_arm_description)

# 查找 ROS 2 的依赖
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

# 安装 URDF、mesh、launch、rviz 配置文件
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 脚本
install(
  PROGRAMS
    src/right_tech_arm_description/robot_arm_control_node.py  # 修改为正确的路径
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
