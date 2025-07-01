from setuptools import setup

package_name = 'right_tech_arm_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'robot_arm_control_node = right_tech_arm_description.robot_arm_control_node:main',  # 确保路径正确
        ],
    },
    zip_safe=True,
)
