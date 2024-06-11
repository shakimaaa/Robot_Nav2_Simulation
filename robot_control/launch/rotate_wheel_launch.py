import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取参数文件的路径
    package_share_directory = FindPackageShare(package='robot_control').find('robot_control')
    params_file = os.path.join(package_share_directory, 'config', 'params.yaml')

    # 创建并返回LaunchDescription对象
    return LaunchDescription([
        Node(
            package='robot_control',
            executable='rotate_wheel_node',
            name='rotate_wheel',
            output='screen',
            parameters=[params_file]
        )
    ])
