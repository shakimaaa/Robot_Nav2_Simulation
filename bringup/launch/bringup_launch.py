import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取 robot_control 和 robot_description_py 包的路径
    robot_control_share = FindPackageShare(package='robot_control').find('robot_control')
    robot_description_py_share = FindPackageShare(package='robot_description_py').find('robot_description_py')

    # 获取参数文件路径
    robot_control_params = os.path.join(robot_control_share, 'config', 'params.yaml')

    # 包含 robot_description_py 的 launch 文件
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_py_share, 'launch', 'display_rviz2.launch.py')
        )
    )

    # 创建并返回LaunchDescription对象
    return LaunchDescription([
        robot_description_launch,
        Node(
            package='robot_control',
            executable='rotate_wheel_node',
            name='rotate_wheel',
            output='screen',
            parameters=[robot_control_params]
        ),
    ])
