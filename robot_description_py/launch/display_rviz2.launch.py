import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #  功能包名和URDF文件名
    package_name = 'robot_description_py'
    urdf_name = 'robot_base.urdf'

    #  创建一个LaunchDescription对象
    #  ROS 2 中用于描述启动文件的类。
    #  通过这个对象，你可以向启动文件中添加各种节点、参数等，从而定义整个系统的启动配置。
    #  在这个例子中，ld 用于构建一个包含了三个节点的启动配置，
    #  这三个节点分别用于发布 URDF 模型的 TF 数据、手动控制关节状态以及可视化机器人模型和 TF 数据。
    ld = LaunchDescription()

    #  找到urdf文件
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    #  创建一个节点用来发布urdf模型的数据
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path] 
        #  指定节点运行时的参数,这里是指定加载urdf文件的路径
    )

    #  创建一个节点，用于手动控制关节状态
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        #  这指定了节点的名称
        #  在这个例子中，joint_state_publisher_gui 是节点的名称，
        #  可以在启动文件中通过这个名称来引用和操作这个节点。

        arguments=[urdf_model_path]
    )

     #  创建一个节点，用于可视化机器人模型和TF数据
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        #  节点的日志将直接显示在终端或控制台上
    )

    #  将三个节点加入到启动描述内
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld