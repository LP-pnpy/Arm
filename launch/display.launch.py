import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare(package='my_robot_description').find('my_robot_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'arm.rviz')

    # 使用 Command 执行 xacro，并将结果转为字符串
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description], # 传入解析后的 XML 字符串，而不是文件路径
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])