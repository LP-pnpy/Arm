import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. 获取各包的路径 (使用 PathJoinSubstitution 是 ROS 2 最规范的做法)
    pkg_share_desc = FindPackageShare(package='my_robot_description')
    pkg_share_bringup = FindPackageShare(package='my_robot_bringup')
    pkg_share_moveit = FindPackageShare(package='my_robot_moveit_config')

    urdf_path = PathJoinSubstitution([pkg_share_desc, 'urdf', 'my_robot.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg_share_desc, 'rviz', 'arm.rviz'])
    controllers_yaml = PathJoinSubstitution([pkg_share_bringup, 'config', 'ros2_controllers.yaml'])
    
    # 2. 解析 URDF (Xacro 转 URDF)
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 3. 定义各节点
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Ros2 Control Node
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen'
    )

    # 4. 使用 TimerAction 延迟启动 Spawner，等待 control_node 启动完毕
    # 注意：如果你的 YAML 里没有包含控制器参数，Spawner 需要加 --param-file 参数
    spawner_joint = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--param-file', controllers_yaml],
            output='screen'
        )]
    )

    spawner_arm = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller', '--param-file', controllers_yaml],
            output='screen'
        )]
    )

    spawner_gripper = TimerAction(
        period=6.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['grapper_controller', '--param-file', controllers_yaml],
            output='screen'
        )]
    )

    # 5. 包含 MoveIt 的 move_group.launch.py 
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share_moveit, 'launch', 'move_group.launch.py'])
        ])
    )

    # 6. 启动 RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # (可选) 如果你希望手动用 GUI 拖拽关节来测试，可以保留这个，但在跑 MoveIt 时通常不需要
    # joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     output='screen'
    # )

    # 7. 返回 LaunchDescription，控制启动顺序
    return LaunchDescription([
        robot_state_publisher,
        control_node,
        spawner_joint,
        spawner_arm,
        spawner_gripper,
        move_group_launch,
        rviz2,
        # joint_state_publisher_gui,
    ])
