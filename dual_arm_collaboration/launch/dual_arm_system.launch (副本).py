# dual_arm_collaboration/launch/dual_arm_system.launch.py
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 聲明參數
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur5_robot_ip",
            default_value="192.168.1.3",
            description="IP address of UR5 robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm0710_robot_ip", 
            default_value="192.168.1.10",
            description="IP address of ARM0710 controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm0710_target_ip",
            default_value="192.168.1.20", 
            description="Target IP for ARM0710",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz for visualization",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware for testing",
        )
    )

    # 配置參數
    ur5_robot_ip = LaunchConfiguration("ur5_robot_ip")
    arm0710_robot_ip = LaunchConfiguration("arm0710_robot_ip")
    arm0710_target_ip = LaunchConfiguration("arm0710_target_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # 雙臂系統描述
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("dual_arm_collaboration"),
            "urdf",
            "dual_arm_system.urdf.xacro"
        ]),
        " ur5_robot_ip:=", ur5_robot_ip,
        " arm0710_robot_ip:=", arm0710_robot_ip,
    ])

    robot_description = {"robot_description": robot_description_content}

    # 機器人狀態發布者
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # UR5 控制系統
    ur5_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur5',
            'robot_ip': ur5_robot_ip,
            'tf_prefix': 'ur5_',
            'launch_rviz': 'false',
            'use_fake_hardware': use_fake_hardware,
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
        }.items()
    )

    # ARM0710 控制節點
    arm0710_controller_node = Node(
        package="dual_arm_collaboration",
        executable="arm0710_controller.py",
        name="arm0710_controller",
        namespace="arm0710",
        output="screen",
        parameters=[
            {"robot_ip": arm0710_robot_ip},
            {"target_ip": arm0710_target_ip},
            {"tf_prefix": "arm0710_"},
        ]
    )

    # 雙臂協作控制器
    dual_arm_coordinator_node = Node(
        package="dual_arm_collaboration", 
        executable="dual_arm_coordinator.py",
        name="dual_arm_coordinator",
        output="screen",
        parameters=[
            {"ur5_planning_group": "ur_manipulator"},
            {"arm0710_planning_group": "arm0710_manipulator"},
            {"workspace_limits": PathJoinSubstitution([
                FindPackageShare("dual_arm_collaboration"),
                "config", 
                "workspace_limits.yaml"
            ])},
        ]
    )

    # RViz 配置
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("dual_arm_collaboration"),
        "rviz",
        "dual_arm_view.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2", 
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
        condition=launch.conditions.IfCondition(launch_rviz),
    )

    # 返回啟動描述
    nodes = [
        robot_state_publisher_node,
        ur5_control_launch,
        arm0710_controller_node,
        dual_arm_coordinator_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

# 必要的 import
import launch.conditions