# dual_arm_collaboration/launch/dual_arm_system.launch.py
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # 聲明參數
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'ur5_robot_ip',
            default_value='192.168.1.3',
            description='UR5 機器人 IP 位址'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm0710_robot_ip',
            default_value='192.168.1.10',
            description='ARM0710 機器人 IP 位址'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='是否啟動 RViz'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='是否使用模擬硬體'
        )
    )

    # 獲取參數
    ur5_robot_ip = LaunchConfiguration('ur5_robot_ip')
    arm0710_robot_ip = LaunchConfiguration('arm0710_robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    # 雙臂機器人描述
    dual_arm_description_content = Command([
        'xacro ', 
        PathJoinSubstitution([FindPackageShare('dual_arm_collaboration'), 'urdf', 'dual_arm_system.urdf.xacro']),
        ' ur5_x:=0 ur5_y:=-0.8 ur5_z:=0',
        ' arm0710_x:=0 arm0710_y:=0.8 arm0710_z:=0'
    ])
    
    dual_arm_description = {'robot_description': dual_arm_description_content}

    # 機器人狀態發布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[dual_arm_description]
    )

    # UR5 控制器啟動 (延遲啟動以避免衝突)
    ur5_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
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
                }.items()
            )
        ]
    )

    # ARM0710 控制器啟動
    arm0710_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='dual_arm_collaboration',
                executable='arm0710_controller.py',
                name='arm0710_controller',
                namespace='arm0710',
                output='screen',
                parameters=[
                    {'robot_ip': arm0710_robot_ip},
                    {'target_ip': '192.168.1.20'},
                    {'tf_prefix': 'arm0710_'},
                ]
            )
        ]
    )

    # MoveIt 配置
    moveit_config_package = 'dual_arm_collaboration'
    
    # SRDF 內容 (簡化版)
    dual_arm_srdf_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare(moveit_config_package), 'config', 'dual_arm.srdf.xacro'])
    ])
    
    robot_description_semantic = {'robot_description_semantic': dual_arm_srdf_content}

    # MoveIt 規劃器配置
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }

    # MoveIt 控制器配置
    moveit_controllers = {
        'moveit_simple_controller_manager': {
            'controller_names': ['ur5_scaled_joint_trajectory_controller', 'arm0710_joint_trajectory_controller'],
            'ur5_scaled_joint_trajectory_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': ['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint', 
                          'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint']
            },
            'arm0710_joint_trajectory_controller': {
                'type': 'FollowJointTrajectory', 
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': ['arm0710_joint1', 'arm0710_joint2', 'arm0710_joint3',
                          'arm0710_joint4', 'arm0710_joint5', 'arm0710_joint6']
            }
        },
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }

    # 軌跡執行參數
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # 規劃場景監視器
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # MoveIt move_group 節點 (延遲啟動)
    move_group_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                name='move_group',
                output='screen',
                parameters=[
                    dual_arm_description,
                    robot_description_semantic,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers,
                    planning_scene_monitor_parameters,
                ]
            )
        ]
    )

    # 雙臂協作控制器 (最後啟動)
    dual_arm_coordinator = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='dual_arm_collaboration',
                executable='dual_arm_coordinator.py',
                name='dual_arm_coordinator',
                output='screen',
                parameters=[
                    {'ur5_planning_group': 'ur5_manipulator'},
                    {'arm0710_planning_group': 'arm0710_manipulator'},
                    PathJoinSubstitution([FindPackageShare('dual_arm_collaboration'), 'config', 'safety_settings.yaml'])
                ]
            )
        ]
    )

    # RViz 配置
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('dual_arm_collaboration'),
        'rviz',
        'dual_arm_view.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(launch_rviz),
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            dual_arm_description,
            robot_description_semantic,
        ]
    )

    # 返回啟動描述
    nodes_to_start = [
        robot_state_publisher,
        ur5_launch,
        arm0710_controller,
        move_group_node,
        dual_arm_coordinator,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)