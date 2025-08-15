#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    # 宣告參數
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Start joint_state_publisher_gui",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="arm0718",
            description="Name of the robot",
        )
    )
    
    # 設定參數
    use_gui = LaunchConfiguration("use_gui")
    robot_name = LaunchConfiguration("robot_name")
    
    # 取得URDF檔案路徑
    pkg_path = get_package_share_directory('arm0718_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'arm0718.urdf')
    
    # 檢查檔案是否存在
    if not os.path.exists(urdf_file):
        print(f"Error: URDF file not found at {urdf_file}")
        print("Please make sure the file exists in the correct location.")
        print("Expected file structure:")
        print("  ~/ros2_ws/src/arm0718_description/")
        print("  ├── urdf/")
        print("  │   └── arm0718.urdf")
        print("  ├── launch/")
        print("  │   └── display.launch.py")
        print("  ├── meshes/")
        print("  │   ├── base_link.STL")
        print("  │   ├── link_2.STL")
        print("  │   ├── link_3.STL")
        print("  │   ├── link_4.STL")
        print("  │   ├── link_5.STL")
        print("  │   ├── link_6.STL")
        print("  │   └── link_7.STL")
        print("  └── package.xml")
        return LaunchDescription([])
    
    # 讀取URDF檔案內容
    try:
        with open(urdf_file, 'r', encoding='utf-8') as file:
            robot_description_content = file.read()
    except Exception as e:
        print(f"Error reading URDF file: {e}")
        return LaunchDescription([])
    
    print(f"Successfully loaded URDF from: {urdf_file}")
    print("Starting 7-DOF ARM0718 robot visualization...")
    
    # Robot State Publisher節點
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
            'frame_prefix': ''
        }]
    )
    
    # Joint State Publisher節點 (without GUI)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(use_gui),
        parameters=[{
            'use_sim_time': False,
            'zeros': {
                'joint2': 0.0,
                'joint3': 0.0,
                'joint4': 0.0,
                'joint5': 0.0,
                'joint6': 0.0,
                'joint7': 0.0
            }
        }]
    )
    
    # Joint State Publisher GUI節點
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    # RViz節點
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'display.rviz')
    rviz_args = []
    if os.path.exists(rviz_config_file):
        rviz_args = ['-d', rviz_config_file]
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args,
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)