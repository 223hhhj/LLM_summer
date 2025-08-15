#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # 聲明參數
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for the arm links and joints",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Start joint_state_publisher_gui",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="display.rviz",
            description="RViz config file name",
        )
    )
    
    # 設定參數
    prefix = LaunchConfiguration("prefix")
    use_gui = LaunchConfiguration("use_gui")
    rviz_config = LaunchConfiguration("rviz_config")
    
    # Robot description
    robot_description_content = ParameterValue(
        Command([
            "xacro ", 
            PathJoinSubstitution([
                FindPackageShare("arm0717_description"), 
                "urdf", 
                "arm0717.urdf.xacro"
            ]),
            " prefix:=", prefix
        ]),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Joint state publisher (without GUI)
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=UnlessCondition(use_gui),
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_gui),
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("arm0717_description"), "rviz", rviz_config
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
    )
    
    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)