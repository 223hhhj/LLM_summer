from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotiq3f_ros2',
            executable='robotiq_gripper_node',
            name='robotiq_gripper_node',
            output='screen',
            parameters=[
                {'gripper_ip': '192.168.1.11'},
                {'update_interval': 0.1}
            ]
        )
    ])