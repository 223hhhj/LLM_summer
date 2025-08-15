from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera1',
            namespace='cam1',
            parameters=[{'serial_no': '844212070157'}]  # 替換為你的第一台相機序列號
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera2',
            namespace='cam2',
            parameters=[{'serial_no': '140122074462'}]  # 替換為你的第二台相機序列號
        )
    ])
