from setuptools import setup

package_name = 'robotiq3f_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robotiq_gripper_launch.py']),
        ('share/' + package_name + '/config', ['config/gripper_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven',
    maintainer_email='mike355116@gmail.com',
    description='ROS2 package for Robotiq 3-finger gripper control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotiq_gripper_node = robotiq3f_ros2.robotiq_gripper_node:main',
            'gripper_status_viewer = robotiq3f_ros2.gripper_status_viewer:main',
            'gripper_test_commander = robotiq3f_ros2.gripper_test_commander:main',
        ],
    },
)