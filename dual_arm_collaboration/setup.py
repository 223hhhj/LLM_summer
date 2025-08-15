# dual_arm_collaboration/setup.py (簡化版)
from setuptools import setup
import os
from glob import glob

package_name = 'dual_arm_collaboration'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', 
            glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', 
            glob('urdf/*.urdf.xacro')),
        ('share/' + package_name + '/rviz', 
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your.email@example.com',
    description='Dual arm collaboration system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm0710_controller = dual_arm_collaboration.arm0710_controller:main',
            'dual_arm_coordinator = dual_arm_collaboration.dual_arm_coordinator:main',
        ],
    },
)