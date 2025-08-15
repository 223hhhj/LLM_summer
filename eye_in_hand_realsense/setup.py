from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'eye_in_hand_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steven',
    maintainer_email='steven@todo.todo',
    description='YOLO Realsense Integration',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_eye_in_hand = eye_in_hand_realsense.yolo_eye_in_hand:main',
        ],
    },
)
