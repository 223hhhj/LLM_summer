from setuptools import setup
import os
from glob import glob

package_name = 'yolo_pf_realsense'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.scripts'],  # 包含 scripts 模組
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steven',
    maintainer_email='steven@todo.todo',
    description='Dual RealSense camera object detection',
    license='TODO: License declaration',
    # 移除了 tests_require=['pytest'] 這行
    entry_points={
        'console_scripts': [
            'realsense_yolo_pf = yolo_pf_realsense.scripts.realsense_yolo_pf:main',
        ],
    },
)