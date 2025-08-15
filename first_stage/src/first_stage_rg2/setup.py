from setuptools import find_packages, setup

package_name = 'first_stage_rg2'

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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rg2_node = first_stage_rg2.first_stage_rg2_ros2:main",
            'gripper_server = first_stage_rg2.gripper_control_client:main',
            'gripper_tester = first_stage_rg2.gripper_test:main',
        ],
    },
)
