from setuptools import setup

package_name = 'my_robot_socket_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 這裡要是資料夾名稱
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steven',
    maintainer_email='steven@example.com',
    description='Socket-based robot joint reader',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_reader = my_robot_socket_node.joint_reader_node:main',
        ],
    },
)
