
from setuptools import setup

package_name = 'arm_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EEG Robotic Arm Team',
    maintainer_email='your.email@example.com',
    description='ROS2 bridge node for EEG robotic arm simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_bridge = arm_node.arm_bridge:main',
            'joint_state_publisher = arm_node.joint_state_publisher:main',
        ],
    },
)


