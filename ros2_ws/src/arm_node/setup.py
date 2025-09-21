from setuptools import setup

package_name = 'arm_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='test@example.com',
    description='ROS2 bridge for EEG robotic arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joint_state_publisher = arm_node.joint_state_publisher:main',
        ],
    },
)
