#!/usr/bin/env python3
"""
Launch file for EEG Robotic Arm ROS2 Bridge
Week 3, Day 3
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Joint state publishing rate in Hz'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='base_link',
            description='Frame ID for joint states'
        ),
        
        # Log info
        LogInfo(msg="🚀 Launching EEG Robotic Arm ROS2 Bridge..."),
        
        # Joint State Publisher Node
        Node(
            package='arm_node',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'joint_names': ['joint1', 'joint2', 'joint3', 'gripper']
            }],
            remappings=[
                ('/joint_states', '/joint_states')
            ]
        ),
        
        # Arm Bridge Node
        Node(
            package='arm_node',
            executable='arm_bridge',
            name='arm_bridge',
            output='screen'
        ),
        
        LogInfo(msg="✅ EEG Robotic Arm ROS2 Bridge launched!")
    ])
