#!/usr/bin/env python3
"""PyBullet simulation node for 4-DOF arm with ROS2 interface"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pybullet as p
import pybullet_data
import numpy as np
import threading
import time

class PyBulletArmSim(Node):
    def __init__(self):
        super().__init__('pybullet_arm_sim')
        
        # ROS2 publishers and subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Initialize PyBullet
        self.physics_client = p.connect(p.DIRECT)  # Use DIRECT for headless
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load environment
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Load robot
        robot_path = "/workspace/robot/urdf/simple_arm.urdf"
        self.robot_id = p.loadURDF(robot_path, [0, 0, 0], useFixedBase=True)
        
        # Get joint info
        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = list(range(self.num_joints))
        self.joint_names = [f"joint{i+1}" for i in range(self.num_joints)]
        
        # Target positions
        self.target_positions = [0.0] * self.num_joints
        
        # Start simulation thread
        self.sim_thread = threading.Thread(target=self.simulation_loop)
        self.sim_thread.daemon = True
        self.running = True
        self.sim_thread.start()
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz
        
        self.get_logger().info(f'PyBullet arm simulation started with {self.num_joints} joints')
    
    def joint_command_callback(self, msg):
        """Handle incoming joint commands"""
        if len(msg.data) <= self.num_joints:
            for i, pos in enumerate(msg.data):
                self.target_positions[i] = pos
            self.get_logger().info(f'Received command: {msg.data}')
    
    def simulation_loop(self):
        """Run PyBullet simulation"""
        while self.running:
            # Set joint positions
            for i in range(self.num_joints):
                p.setJointMotorControl2(
                    self.robot_id,
                    i,
                    p.POSITION_CONTROL,
                    targetPosition=self.target_positions[i]
                )
            
            p.stepSimulation()
            time.sleep(1/240.0)  # 240 Hz
    
    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        positions = []
        velocities = []
        
        for i in range(self.num_joints):
            state = p.getJointState(self.robot_id, i)
            positions.append(state[0])
            velocities.append(state[1])
        
        joint_state.position = positions
        joint_state.velocity = velocities
        
        self.joint_pub.publish(joint_state)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.running = False
        if self.sim_thread.is_alive():
            self.sim_thread.join(timeout=1.0)
        p.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PyBulletArmSim()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
