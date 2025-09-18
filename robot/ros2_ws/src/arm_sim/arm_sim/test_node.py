#!/usr/bin/env python3
"""ROS2 test node for simulation verification"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np

class SimTestNode(Node):
    def __init__(self):
        super().__init__('sim_test_node')
        
        # Simple publisher for testing
        self.publisher_ = self.create_publisher(String, 'test_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0
        
        self.get_logger().info('Simulation test node started!')
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Test message {self.counter}: Joint angles = {np.random.randn(4).tolist()}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = SimTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
