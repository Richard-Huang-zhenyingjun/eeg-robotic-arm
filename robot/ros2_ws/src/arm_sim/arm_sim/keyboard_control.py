#!/usr/bin/env python3
"""Keyboard control for the 4-DOF arm"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.positions = [0.0, 0.0, 0.0, 0.0]
        self.step = 0.1
        
        print('Keyboard Control for 4-DOF Arm')
        print('--------------------------------')
        print('Controls:')
        print('  q/a: Joint 1 +/-')
        print('  w/s: Joint 2 +/-') 
        print('  e/d: Joint 3 +/-')
        print('  r/f: Joint 4 +/-')
        print('  Space: Reset all joints to zero')
        print('  ESC: Quit')
        print('--------------------------------')
        
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x1b':  # ESC
                    break
                elif key == 'q': 
                    self.positions[0] = min(3.14, self.positions[0] + self.step)
                elif key == 'a': 
                    self.positions[0] = max(-3.14, self.positions[0] - self.step)
                elif key == 'w': 
                    self.positions[1] = min(1.57, self.positions[1] + self.step)
                elif key == 's': 
                    self.positions[1] = max(-1.57, self.positions[1] - self.step)
                elif key == 'e': 
                    self.positions[2] = min(2.36, self.positions[2] + self.step)
                elif key == 'd': 
                    self.positions[2] = max(-2.36, self.positions[2] - self.step)
                elif key == 'r': 
                    self.positions[3] = min(1.57, self.positions[3] + self.step)
                elif key == 'f': 
                    self.positions[3] = max(-1.57, self.positions[3] - self.step)
                elif key == ' ':  # Space bar
                    self.positions = [0.0, 0.0, 0.0, 0.0]
                    print('Reset all joints to zero')
                
                if key and key != '\x1b':
                    msg = Float64MultiArray()
                    msg.data = self.positions
                    self.publisher.publish(msg)
                    print(f'Joint positions: [{self.positions[0]:.2f}, {self.positions[1]:.2f}, {self.positions[2]:.2f}, {self.positions[3]:.2f}]')
                    
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print('\nKeyboard control terminated')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    try:
        node.run()
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
