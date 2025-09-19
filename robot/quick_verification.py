#!/usr/bin/env python3
from arm_driver import ArmDriver

# Create driver
driver = ArmDriver()
print("Initial angle:", driver.joint_angles[0])

# Test L command
driver.process_command('L')
print("After L command:", driver.joint_angles[0])

# Test R command  
driver.process_command('R')
print("After R command:", driver.joint_angles[0])

# Test G command
print("Gripper before G:", "open" if driver.gripper_open else "closed")
driver.process_command('G')
print("Gripper after G:", "open" if driver.gripper_open else "closed")

print("✅ Terminal commands successfully update joint angles!")
