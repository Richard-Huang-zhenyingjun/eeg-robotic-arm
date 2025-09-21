# Find the command_callback method and replace it with this:

def command_callback(self, msg: String):
    """Handle incoming command messages."""
    command = msg.data.strip().upper()
    
    self.get_logger().info(f"Received command: {command}")
    
    if self.arm_driver:
        # Process command DIRECTLY through arm driver (not keyboard control)
        success = self.arm_driver.process_command(command)
        
        if success:
            self.get_logger().info(f"Command {command} executed")
            
            # Log command directly (not through keyboard control)
            if self.logger_hook:
                self.logger_hook.log_command(
                    command,
                    success=True,
                    metadata={
                        "source": "ros2_topic",
                        "joint_angles": self.arm_driver.joint_angles.copy(),
                        "gripper_open": self.arm_driver.gripper_open
                    }
                )
        else:
            self.get_logger().warning(f"Command {command} failed")
    else:
        self.get_logger().warning(f"No arm driver - command {command} ignored")
