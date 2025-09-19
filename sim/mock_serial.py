#!/usr/bin/env python3
"""
Mock Serial Driver for EEG Robotic Arm Project
Week 2, Day 2

This module provides a MockSerial class that mimics the behavior of a real
Arduino serial connection for testing purposes. It simulates the pySerial
Serial class interface without requiring actual hardware.

Commands supported:
- L\n: Left joint movement
- R\n: Right joint movement  
- G\n: Grab/Gripper action
- RESET\n: Reset to home position
- STATUS\n: Get current status
"""

import time
import threading
from typing import Optional, List
from datetime import datetime
import logging


class MockSerial:
    """
    Mock serial class that simulates Arduino communication.
    
    Mimics the pySerial Serial class interface for testing without hardware.
    """
    
    def __init__(self, port: str = "MOCK_PORT", baudrate: int = 9600, timeout: float = 1.0):
        """
        Initialize the mock serial connection.
        
        Args:
            port: Mock port name (for compatibility)
            baudrate: Mock baudrate (for compatibility)
            timeout: Read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        
        # Internal state
        self._read_buffer = []
        self._command_history = []
        self._robot_state = {
            'position': 'home',
            'last_command': None,
            'command_count': 0,
            'joint_positions': [0, 0, 0, 0],  # 4 joints
            'gripper_state': 'open'
        }
        
        # Setup logging
        self._setup_logging()
        
        # Command mapping
        self._command_map = {
            b'L\n': self._handle_left_command,
            b'R\n': self._handle_right_command,
            b'G\n': self._handle_grip_command,
            b'RESET\n': self._handle_reset_command,
            b'STATUS\n': self._handle_status_command
        }
        
        self._logger.info(f"MockSerial initialized on {port} at {baudrate} baud")
    
    def _setup_logging(self):
        """Setup logging for mock serial communication."""
        self._logger = logging.getLogger('MockSerial')
        self._logger.setLevel(logging.INFO)
        
        # Create console handler if not already exists
        if not self._logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '[%(asctime)s] %(name)s - %(levelname)s: %(message)s',
                datefmt='%H:%M:%S'
            )
            handler.setFormatter(formatter)
            self._logger.addHandler(handler)
    
    def write(self, data: bytes) -> int:
        """
        Write data to the mock serial port.
        
        Args:
            data: Bytes to write (typically command + newline)
            
        Returns:
            Number of bytes written
        """
        if not self.is_open:
            raise Exception("Mock serial port is not open")
        
        # Log the command
        command_str = data.decode('utf-8', errors='ignore').strip()
        self._logger.info(f"TX: {repr(data)} -> '{command_str}'")
        
        # Store command in history
        self._command_history.append({
            'timestamp': datetime.now(),
            'command': data,
            'command_str': command_str
        })
        
        # Process the command and generate response
        if data in self._command_map:
            response = self._command_map[data]()
        else:
            response = self._handle_unknown_command(data)
        
        # Add response to read buffer
        self._read_buffer.append(response)
        
        # Update robot state
        self._robot_state['last_command'] = command_str
        self._robot_state['command_count'] += 1
        
        return len(data)
    
    def readline(self, timeout: Optional[float] = None) -> bytes:
        """
        Read a line from the mock serial port.
        
        Args:
            timeout: Read timeout (uses instance timeout if None)
            
        Returns:
            Response bytes ending with newline
        """
        if not self.is_open:
            raise Exception("Mock serial port is not open")
        
        # Use instance timeout if not specified
        if timeout is None:
            timeout = self.timeout
        
        # Wait for response with timeout
        start_time = time.time()
        while not self._read_buffer:
            if time.time() - start_time > timeout:
                self._logger.warning("Read timeout occurred")
                return b"[MOCK] TIMEOUT\n"
            time.sleep(0.01)  # Small delay to prevent busy waiting
        
        # Get response from buffer
        response = self._read_buffer.pop(0)
        
        # Log the response
        response_str = response.decode('utf-8', errors='ignore').strip()
        self._logger.info(f"RX: {repr(response)} -> '{response_str}'")
        
        return response
    
    def read(self, size: int = 1) -> bytes:
        """
        Read specified number of bytes.
        
        Args:
            size: Number of bytes to read
            
        Returns:
            Read bytes
        """
        # For simplicity, just return from readline and truncate
        data = self.readline()
        return data[:size]
    
    def close(self):
        """Close the mock serial connection."""
        self.is_open = False
        self._logger.info("Mock serial connection closed")
    
    def open(self):
        """Open the mock serial connection."""
        self.is_open = True
        self._logger.info("Mock serial connection opened")
    
    # Command Handlers
    
    def _handle_left_command(self) -> bytes:
        """Handle left joint movement command."""
        self._logger.info("🔄 Left joint moved")
        
        # Simulate joint movement (decrease angle)
        self._robot_state['joint_positions'][0] -= 10
        self._robot_state['position'] = 'left'
        
        return b"[MOCK] OK - Left joint moved\n"
    
    def _handle_right_command(self) -> bytes:
        """Handle right joint movement command."""
        self._logger.info("🔄 Right joint moved")
        
        # Simulate joint movement (increase angle)
        self._robot_state['joint_positions'][0] += 10
        self._robot_state['position'] = 'right'
        
        return b"[MOCK] OK - Right joint moved\n"
    
    def _handle_grip_command(self) -> bytes:
        """Handle gripper action command."""
        # Toggle gripper state
        if self._robot_state['gripper_state'] == 'open':
            self._robot_state['gripper_state'] = 'closed'
            action = "closed"
        else:
            self._robot_state['gripper_state'] = 'open'
            action = "opened"
        
        self._logger.info(f"🤖 Gripper {action}")
        
        return f"[MOCK] OK - Gripper {action}\n".encode()
    
    def _handle_reset_command(self) -> bytes:
        """Handle reset to home position command."""
        self._logger.info("🏠 Reset to home position")
        
        # Reset robot state
        self._robot_state['position'] = 'home'
        self._robot_state['joint_positions'] = [0, 0, 0, 0]
        self._robot_state['gripper_state'] = 'open'
        
        return b"[MOCK] OK - Reset to home\n"
    
    def _handle_status_command(self) -> bytes:
        """Handle status request command."""
        self._logger.info("📊 Status requested")
        
        # Build status response
        joints_str = ','.join(map(str, self._robot_state['joint_positions']))
        status = (f"[MOCK] STATUS - Pos:{self._robot_state['position']} "
                 f"Joints:[{joints_str}] Gripper:{self._robot_state['gripper_state']} "
                 f"Commands:{self._robot_state['command_count']}")
        
        return f"{status}\n".encode()
    
    def _handle_unknown_command(self, data: bytes) -> bytes:
        """Handle unknown command."""
        command_str = data.decode('utf-8', errors='ignore').strip()
        self._logger.warning(f"❌ Unknown command: '{command_str}'")
        
        return f"[MOCK] ERROR - Unknown command: {command_str}\n".encode()
    
    # Utility Methods
    
    def get_command_history(self) -> List[dict]:
        """Get command history for debugging."""
        return self._command_history.copy()
    
    def get_robot_state(self) -> dict:
        """Get current robot state."""
        return self._robot_state.copy()
    
    def clear_history(self):
        """Clear command history."""
        self._command_history.clear()
        self._logger.info("Command history cleared")
    
    def simulate_delay(self, delay_ms: int = 100):
        """Simulate Arduino processing delay."""
        time.sleep(delay_ms / 1000.0)
    
    # Properties for compatibility with pySerial
    
    @property
    def in_waiting(self) -> int:
        """Number of bytes waiting to be read."""
        return len(self._read_buffer)
    
    @property
    def out_waiting(self) -> int:
        """Number of bytes waiting to be written (always 0 for mock)."""
        return 0


def demo_mock_serial():
    """Demonstration of MockSerial functionality."""
    print("\n" + "="*60)
    print("MOCK SERIAL DRIVER DEMONSTRATION")
    print("="*60)
    
    # Create mock serial instance
    mock_ser = MockSerial(port="COM_MOCK", baudrate=115200)
    
    # Test commands
    test_commands = [
        b"L\n",
        b"R\n", 
        b"G\n",
        b"STATUS\n",
        b"RESET\n",
        b"INVALID\n"
    ]
    
    print("\nTesting commands:")
    for cmd in test_commands:
        print(f"\nSending: {repr(cmd)}")
        mock_ser.write(cmd)
        response = mock_ser.readline()
        print(f"Response: {response.decode().strip()}")
    
    # Show final state
    print(f"\nFinal robot state: {mock_ser.get_robot_state()}")
    print(f"Command history length: {len(mock_ser.get_command_history())}")
    
    # Close connection
    mock_ser.close()
    print("\nDemo completed!")


if __name__ == "__main__":
    # Run demonstration
    demo_mock_serial()

