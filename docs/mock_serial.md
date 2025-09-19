
Mock Serial Driver Documentation
Overview
The MockSerial class simulates an Arduino serial connection for the EEG Robotic Arm project. It provides a drop-in replacement for pySerial's Serial class, allowing development and testing without physical hardware.
Purpose
Hardware-independent development: Test serial communication logic without Arduino
Consistent responses: Predictable behavior for automated testing
Command logging: Track all commands sent and responses received
State simulation: Maintain virtual robot state for realistic testing
File Structure
sim/
├── mock_serial.py         # Main MockSerial class implementation
├── test_mock_serial.py    # Test suite for verification
└── ...

MockSerial Class
Constructor
MockSerial(port="MOCK_PORT", baudrate=9600, timeout=1.0)

Parameters:
port: Mock port name (compatibility parameter)
baudrate: Mock baudrate (compatibility parameter)
timeout: Read timeout in seconds
Core Methods
write(data: bytes) -> int
Sends command to the mock Arduino.
ser = MockSerial()
bytes_written = ser.write(b"L\n")  # Returns number of bytes written

readline(timeout: Optional[float] = None) -> bytes
Reads response from mock Arduino.
response = ser.readline()  # Returns b"[MOCK] OK - Left joint moved\n"

close() / open()
Close or open the mock connection.
ser.close()  # Close connection
ser.open()   # Reopen connection

Supported Commands
Command
Description
Response
Robot State Change
L\n
Move left joint
[MOCK] OK - Left joint moved\n
Decreases joint 0 angle by 10°
R\n
Move right joint
[MOCK] OK - Right joint moved\n
Increases joint 0 angle by 10°
G\n
Toggle gripper
[MOCK] OK - Gripper [opened/closed]\n
Toggles gripper state
RESET\n
Reset to home
[MOCK] OK - Reset to home\n
All joints to 0°, gripper open
STATUS\n
Get status
[MOCK] STATUS - Pos:... Joints:[...] ...
No change
Invalid
Unknown command
[MOCK] ERROR - Unknown command: ...\n
No change

Robot State
The mock maintains internal state:
{
    'position': 'home',           # Current position description
    'last_command': 'L',          # Last command received
    'command_count': 5,           # Total commands processed
    'joint_positions': [10, 0, 0, 0],  # 4 joint angles
    'gripper_state': 'open'       # Gripper state
}

Usage Examples
Basic Usage (Day 2 Requirement)
from sim.mock_serial import MockSerial

ser = MockSerial()
ser.write(b"L\n")
print(ser.readline())  # Prints: b"[MOCK] OK - Left joint moved\n"
ser.close()

Extended Usage
from sim.mock_serial import MockSerial

# Create connection
ser = MockSerial(port="COM_MOCK", baudrate=115200)

# Send command sequence
commands = [b"L\n", b"R\n", b"G\n", b"STATUS\n", b"RESET\n"]

for cmd in commands:
    ser.write(cmd)
    response = ser.readline()
    print(f"Command: {cmd.decode().strip()}")
    print(f"Response: {response.decode().strip()}")
    print()

# Get state information
state = ser.get_robot_state()
history = ser.get_command_history()

print(f"Final state: {state}")
print(f"Commands sent: {len(history)}")

ser.close()

Integration with Real pySerial Code
The MockSerial class is designed to be a drop-in replacement:
# Production code
if USE_REAL_HARDWARE:
    import serial
    ser = serial.Serial('COM3', 115200)
else:
    from sim.mock_serial import MockSerial
    ser = MockSerial()

# Same code works with both
ser.write(b"L\n")
response = ser.readline()
ser.close()

Logging
The MockSerial class provides detailed logging:
[14:32:15] MockSerial - INFO: MockSerial initialized on MOCK_PORT at 9600 baud
[14:32:15] MockSerial - INFO: TX: b'L\n' -> 'L'
[14:32:15] MockSerial - INFO: 🔄 Left joint moved
[14:32:15] MockSerial - INFO: RX: b'[MOCK] OK - Left joint moved\n' -> '[MOCK] OK - Left joint moved'

Testing
Run the test suite:
cd ~/eeg-robotic-arm/sim
python3 test_mock_serial.py

The test suite includes:
Basic functionality: Core write/readline operations
Extended functionality: All supported commands
Error handling: Timeouts, closed connections, invalid commands
Performance: Rapid command processing
Error Handling
Timeout Behavior
If no response is available within the timeout period:
response = ser.readline(timeout=0.1)  # Returns b"[MOCK] TIMEOUT\n"

Closed Connection
Writing to a closed connection raises an exception:
ser.close()
ser.write(b"L\n")  # Raises: Exception("Mock serial port is not open")

Invalid Commands
Unknown commands return error responses:
ser.write(b"INVALID\n")
response = ser.readline()  # b"[MOCK] ERROR - Unknown command: INVALID\n"

Implementation Notes
Thread Safety
The current implementation is not thread-safe. For multi-threaded usage, consider adding locks around buffer operations.
Memory Usage
Command history is stored indefinitely. Use clear_history() for long-running applications:
ser.clear_history()  # Clear command history to save memory

Timing Simulation
Add realistic delays to simulate Arduino processing:
ser.simulate_delay(150)  # Simulate 150ms processing delay

Compatibility
pySerial Compatibility
The MockSerial class implements these pySerial Serial class methods:
write(data)
readline(timeout)
read(size)
close()
open()
Properties: in_waiting, out_waiting
Python Version
Minimum: Python 3.7+
Recommended: Python 3.10+
Dependencies: Standard library only
Future Enhancements
Potential improvements for future versions:
Thread safety: Add locks for concurrent access
Configuration file: Load commands/responses from YAML/JSON
Network mode: Simulate TCP/UDP communication
Advanced timing: Variable response delays per command
State persistence: Save/load robot state to file
Troubleshooting
Common Issues
Import Error:
# Error: ImportError: cannot import name 'MockSerial'
# Solution: Check that mock_serial.py is in the same directory
import sys
sys.path.append('/path/to/sim')
from mock_serial import MockSerial

No Response:
# Issue: readline() returns timeout
# Solution: Make sure write() was called first
ser.write(b"L\n")  # Must write before reading
response = ser.readline()

Unexpected Response:
# Issue: Getting wrong response format
# Solution: Commands must end with \n
ser.write(b"L\n")  # Correct - with newline
ser.write(b"L")    # Wrong - missing newline

Version History
v1.0: Initial implementation with basic L/R/G commands
v1.1: Added STATUS and RESET commands
v1.2: Added state tracking and command history
v1.3: Enhanced logging and error handling
License
This code is part of the EEG Robotic Arm project and follows the same license terms as the main project.


