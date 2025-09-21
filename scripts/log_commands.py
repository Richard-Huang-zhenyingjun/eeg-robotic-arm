
#!/usr/bin/env python3
"""
Command Logging System
Week 3, Day 2 - EEG Robotic Arm Project

Logs all keyboard commands with timestamps to daily log files.
Integrates with keyboard_control.py to automatically save command history.
"""

import os
import sys
from datetime import datetime
from typing import Optional, List, Dict, Any
from pathlib import Path
import threading
import json


class CommandLogger:
    """
    Command logging system that saves all commands with timestamps.
    
    Creates daily log files in format: commands_YYYYMMDD.txt
    Each line: [YYYY-MM-DD HH:MM:SS] COMMAND
    """
    
    def __init__(self, log_dir: str = None):
        """
        Initialize command logger.
        
        Args:
            log_dir: Directory to save log files (default: data/logs)
        """
        # Determine project root and log directory
        if log_dir is None:
            # Get project root (assuming this script is in scripts/)
            script_dir = Path(__file__).parent
            project_root = script_dir.parent
            self.log_dir = project_root / "data" / "logs"
        else:
            self.log_dir = Path(log_dir)
        
        # Create log directory if it doesn't exist
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Session tracking
        self.session_start = datetime.now()
        self.session_id = self.session_start.strftime("%Y%m%d_%H%M%S")
        self.commands_logged = 0
        
        # Thread safety
        self._lock = threading.Lock()
        
        print(f"📝 CommandLogger initialized")
        print(f"   Log directory: {self.log_dir}")
        print(f"   Session ID: {self.session_id}")
    
    def get_daily_log_filename(self, date: datetime = None) -> Path:
        """
        Get the filename for daily command log.
        
        Args:
            date: Date for log file (default: today)
            
        Returns:
            Path to daily log file
        """
        if date is None:
            date = datetime.now()
        
        filename = f"commands_{date.strftime('%Y%m%d')}.txt"
        return self.log_dir / filename
    
    def log_command(self, command: str, timestamp: datetime = None, metadata: Dict[str, Any] = None):
        """
        Log a single command with timestamp.
        
        Args:
            command: Command string (e.g., 'L', 'R', 'U', 'D', 'G')
            timestamp: Command timestamp (default: now)
            metadata: Additional metadata (optional)
        """
        if timestamp is None:
            timestamp = datetime.now()
        
        # Thread-safe logging
        with self._lock:
            try:
                # Get daily log file
                log_file = self.get_daily_log_filename(timestamp)
                
                # Format log entry
                timestamp_str = timestamp.strftime("%Y-%m-%d %H:%M:%S")
                log_entry = f"[{timestamp_str}] {command}"
                
                # Add metadata if provided
                if metadata:
                    metadata_str = json.dumps(metadata, separators=(',', ':'))
                    log_entry += f" {metadata_str}"
                
                # Append to log file
                with open(log_file, 'a', encoding='utf-8') as f:
                    f.write(log_entry + '\n')
                
                self.commands_logged += 1
                
                # Print confirmation (optional, can be disabled)
                print(f"📝 Logged: {log_entry}")
                
            except Exception as e:
                print(f"❌ Error logging command: {e}")
    
    def log_session_start(self):
        """Log session start marker."""
        self.log_command(
            "SESSION_START",
            metadata={
                "session_id": self.session_id,
                "session_start": self.session_start.isoformat()
            }
        )
    
    def log_session_end(self):
        """Log session end marker."""
        session_end = datetime.now()
        duration = (session_end - self.session_start).total_seconds()
        
        self.log_command(
            "SESSION_END",
            metadata={
                "session_id": self.session_id,
                "session_end": session_end.isoformat(),
                "duration_seconds": duration,
                "commands_logged": self.commands_logged
            }
        )
    
    def get_session_summary(self) -> Dict[str, Any]:
        """Get summary of current logging session."""
        current_time = datetime.now()
        duration = (current_time - self.session_start).total_seconds()
        
        return {
            "session_id": self.session_id,
            "session_start": self.session_start.isoformat(),
            "current_time": current_time.isoformat(),
            "duration_seconds": duration,
            "commands_logged": self.commands_logged,
            "log_directory": str(self.log_dir),
            "current_log_file": str(self.get_daily_log_filename())
        }
    
    def read_daily_log(self, date: datetime = None) -> List[Dict[str, Any]]:
        """
        Read and parse daily log file.
        
        Args:
            date: Date to read (default: today)
            
        Returns:
            List of parsed log entries
        """
        log_file = self.get_daily_log_filename(date)
        
        if not log_file.exists():
            return []
        
        entries = []
        try:
            with open(log_file, 'r', encoding='utf-8') as f:
                for line_num, line in enumerate(f, 1):
                    line = line.strip()
                    if not line:
                        continue
                    
                    try:
                        # Parse timestamp and command
                        if line.startswith('[') and ']' in line:
                            timestamp_end = line.index(']')
                            timestamp_str = line[1:timestamp_end]
                            rest = line[timestamp_end + 1:].strip()
                            
                            # Split command and metadata
                            parts = rest.split(' ', 1)
                            command = parts[0]
                            metadata = None
                            
                            if len(parts) > 1 and parts[1].startswith('{'):
                                try:
                                    metadata = json.loads(parts[1])
                                except json.JSONDecodeError:
                                    pass
                            
                            # Parse timestamp
                            timestamp = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S")
                            
                            entries.append({
                                "timestamp": timestamp,
                                "command": command,
                                "metadata": metadata,
                                "line_number": line_num,
                                "raw_line": line
                            })
                        
                    except Exception as e:
                        print(f"⚠️  Error parsing line {line_num}: {e}")
                        entries.append({
                            "timestamp": None,
                            "command": "PARSE_ERROR",
                            "metadata": {"error": str(e)},
                            "line_number": line_num,
                            "raw_line": line
                        })
        
        except Exception as e:
            print(f"❌ Error reading log file {log_file}: {e}")
        
        return entries
    
    def get_log_statistics(self, date: datetime = None) -> Dict[str, Any]:
        """
        Get statistics for a daily log.
        
        Args:
            date: Date to analyze (default: today)
            
        Returns:
            Dictionary with log statistics
        """
        entries = self.read_daily_log(date)
        
        if not entries:
            return {"total_entries": 0, "commands": {}}
        
        # Count commands
        command_counts = {}
        valid_entries = 0
        
        for entry in entries:
            if entry["command"] and entry["command"] != "PARSE_ERROR":
                command = entry["command"]
                command_counts[command] = command_counts.get(command, 0) + 1
                valid_entries += 1
        
        # Calculate time span
        valid_timestamps = [e["timestamp"] for e in entries if e["timestamp"]]
        time_span = None
        if valid_timestamps:
            time_span = {
                "start": min(valid_timestamps).isoformat(),
                "end": max(valid_timestamps).isoformat(),
                "duration_seconds": (max(valid_timestamps) - min(valid_timestamps)).total_seconds()
            }
        
        return {
            "date": date.strftime("%Y-%m-%d") if date else datetime.now().strftime("%Y-%m-%d"),
            "total_entries": len(entries),
            "valid_entries": valid_entries,
            "parse_errors": len(entries) - valid_entries,
            "command_counts": command_counts,
            "time_span": time_span,
            "log_file": str(self.get_daily_log_filename(date))
        }


class CommandLoggerHook:
    """
    Hook for integrating CommandLogger with keyboard control systems.
    
    Provides easy integration with existing command processing systems.
    """
    
    def __init__(self, logger: CommandLogger = None):
        """
        Initialize logger hook.
        
        Args:
            logger: CommandLogger instance (creates new if None)
        """
        self.logger = logger or CommandLogger()
        self.enabled = True
        
        # Start session logging
        self.logger.log_session_start()
        
        print("🔗 CommandLoggerHook ready")
    
    def log_command(self, command: str, success: bool = True, metadata: Dict[str, Any] = None):
        """
        Log a command through the hook.
        
        Args:
            command: Command string
            success: Whether command was successful
            metadata: Additional metadata
        """
        if not self.enabled:
            return
        
        # Enhance metadata with success status
        enhanced_metadata = {"success": success}
        if metadata:
            enhanced_metadata.update(metadata)
        
        self.logger.log_command(command, metadata=enhanced_metadata)
    
    def enable_logging(self):
        """Enable command logging."""
        self.enabled = True
        self.logger.log_command("LOGGING_ENABLED")
    
    def disable_logging(self):
        """Disable command logging."""
        self.logger.log_command("LOGGING_DISABLED")
        self.enabled = False
    
    def end_session(self):
        """End the logging session."""
        self.logger.log_session_end()
        print("📝 Logging session ended")
    
    def get_summary(self) -> Dict[str, Any]:
        """Get session summary."""
        return self.logger.get_session_summary()


def test_command_logger():
    """Test the command logger functionality."""
    print("🧪 Testing Command Logger...")
    
    # Create logger
    logger = CommandLogger()
    
    # Test logging some commands
    test_commands = [
        ("L", {"joint": "base", "direction": "left"}),
        ("R", {"joint": "base", "direction": "right"}),
        ("U", {"joint": "shoulder", "direction": "up"}),
        ("D", {"joint": "shoulder", "direction": "down"}),
        ("G", {"action": "toggle_gripper"})
    ]
    
    print("\n📝 Logging test commands...")
    for command, metadata in test_commands:
        logger.log_command(command, metadata=metadata)
    
    # Test session management
    print("\n📊 Session Summary:")
    summary = logger.get_session_summary()
    for key, value in summary.items():
        print(f"   {key}: {value}")
    
    # Test reading log
    print("\n📖 Reading today's log:")
    entries = logger.read_daily_log()
    for entry in entries[-5:]:  # Show last 5 entries
        print(f"   {entry['timestamp']} - {entry['command']}")
    
    # Test statistics
    print("\n📈 Log Statistics:")
    stats = logger.get_log_statistics()
    for key, value in stats.items():
        print(f"   {key}: {value}")
    
    # End session
    logger.log_session_end()
    
    print("\n✅ Command logger test completed!")


def demo_logger_hook():
    """Demonstrate logger hook functionality."""
    print("🧪 Testing Logger Hook...")
    
    # Create hook
    hook = CommandLoggerHook()
    
    # Simulate some commands
    demo_commands = [
        ("L", True),
        ("L", True),
        ("R", True),
        ("U", True),
        ("G", True),
        ("INVALID", False)
    ]
    
    print("\n🔗 Logging commands through hook...")
    for command, success in demo_commands:
        hook.log_command(command, success=success)
    
    # Show summary
    print("\n📊 Hook Summary:")
    summary = hook.get_summary()
    for key, value in summary.items():
        print(f"   {key}: {value}")
    
    # End session
    hook.end_session()
    
    print("\n✅ Logger hook test completed!")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == 'test':
            test_command_logger()
        elif sys.argv[1] == 'hook':
            demo_logger_hook()
        elif sys.argv[1] == 'stats':
            # Show today's statistics
            logger = CommandLogger()
            stats = logger.get_log_statistics()
            print("📈 Today's Command Log Statistics:")
            print(json.dumps(stats, indent=2, default=str))
    else:
        print("Command Logger Module")
        print("Usage:")
        print("  python log_commands.py test    # Test logger")
        print("  python log_commands.py hook    # Test hook")
        print("  python log_commands.py stats   # Show today's stats")


