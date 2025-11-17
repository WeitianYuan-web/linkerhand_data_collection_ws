#!/usr/bin/env python3
"""
Time synchronization utilities for ROS1 and ROS2 systems.
Provides unified interface for timestamp handling and validation.
"""

import time
import numpy as np
from typing import Optional, Dict, Any, Tuple

# Try to import ROS2 first, then ROS1 as fallback
try:
    import rclpy
    from rclpy.time import Time as ROS2Time
    from rclpy.clock import Clock, ClockType
    ROS_VERSION = 2
    print("Using ROS2 time synchronization")
except ImportError:
    try:
        import rospy
        ROS_VERSION = 1
        print("Using ROS1 time synchronization")
    except ImportError:
        ROS_VERSION = 0
        print("No ROS detected, using system time")

class TimeSyncManager:
    """Manages time synchronization across different sensors and ROS versions."""
    
    def __init__(self, max_timestamp_diff: float = 0.1, use_system_time: bool = False):
        """
        Initialize time synchronization manager.
        
        Args:
            max_timestamp_diff: Maximum allowed difference between timestamps (seconds)
            use_system_time: Force using system time instead of ROS time
                            True: Use system time (for tasks with non-ROS sensors like Piper SDK)
                            False: Use ROS time if available (for pure ROS2 tasks)
        """
        self.max_timestamp_diff = max_timestamp_diff
        self.use_system_time = use_system_time
        self.clock = None
        self.time_source = "unknown"
        self.ros2_to_system_offset = None  # Offset to convert ROS2 time to system time
        
        # Determine which time source to use
        if use_system_time:
            self.time_source = "system"
            print(f"TimeSyncManager: Using system time (forced)")
            # Calculate offset between ROS2 time and system time for conversion
            if ROS_VERSION == 2:
                try:
                    self.clock = Clock(clock_type=ClockType.ROS_TIME)
                    # Calculate offset once at initialization
                    current_system_time = time.time()
                    current_ros2_time = self.clock.now().nanoseconds / 1e9
                    self.ros2_to_system_offset = current_system_time - current_ros2_time
                    print(f"TimeSyncManager: ROS2 to system time offset: {self.ros2_to_system_offset:.6f}s")
                except Exception as e:
                    print(f"Warning: Could not initialize ROS2 clock for time conversion: {e}")
                    self.clock = None
        elif ROS_VERSION == 2:
            self.clock = Clock(clock_type=ClockType.ROS_TIME)
            self.time_source = "ros2"
            print(f"TimeSyncManager: Using ROS2 time")
        elif ROS_VERSION == 1:
            # ROS1 clock will be initialized when rospy is ready
            self.time_source = "ros1"
            print(f"TimeSyncManager: Using ROS1 time")
        else:
            self.time_source = "system"
            print(f"TimeSyncManager: Using system time (no ROS detected)")
    
    def get_current_time(self) -> float:
        """Get current synchronized time in seconds."""
        # If forced to use system time, always return system time
        if self.use_system_time:
            return time.time()
        
        # Otherwise use ROS time if available
        if ROS_VERSION == 2 and self.clock is not None:
            return self.clock.now().nanoseconds / 1e9
        elif ROS_VERSION == 1:
            return rospy.Time.now().to_sec()
        else:
            return time.time()
    
    def get_timestamp_from_msg(self, msg) -> Optional[float]:
        """
        Extract timestamp from ROS message.
        If use_system_time is True, converts ROS2 time to system time.
        """
        try:
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                if ROS_VERSION == 2:
                    ros2_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                    
                    # If forced to use system time, convert ROS2 time to system time
                    if self.use_system_time:
                        # Use pre-calculated offset if available
                        if self.ros2_to_system_offset is not None:
                            system_timestamp = ros2_timestamp + self.ros2_to_system_offset
                            return system_timestamp
                        else:
                            # Fallback: use current system time if offset not available
                            return time.time()
                    
                    return ros2_timestamp
                else:
                    return msg.header.stamp.to_sec()
        except Exception as e:
            print(f"Error extracting timestamp: {e}")
        return None
    
    def validate_timestamps(self, timestamps: Dict[str, float]) -> Tuple[bool, Dict[str, Any]]:
        """
        Validate that all timestamps are within acceptable bounds.
        
        Args:
            timestamps: Dictionary of sensor_name -> timestamp pairs
            
        Returns:
            Tuple of (is_valid, validation_info)
        """
        if not timestamps:
            return False, {"error": "No timestamps provided"}
        
        current_time = self.get_current_time()
        timestamp_values = list(timestamps.values())
        
        # Check if all timestamps are recent
        max_age = 1.0  # 1 second
        for sensor, ts in timestamps.items():
            if current_time - ts > max_age:
                return False, {
                    "error": f"Timestamp too old for {sensor}",
                    "age": current_time - ts,
                    "max_age": max_age
                }
        
        # Check if timestamps are close to each other
        if len(timestamp_values) > 1:
            max_diff = max(timestamp_values) - min(timestamp_values)
            if max_diff > self.max_timestamp_diff:
                return False, {
                    "error": "Timestamps too far apart",
                    "max_diff": max_diff,
                    "max_allowed": self.max_timestamp_diff,
                    "timestamps": timestamps
                }
        
        return True, {
            "max_diff": max(timestamp_values) - min(timestamp_values) if len(timestamp_values) > 1 else 0,
            "timestamps": timestamps
        }
    
    def wait_for_synchronized_time(self, timeout: float = 5.0) -> bool:
        """
        Wait for time synchronization to be ready.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if synchronized, False if timeout
        """
        # If using system time, it's always available
        if self.use_system_time:
            return True
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if ROS_VERSION == 2:
                if self.clock is not None:
                    if self.clock.now().nanoseconds > 0:
                        return True
                else:
                    # Clock not initialized, but system time is available
                    return True
            elif ROS_VERSION == 1:
                if rospy.Time.now().to_sec() > 0:
                    return True
            else:
                return True  # System time is always available
            
            time.sleep(0.01)
        
        return False

# Note: create_timestamp_dict() function removed as it was using get_current_time()
# instead of extracting real timestamps. Timestamp collection is now properly
# handled in real_env.py where we extract timestamps from actual data sources:
# - Camera timestamps from ROS message headers
# - Hand timestamps from ROS message headers  
# - Arm timestamps from Piper SDK

def log_synchronization_info(timestamps: Dict[str, float], validation_result: Tuple[bool, Dict[str, Any]]):
    """Log synchronization information for debugging."""
    is_valid, info = validation_result
    
    if is_valid:
        print(f"✓ Time synchronization OK - Max diff: {info.get('max_diff', 0):.3f}s")
    else:
        print(f"✗ Time synchronization failed: {info.get('error', 'Unknown error')}")
        if 'timestamps' in info:
            for sensor, ts in info['timestamps'].items():
                print(f"  {sensor}: {ts:.6f}")
