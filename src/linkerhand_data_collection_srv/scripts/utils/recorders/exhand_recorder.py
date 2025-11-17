#!/usr/bin/env python3
# -*-coding:utf8-*-
import os
import sys
import time
import json
import numpy as np
from typing import Optional, Dict, Any

from utils.config_loader import build_runtime_config

DEFAULT_TOPICS = {
    "left": {
        "sensor": "/exhand/sensor_data_left",
        "mapping": "/exhand/mapping_data_left",
        "status": "/exhand/status",
    },
    "right": {
        "sensor": "/exhand/sensor_data_right",
        "mapping": "/exhand/mapping_data_right",
        "status": "/exhand/status",
    },
}

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except Exception:
    rclpy = None
    Node = object  # type: ignore


class ExHandRecorder:
    """ROS2-based recorder for ExHand (exoskeleton glove) sensor and mapping data.

    Subscribes to ExHand sensor and mapping topics and exposes latest data.
    """

    def __init__(self, side: str = 'left', node: Optional[Node] = None, task_name: str = ""):
        from std_msgs.msg import UInt16MultiArray, Float32MultiArray, UInt8
        self.side = side
        self.node: Optional[Node] = node
        self._owns_node = False
        self._spin_thread = None
        self._spin_active = False

        # latest sensor/mapping data
        self.sensor_data = None  # 15 uint16 values
        self.mapping_data = None  # 15 float values
        self.status_data = None  # uint8 status
        
        # Store timestamps from ROS messages
        self._latest_sensor_timestamp = None
        self._latest_mapping_timestamp = None
        self._latest_status_timestamp = None

        # determine topics
        config = self._resolve_config(side, task_name)
        sensor_topic = config["sensor_topic"]
        mapping_topic = config["mapping_topic"]
        status_topic = config["status_topic"]

        # bring up node if necessary, but do not spin a separate thread/executor
        if self.node is None and rclpy is not None:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node(f'{side}_exhand_recorder')
            self._owns_node = True

        if self.node is not None and hasattr(self.node, 'create_subscription'):
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.node.create_subscription(UInt16MultiArray, sensor_topic, self._sensor_cb, qos)
            self.node.create_subscription(Float32MultiArray, mapping_topic, self._mapping_cb, qos)
            self.node.create_subscription(UInt8, status_topic, self._status_cb, qos)

        # give time for initial messages (non-blocking if no publishers yet)
        time.sleep(0.3)

    def _spin_loop(self):
        # Deprecated: no background spinning; callbacks run on outer executor
        return

    def shutdown(self):
        self._spin_active = False
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
        if self._owns_node and rclpy is not None and rclpy.ok():
            try:
                self.node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()

    # Callbacks
    def _sensor_cb(self, msg):
        """Callback for sensor data messages (15 uint16 values)."""
        self.sensor_data = list(msg.data) if msg.data else None
        
        # Extract timestamp from ROS message header
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            self._latest_sensor_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self._latest_sensor_timestamp = time.time()

    def _mapping_cb(self, msg):
        """Callback for mapping data messages (15 float values)."""
        self.mapping_data = list(msg.data) if msg.data else None
        
        # Extract timestamp from ROS message header
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            self._latest_mapping_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self._latest_mapping_timestamp = time.time()

    def _status_cb(self, msg):
        """Callback for status messages."""
        self.status_data = msg.data if hasattr(msg, 'data') else None
        
        # Extract timestamp from ROS message header
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            self._latest_status_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self._latest_status_timestamp = time.time()

    # Accessors
    def get_sensor_data(self):
        """Get latest sensor data (15 uint16 values)."""
        if self.sensor_data is None:
            return [0] * 15
        if len(self.sensor_data) != 15:
            # Pad or truncate to 15
            if len(self.sensor_data) < 15:
                return list(self.sensor_data) + [0] * (15 - len(self.sensor_data))
            else:
                return self.sensor_data[:15]
        return self.sensor_data

    def get_mapping_data(self):
        """Get latest mapping data (15 float values)."""
        if self.mapping_data is None:
            return [0.0] * 15
        if len(self.mapping_data) != 15:
            # Pad or truncate to 15
            if len(self.mapping_data) < 15:
                return list(self.mapping_data) + [0.0] * (15 - len(self.mapping_data))
            else:
                return self.mapping_data[:15]
        return self.mapping_data

    def get_status(self):
        """Get latest status data."""
        return self.status_data if self.status_data is not None else 0
    
    def get_latest_timestamps(self) -> dict:
        """Get the latest timestamps from all data sources.
        
        Returns:
            dict: Dictionary containing timestamps for sensor, mapping, and status data
        """
        return {
            'sensor_timestamp': self._latest_sensor_timestamp,
            'mapping_timestamp': self._latest_mapping_timestamp,
            'status_timestamp': self._latest_status_timestamp
        }
    
    def get_representative_timestamp(self) -> float:
        """Get the most representative timestamp for this hand.
        
        Returns:
            float: The most recent timestamp available, or current time if none available
        """
        timestamps = [
            self._latest_sensor_timestamp,
            self._latest_mapping_timestamp,
            self._latest_status_timestamp
        ]
        
        # Return the most recent non-None timestamp
        valid_timestamps = [ts for ts in timestamps if ts is not None]
        if valid_timestamps:
            return max(valid_timestamps)
        else:
            return time.time()

    def _resolve_config(self, side: str, task_name: str) -> Dict[str, object]:
        runtime_config = build_runtime_config(task_name) if task_name else None
        hardware = runtime_config.get('hardware', {}) if runtime_config else {}
        topics = hardware.get('topics', {})

        prefix = 'left' if side == 'left' else 'right'
        sensor_topic = topics.get(f'{prefix}_exhand_sensor', DEFAULT_TOPICS[prefix]['sensor'])
        mapping_topic = topics.get(f'{prefix}_exhand_mapping', DEFAULT_TOPICS[prefix]['mapping'])
        status_topic = topics.get('exhand_status', DEFAULT_TOPICS[prefix]['status'])

        return {
            'sensor_topic': sensor_topic,
            'mapping_topic': mapping_topic,
            'status_topic': status_topic,
        }

