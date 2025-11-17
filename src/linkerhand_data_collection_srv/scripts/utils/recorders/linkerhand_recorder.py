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
        "state": "/cb_left_hand_state_arc",
        "cmd": "/cb_left_hand_control_cmd",
        "tactile": "/cb_left_hand_matrix_touch",
        "dof": 10,
    },
    "right": {
        "state": "/cb_right_hand_state_arc",
        "cmd": "/cb_right_hand_control_cmd",
        "tactile": "/cb_right_hand_matrix_touch",
        "dof": 10,
    },
}

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except Exception:
    rclpy = None
    Node = object  # type: ignore


class LinkerHandRecorder:
    """ROS2-based recorder for LinkerHand joint states and commands.

    Subscribes to state and command topics and exposes latest positions/velocities/efforts.
    Conditionally subscribes to tactile topics based on hand model (O6 has no tactile sensors).
    """

    def __init__(self, side: str = 'left', node: Optional[Node] = None, enable_tactile: bool = True, task_name: str = ""):
        from sensor_msgs.msg import JointState        
        self.side = side
        self.node: Optional[Node] = node
        self._owns_node = False
        self._spin_thread = None
        self._spin_active = False
        self.enable_tactile = enable_tactile  # Control tactile sensor subscription

        # latest state/command
        self.names = None
        self.qpos = None
        self.qvel = None
        self.effort = None
        self.cmd_names = None
        self.cmd_pos = None
        
        # Store timestamps from ROS messages
        self._latest_state_timestamp = None
        self._latest_cmd_timestamp = None

        # latest tactile data - 5 fingers (thumb, index, middle, ring, little)
        # Each finger has a 12x6 matrix
        self.tactile_data = {
            'thumb_matrix': np.zeros((12, 6), dtype=np.float32),
            'index_matrix': np.zeros((12, 6), dtype=np.float32),
            'middle_matrix': np.zeros((12, 6), dtype=np.float32),
            'ring_matrix': np.zeros((12, 6), dtype=np.float32),
            'little_matrix': np.zeros((12, 6), dtype=np.float32)
        }
        self.tactile_timestamp = None

        # determine topics
        config = self._resolve_config(side, task_name)
        state_topic = config["state_topic"]
        cmd_topic = config["cmd_topic"]
        tactile_topic = config["tactile_topic"]
        self._expected_dof = config["dof"]
        self._dof = self._expected_dof

        # bring up node if necessary, but do not spin a separate thread/executor
        if self.node is None and rclpy is not None:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node(f'{side}_linkerhand_recorder')
            self._owns_node = True

        if self.node is not None and hasattr(self.node, 'create_subscription'):
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.node.create_subscription(JointState, state_topic, self._state_cb, qos)
            self.node.create_subscription(JointState, cmd_topic, self._cmd_cb, qos)
            
            # Conditionally subscribe to tactile topic (O6 has no tactile sensors)
            if self.enable_tactile:
                from std_msgs.msg import String
                self.node.create_subscription(String, tactile_topic, self._tactile_cb, qos)
                print(f"LinkerHandRecorder [{side}]: Tactile sensor enabled")
            else:
                print(f"LinkerHandRecorder [{side}]: Tactile sensor disabled (not supported by this hand model)")

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
    def _state_cb(self, msg):
        self.names = list(msg.name) if msg.name else None
        self.qpos = list(msg.position) if msg.position else None
        self.qvel = list(msg.velocity) if msg.velocity else None
        self.effort = list(msg.effort) if msg.effort else None

        if self.qpos:
            actual_dof = len(self.qpos)
            if actual_dof and actual_dof != self._dof:
                self._dof = actual_dof
        
        # Extract timestamp from ROS message header
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            self._latest_state_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self._latest_state_timestamp = time.time()

    def _cmd_cb(self, msg):
        self.cmd_names = list(msg.name) if msg.name else None
        self.cmd_pos = list(msg.position) if msg.position else None

        if self.cmd_pos:
            actual_dof = len(self.cmd_pos)
            if actual_dof and actual_dof != self._dof:
                self._dof = actual_dof
        
        # Extract timestamp from ROS message header
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            self._latest_cmd_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            self._latest_cmd_timestamp = time.time()

    def _tactile_cb(self, msg):
        """Callback for tactile data messages.
        
        Expects JSON string with format:
        {
            "thumb_matrix": [[...], [...], ...],  # 12x6 array
            "index_matrix": [[...], [...], ...],  # 12x6 array
            "middle_matrix": [[...], [...], ...], # 12x6 array
            "ring_matrix": [[...], [...], ...],   # 12x6 array
            "little_matrix": [[...], [...], ...]  # 12x6 array
        }
        """
        try:
            # Parse JSON data
            data = json.loads(msg.data)
            
            # Update tactile matrices
            for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                if finger in data:
                    matrix_data = data[finger]
                    if isinstance(matrix_data, list):
                        # Convert to numpy array
                        matrix = np.array(matrix_data, dtype=np.float32)
                        
                        # Handle both 12x6 and 6x12 formats
                        if matrix.shape == (12, 6):
                            # Already in correct format
                            self.tactile_data[finger] = matrix
                        elif matrix.shape == (6, 12):
                            # Transpose from 6x12 to 12x6
                            self.tactile_data[finger] = matrix.T
                        else:
                            print(f"Warning: Invalid matrix shape for {finger}: {matrix.shape}, expected (12, 6) or (6, 12)")
                    else:
                        print(f"Warning: Invalid matrix data for {finger} - not a list")
                else:
                    print(f"Warning: Missing {finger} in tactile data")
            
            # Update timestamp
            self.tactile_timestamp = time.time()
            
        except json.JSONDecodeError as e:
            print(f"Error parsing tactile JSON data: {e}")
        except Exception as e:
            print(f"Error processing tactile data: {e}")

    # Accessors
    def get_qpos(self):
        return self._ensure_length(self.qpos)

    def get_qvel(self):
        return self._ensure_length(self.qvel)

    def get_effort(self):
        return self._ensure_length(self.effort)

    def get_cmd_pos(self):
        return self._ensure_length(self.cmd_pos)

    def get_dof(self) -> int:
        return self._dof

    def _ensure_length(self, data):
        expected = self._expected_dof or self._dof
        if data is None:
            return [0.0] * expected

        if len(data) == expected:
            return data[:expected]

        if len(data) > expected:
            return data[:expected]

        padded = list(data) + [0.0] * (expected - len(data))
        return padded

    # Tactile data accessors
    def get_tactile_data(self) -> Dict[str, np.ndarray]:
        """Get the latest tactile data for all fingers.
        
        Returns:
            Dict with keys: 'thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix'
            Each value is a numpy array of shape (12, 6)
        """
        return self.tactile_data.copy()

    def get_finger_tactile(self, finger: str) -> np.ndarray:
        """Get tactile data for a specific finger.
        
        Args:
            finger: One of 'thumb', 'index', 'middle', 'ring', 'little'
            
        Returns:
            Numpy array of shape (12, 6) for the specified finger
        """
        finger_key = f'{finger}_matrix'
        if finger_key in self.tactile_data:
            return self.tactile_data[finger_key].copy()
        else:
            print(f"Warning: Unknown finger '{finger}', returning zeros")
            return np.zeros((12, 6), dtype=np.float32)

    def get_tactile_summary(self) -> Dict[str, float]:
        """Get summary statistics for tactile data.
        
        Returns:
            Dict with summary statistics for each finger
        """
        summary = {}
        for finger in ['thumb', 'index', 'middle', 'ring', 'little']:
            matrix = self.get_finger_tactile(finger)
            summary[f'{finger}_mean'] = float(np.mean(matrix))
            summary[f'{finger}_max'] = float(np.max(matrix))
            summary[f'{finger}_min'] = float(np.min(matrix))
            summary[f'{finger}_std'] = float(np.std(matrix))
        return summary

    def get_tactile_flat(self) -> np.ndarray:
        """Get all tactile data as a flattened array.
        
        Returns:
            Flattened numpy array of shape (360,) containing all tactile data
            (5 fingers * 12 * 6 = 360 values)
        """
        flat_data = []
        for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
            flat_data.extend(self.tactile_data[finger].flatten())
        return np.array(flat_data, dtype=np.float32)

    def is_tactile_data_available(self) -> bool:
        """Check if tactile data has been received.
        
        Returns:
            True if tactile data has been received, False otherwise
        """
        return self.tactile_timestamp is not None
    
    def get_latest_timestamps(self) -> dict:
        """Get the latest timestamps from all data sources.
        
        Returns:
            dict: Dictionary containing timestamps for state, command, and tactile data
        """
        return {
            'state_timestamp': self._latest_state_timestamp,
            'cmd_timestamp': self._latest_cmd_timestamp,
            'tactile_timestamp': self.tactile_timestamp
        }
    
    def get_representative_timestamp(self) -> float:
        """Get the most representative timestamp for this hand.
        
        Returns:
            float: The most recent timestamp available, or current time if none available
        """
        timestamps = [
            self._latest_state_timestamp,
            self._latest_cmd_timestamp,
            self.tactile_timestamp
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
        hand_joints = hardware.get('hand_joints', {})

        prefix = 'left' if side == 'left' else 'right'
        state_topic = topics.get(f'{prefix}_hand_state', DEFAULT_TOPICS[prefix]['state'])
        cmd_topic = topics.get(f'{prefix}_hand_cmd', DEFAULT_TOPICS[prefix]['cmd'])
        tactile_topic = topics.get(f'{prefix}_hand_tactile', DEFAULT_TOPICS[prefix]['tactile'])
        dof = hand_joints.get(f'{prefix}_hand_dof', self._infer_hand_dof(runtime_config, prefix))

        return {
            'state_topic': state_topic,
            'cmd_topic': cmd_topic,
            'tactile_topic': tactile_topic,
            'dof': dof,
        }

    def _infer_hand_dof(self, runtime_config: Optional[Dict[str, Any]], prefix: str) -> int:
        if runtime_config:
            hands_section = runtime_config.get('hardware', {}).get('hands', {})
            dof_from_preset = hands_section.get('dof_per_hand')
            if isinstance(dof_from_preset, int) and dof_from_preset > 0:
                return dof_from_preset
        return DEFAULT_TOPICS[prefix]['dof']
