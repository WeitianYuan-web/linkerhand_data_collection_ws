#!/usr/bin/env python3
# -*-coding:utf8-*-
import time
from typing import Optional, Dict

from utils.config_loader import build_runtime_config

DEFAULT_TOPICS = {
    "left": {
        "state": "/left_arm_joint_state",
        "cmd": "/left_arm_joint_control",
        "dof": 7,
    },
    "right": {
        "state": "/right_arm_joint_state",
        "cmd": "/right_arm_joint_control",
        "dof": 7,
    },
}

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except Exception:
    rclpy = None
    Node = object  # type: ignore


class ArmRecorder:
    """ROS2-based recorder for robotic arm joint states and commands."""

    def __init__(self, side: str = 'left', node: Optional[Node] = None, task_name: str = ""):
        from sensor_msgs.msg import JointState
        self.side = side
        self.node: Optional[Node] = node
        self._owns_node = False
        self._spin_thread = None
        self._spin_active = False

        # latest state/command
        self.names = None
        self.qpos = None
        self.qvel = None
        self.effort = None
        self.cmd_names = None
        self.cmd_pos = None

        # determine topics
        config = self._resolve_config(task_name, side)
        state_topic = config["state_topic"]
        cmd_topic = config["cmd_topic"]
        self._expected_dof = config["dof"]
        self._dof = self._expected_dof

        # bring up node if necessary, but do not spin a separate thread/executor
        if self.node is None and rclpy is not None:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node(f'{side}_arm_recorder')
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

        # give time for initial messages (non-blocking if no publishers yet)
        time.sleep(0.3)

    def _resolve_config(self, task_name: str, side: str) -> Dict[str, object]:
        runtime_config = build_runtime_config(task_name) if task_name else None
        hardware = runtime_config.get('hardware', {}) if runtime_config else {}
        topics = hardware.get('topics', {})
        arm_joints = hardware.get('arm_joints', {})

        prefix = 'left' if side == 'left' else 'right'
        state_topic = topics.get(f'{prefix}_arm_state', DEFAULT_TOPICS[prefix]['state'])
        cmd_topic = topics.get(f'{prefix}_arm_cmd', DEFAULT_TOPICS[prefix]['cmd'])
        dof = arm_joints.get(f'{prefix}_arm_dof', DEFAULT_TOPICS[prefix]['dof'])

        return {
            'state_topic': state_topic,
            'cmd_topic': cmd_topic,
            'dof': dof,
        }

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

    def _cmd_cb(self, msg):
        self.cmd_names = list(msg.name) if msg.name else None
        self.cmd_pos = list(msg.position) if msg.position else None

        if self.cmd_pos:
            actual_dof = len(self.cmd_pos)
            if actual_dof and actual_dof != self._dof:
                self._dof = actual_dof

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

    # # Placeholder methods for backward compatibility =============================================
    # def get_end_pose(self):
    #     """Placeholder: Get end-effector pose (6 DOF: x, y, z, rx, ry, rz)"""
    #     # TODO: Implement forward kinematics or subscribe to end-effector pose topic
    #     return [0.0] * 6

    # def get_master_end_pose(self):
    #     """Placeholder: Get master end-effector pose (6 DOF: x, y, z, rx, ry, rz)"""
    #     # TODO: Implement forward kinematics or subscribe to end-effector pose topic
    #     return [0.0] * 6
