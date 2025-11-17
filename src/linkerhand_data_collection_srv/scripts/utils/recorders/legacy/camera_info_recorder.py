#!/usr/bin/env python3
# -*-coding:utf8-*-
import os
import sys
import time
import threading
import numpy as np
from typing import Optional, Dict, Any

# Legacy recorder retained for backward compatibility; expects topics to be provided externally.

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except Exception:
    rclpy = None
    Node = object  # type: ignore


class CameraInfoRecorder:
    def __init__(self, camera_names, node=None):
        self._owns_node = False
        self.node = node
        self.camera_names = camera_names
        self._first_log_done = set()
        
        # Store camera_info data
        self.camera_info_data = {}
        
        # If no node is provided, create a ROS2 node
        if self.node is None and rclpy is not None:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node('camera_info_recorder')
            print("CameraInfoRecorder: created node 'camera_info_recorder'")
            self._owns_node = True
        
        # Subscribe to camera_info topics
        for cam_name in self.camera_names:
            if self.node:
                # TODO: inject topic name via runtime configuration when legacy recorder is used.
                print(f"Warning: No camera_info topic found for {cam_name}. Please ensure CAMERA_INFO_TOPICS is defined.")
                # Example placeholder for CAMERA_INFO_TOPICS if not provided:
                # CAMERA_INFO_TOPICS = {
                #     "camera_name": "camera_info_topic_name"
                # }
                # This would require the user to pass CAMERA_INFO_TOPICS to the constructor.
                # For now, we'll just skip this camera if no topic is found.
                continue
        
        # Allow time for subscriptions to be established
        time.sleep(0.2)

    def _create_camera_info_sub(self, topic, cam_name):
        from sensor_msgs.msg import CameraInfo
        if self.node is None:
            return
        if hasattr(self.node, 'create_subscription'):
            if 'rclpy' in sys.modules and rclpy is not None:
                # Match RELIABLE publisher
                qos_reliable = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                )
                self.node.create_subscription(CameraInfo, topic, 
                                            lambda msg: self.camera_info_cb(cam_name, msg), 
                                            qos_reliable)
            else:
                self.node.create_subscription(CameraInfo, topic, 
                                            lambda msg: self.camera_info_cb(cam_name, msg), 10)

    def camera_info_cb(self, cam_name, data):
        """Callback for camera_info messages"""
        try:
            # Convert camera_info to a dictionary format for storage
            camera_info_dict = {
                'header': {
                    'frame_id': data.header.frame_id,
                    'stamp': {
                        'sec': data.header.stamp.sec,
                        'nanosec': data.header.stamp.nanosec
                    }
                },
                'height': data.height,
                'width': data.width,
                'distortion_model': data.distortion_model,
                'D': list(data.d),  # Distortion coefficients
                'K': list(data.k),  # Intrinsic camera matrix
                'R': list(data.r),  # Rectification matrix
                'P': list(data.p),  # Projection matrix
                'binning_x': data.binning_x,
                'binning_y': data.binning_y,
                'roi': {
                    'x_offset': data.roi.x_offset,
                    'y_offset': data.roi.y_offset,
                    'height': data.roi.height,
                    'width': data.roi.width,
                    'do_rectify': data.roi.do_rectify
                }
            }
            
            self.camera_info_data[cam_name] = camera_info_dict
            
            if cam_name not in self._first_log_done:
                print(f"Received camera_info: {cam_name}, resolution={data.width}x{data.height}")
                self._first_log_done.add(cam_name)
                
        except Exception as e:
            if cam_name not in self._first_log_done:
                print(f"Error processing camera_info for {cam_name}: {e}")
                self._first_log_done.add(cam_name)

    def get_camera_info(self, cam_name=None):
        """Get camera_info data for a specific camera or all cameras"""
        if cam_name is None:
            return self.camera_info_data
        return self.camera_info_data.get(cam_name)

    def wait_until_ready(self, required_names=None, timeout_sec: float = 3.0):
        """Block until required cameras have received camera_info or timeout."""
        if required_names is None:
            required_names = list(self.camera_names)
        
        print(f"Waiting for camera_info from: {required_names}")
        pending = set(required_names)
        start = time.time()
        while pending and (time.time() - start) < timeout_sec:
            for name in list(pending):
                if name in self.camera_info_data:
                    pending.discard(name)
            time.sleep(0.02)
        if pending:
            print(f"Camera_info wait timeout, still pending: {sorted(list(pending))}")
        else:
            print("All camera_info received!")

    def shutdown(self):
        """Cleanup resources"""
        if self._owns_node and rclpy is not None and rclpy.ok():
            try:
                self.node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()

    def print_diagnostics(self):
        """Print diagnostic information about received camera_info"""
        for cam_name in self.camera_names:
            if cam_name in self.camera_info_data:
                info = self.camera_info_data[cam_name]
                print(f'{cam_name}: {info["width"]}x{info["height"]}, frame_id={info["header"]["frame_id"]}')
            else:
                print(f'{cam_name}: No camera_info received')
        print()
