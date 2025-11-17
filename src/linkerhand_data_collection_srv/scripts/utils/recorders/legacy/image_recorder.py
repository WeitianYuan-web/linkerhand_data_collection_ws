#!/usr/bin/env python3
# -*-coding:utf8-*-
import os
import time
import threading
import numpy as np
from typing import Optional

# Legacy image recorder retained for backward compatibility.
# Modern pipeline should inject camera topics explicitly when instantiating.

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except Exception:
    rclpy = None
    Node = object  # type: ignore


class ImageRecorder:
    def __init__(self, camera_names, node=None, stereo_mode=False):
        from collections import deque
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        self.bridge = CvBridge()
        self._first_log_done = set()
        self._owns_node = False
        self.node = node
        self.camera_names = camera_names
        self._spin_thread = None
        self._spin_active = False
        self.stereo_mode = stereo_mode
        
        # Initialize time synchronization
        try:
            # Try relative import first
            from ..time_sync import TimeSyncManager
            self.time_sync_manager = TimeSyncManager()
        except ImportError:
            try:
                # Try absolute import as fallback
                from utils.time_sync import TimeSyncManager
                self.time_sync_manager = TimeSyncManager()
            except ImportError:
                self.time_sync_manager = None
                print("Warning: TimeSyncManager not available")

        # If no node is provided, create a ROS2 node; do NOT spin in background
        if self.node is None and rclpy is not None:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node('image_recorder')
            print("ImageRecorder: created node 'image_recorder'")
            self._owns_node = True
            # No background spin here; rely on outer executor if any
        for cam_name in self.camera_names:
            if self.stereo_mode:
                # Stereo mode: subscribe to both left and right streams
                stereo_topics = {}
                if stereo_topics:
                    # Predefine attributes for stereo streams
                    setattr(self, f'{cam_name}_left_image', None)
                    setattr(self, f'{cam_name}_right_image', None)
                    setattr(self, f'{cam_name}_left_secs', None)
                    setattr(self, f'{cam_name}_left_nsecs', None)
                    setattr(self, f'{cam_name}_right_secs', None)
                    setattr(self, f'{cam_name}_right_nsecs', None)
                    
                    # Subscribe to left stream
                    if 'left' in stereo_topics:
                        left_topic = stereo_topics['left']
                        print(f"ImageRecorder: subscribing {cam_name}_left -> {left_topic}")
                        self._create_image_sub(left_topic, lambda msg, cam=cam_name, stream='left': self.image_cb_stereo(cam, stream, msg))
                    
                    # Subscribe to right stream
                    if 'right' in stereo_topics:
                        right_topic = stereo_topics['right']
                        print(f"ImageRecorder: subscribing {cam_name}_right -> {right_topic}")
                        self._create_image_sub(right_topic, lambda msg, cam=cam_name, stream='right': self.image_cb_stereo(cam, stream, msg))
                else:
                    print(f"Warning: No stereo topics found for {cam_name}")
            else:
                # Mono mode: subscribe to single stream (legacy behavior)
                setattr(self, f'{cam_name}_image', None)
                setattr(self, f'{cam_name}_secs', None)
                setattr(self, f'{cam_name}_nsecs', None)
                
                if cam_name == "cam_left_wrist":
                    callback_func = self.image_cb_cam_right_wrist
                    if self.node:
                        topic = "/cam_left_wrist/zed_node/left_raw/image_raw_color"
                        print(f"ImageRecorder: subscribing {cam_name} -> {topic}")
                        self._create_image_sub(topic, callback_func)
                elif cam_name == "cam_right_wrist":
                    callback_func = self.image_cb_cam_left_wrist
                    if self.node:
                        topic = "/cam_right_wrist/zed_node/left_raw/image_raw_color"
                        print(f"ImageRecorder: subscribing {cam_name} -> {topic}")
                        self._create_image_sub(topic, callback_func)
                elif cam_name == "cam_top":
                    callback_func = self.image_cb_cam_top
                    if self.node:
                        topic = "/cam_top/zed_node/left_raw/image_raw_color"
                        print(f"ImageRecorder: subscribing {cam_name} -> {topic}")
                        self._create_image_sub(topic, callback_func)
                else:
                    raise NotImplementedError

        # allow buffers to fill briefly
        time.sleep(0.2)

    def _create_image_sub(self, topic, callback):
        from sensor_msgs.msg import Image
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
                self.node.create_subscription(Image, topic, callback, qos_reliable)
            else:
                self.node.create_subscription(Image, topic, callback, 10)

    def _spin_loop(self):
        # Deprecated: no background spinning; keep for backward compat
        return

    def shutdown(self):
        # Stop background spinner if owned
        self._spin_active = False
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
        if self._owns_node and rclpy is not None and rclpy.ok():
            try:
                self.node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()

    def image_cb(self, cam_name, data):
        img = None
        try:
            # Prefer a consistent 3-channel format when possible
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception:
            try:
                img = self.bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
            except Exception:
                try:
                    img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
                except Exception as e:
                    # Conversion failed; leave as None and log once
                    if cam_name not in self._first_log_done:
                        print(f"图像转换失败: {cam_name}, encoding={getattr(data, 'encoding', 'unknown')}, error={e}")
                        self._first_log_done.add(cam_name)
                    return
        
        # Store image and timestamp
        setattr(self, f'{cam_name}_image', img)
        
        # Extract and store timestamp using time sync manager
        if self.time_sync_manager:
            timestamp = self.time_sync_manager.get_timestamp_from_msg(data)
            if timestamp is not None:
                setattr(self, f'{cam_name}_timestamp', timestamp)
            else:
                # Fallback to current time if extraction fails
                setattr(self, f'{cam_name}_timestamp', self.time_sync_manager.get_current_time())
        
        # Legacy timestamp handling for backward compatibility
        if hasattr(data.header.stamp, 'sec'):  # ROS2 format
            setattr(self, f'{cam_name}_secs', data.header.stamp.sec)
            setattr(self, f'{cam_name}_nsecs', data.header.stamp.nanosec)
        
        if cam_name not in self._first_log_done and img is not None:
            try:
                shape_info = getattr(img, 'shape', None)
                print(f"收到图像: {cam_name}, encoding={getattr(data, 'encoding', 'unknown')}, shape={shape_info}")
            finally:
                self._first_log_done.add(cam_name)

    def image_cb_stereo(self, cam_name, stream, data):
        """Callback for stereo camera streams"""
        img = None
        try:
            # Prefer a consistent 3-channel format when possible
            img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception:
            try:
                img = self.bridge.imgmsg_to_cv2(data, desired_encoding='rgb8')
            except Exception:
                try:
                    img = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
                except Exception as e:
                    # Conversion failed; leave as None and log once
                    log_key = f"{cam_name}_{stream}"
                    if log_key not in self._first_log_done:
                        print(f"图像转换失败: {cam_name}_{stream}, encoding={getattr(data, 'encoding', 'unknown')}, error={e}")
                        self._first_log_done.add(log_key)
                    return
        
        # Store image and timestamp for specific stream
        setattr(self, f'{cam_name}_{stream}_image', img)
        
        # Extract and store timestamp using time sync manager
        if self.time_sync_manager:
            timestamp = self.time_sync_manager.get_timestamp_from_msg(data)
            if timestamp is not None:
                setattr(self, f'{cam_name}_{stream}_timestamp', timestamp)
            else:
                # Fallback to current time if extraction fails
                setattr(self, f'{cam_name}_{stream}_timestamp', self.time_sync_manager.get_current_time())
        
        # Legacy timestamp handling for backward compatibility
        if hasattr(data.header.stamp, 'sec'):  # ROS2 format
            setattr(self, f'{cam_name}_{stream}_secs', data.header.stamp.sec)
            setattr(self, f'{cam_name}_{stream}_nsecs', data.header.stamp.nanosec)
        
        log_key = f"{cam_name}_{stream}"
        if log_key not in self._first_log_done and img is not None:
            try:
                shape_info = getattr(img, 'shape', None)
                print(f"收到图像: {cam_name}_{stream}, encoding={getattr(data, 'encoding', 'unknown')}, shape={shape_info}")
            finally:
                self._first_log_done.add(log_key)

    def image_cb_cam_right_wrist(self, data): #TODO: change hardcoding
        cam_name = 'cam_right_wrist'
        return self.image_cb(cam_name, data)

    def image_cb_cam_left_wrist(self, data): #TODO: change hardcoding
        cam_name = 'cam_left_wrist'
        return self.image_cb(cam_name, data)

    def image_cb_cam_top(self, data): #TODO: change hardcoding
        cam_name = 'cam_top'
        return self.image_cb(cam_name, data)

    def get_images(self):
        image_dict = dict()
        for cam_name in self.camera_names:
            if self.stereo_mode:
                # Stereo mode: return both left and right images
                left_image = getattr(self, f'{cam_name}_left_image', None)
                right_image = getattr(self, f'{cam_name}_right_image', None)
                
                if left_image is None and right_image is None:
                    print(f"相机 {cam_name} 没有立体图像数据")
                    continue
                
                # Store both streams
                image_dict[f'{cam_name}_left'] = left_image
                image_dict[f'{cam_name}_right'] = right_image
            else:
                # Mono mode: return single image (legacy behavior)
                image_attr = getattr(self, f'{cam_name}_image', None)
                if image_attr is None:
                    print(f"相机 {cam_name} 没有图像数据")
                    continue
                image_dict[cam_name] = image_attr
        return image_dict
    
    def get_timestamps(self):
        """Get timestamps for all cameras."""
        timestamp_dict = dict()
        for cam_name in self.camera_names:
            if self.stereo_mode:
                # Stereo mode: get timestamps for both streams
                left_timestamp = getattr(self, f'{cam_name}_left_timestamp', None)
                right_timestamp = getattr(self, f'{cam_name}_right_timestamp', None)
                
                if left_timestamp is not None:
                    timestamp_dict[f'{cam_name}_left'] = left_timestamp
                if right_timestamp is not None:
                    timestamp_dict[f'{cam_name}_right'] = right_timestamp
            else:
                # Mono mode: get single timestamp (legacy behavior)
                timestamp_attr = getattr(self, f'{cam_name}_timestamp', None)
                if timestamp_attr is not None:
                    timestamp_dict[cam_name] = timestamp_attr
        return timestamp_dict
    
    def validate_synchronization(self):
        """Validate that all camera timestamps are synchronized."""
        if not self.time_sync_manager:
            return True, {"warning": "Time sync manager not available"}
        
        timestamps = self.get_timestamps()
        if not timestamps:
            return False, {"error": "No timestamps available"}
        
        return self.time_sync_manager.validate_timestamps(timestamps)

    def wait_until_ready(self, required_names=None, timeout_sec: float = 3.0):
        """Block until required cameras have received at least one frame or timeout."""
        if required_names is None:
            required_names = list(self.camera_names)
        
        print(required_names)
        pending = set(required_names)
        start = time.time()
        while pending and (time.time() - start) < timeout_sec:
            for name in list(pending):
                if self.stereo_mode:
                    # Stereo mode: check both left and right images
                    left_image = getattr(self, f'{name}_left_image', None)
                    right_image = getattr(self, f'{name}_right_image', None)
                    if left_image is not None and right_image is not None:
                        pending.discard(name)
                else:
                    # Mono mode: check single image (legacy behavior)
                    if getattr(self, f'{name}_image') is not None:
                        pending.discard(name)
            time.sleep(0.02)
        if pending:
            print(f"等待相机超时，仍未准备: {sorted(list(pending))}")

    def print_diagnostics(self):
        def dt_helper(l):
            l = np.array(l)
            diff = l[1:] - l[:-1]
            return np.mean(diff)
        for cam_name in self.camera_names:
            image_freq = 1 / dt_helper(getattr(self, f'{cam_name}_timestamps'))
            print(f'{cam_name} {image_freq=:.2f}')
        print()
