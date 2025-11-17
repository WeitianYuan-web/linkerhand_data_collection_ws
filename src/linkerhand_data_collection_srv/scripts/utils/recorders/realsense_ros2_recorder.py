#!/usr/bin/env python3
# -*-coding:utf8-*-
import os
import time
import threading
import numpy as np
from typing import Optional, Dict, Any, List
import cv2
from collections import deque

from utils.config_loader import build_runtime_config

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge
except Exception:
    rclpy = None
    Node = object  # type: ignore
    Image = object  # type: ignore
    CameraInfo = object  # type: ignore
    CvBridge = None


class RealSenseROS2Recorder:
    """
    Intel RealSense camera recorder using ROS2 topics.
    
    This class subscribes to RealSense camera topics and provides synchronized
    image capture with timestamp management.
    """
    
    def __init__(self, camera_names: List[str], node=None, stereo_mode=False, video_recording=False, video_codec='mp4v', video_quality=80, task_name: str = "", use_system_time: bool = False):
        """
        Initialize RealSense ROS2 recorder.
        
        Args:
            camera_names: List of camera names to record from
            node: Optional ROS2 node for subscriptions
            stereo_mode: Whether to capture stereo images
            video_recording: Whether to enable video recording (unused for now)
            video_codec: Video codec for recording (unused for now)
            video_quality: Video quality for recording (unused for now)
            task_name: Task name for configuration lookup
            use_system_time: Force using system time instead of ROS time
        """
        if rclpy is None:
            raise ImportError("ROS2 not available. Install ROS2 and required packages.")
        
        self.camera_names = camera_names
        self.node = node
        self.stereo_mode = stereo_mode
        self._first_log_done = set()
        self._owns_node = False
        self.camera_config = self._resolve_camera_topics(task_name)
        # Debug output removed for cleaner logs
        # print(f"[DEBUG——realsense_ros2_recorder] camera topic_map = {self.camera_config.get('topic_map')}")
        # print(f"[DEBUG——realsense_ros2_recorder] task_name = {task_name}")
        # print(f"[DEBUG——realsense_ros2_recorder] self.camera_config = {self.camera_config}")

        self.color_format_map = self.camera_config.get('color_format_map', {})
        if self.camera_config.get('stereo_mode', False):
            self.stereo_mode = True

        # Store use_system_time for later use
        self.use_system_time = use_system_time
        
        # If use_system_time not explicitly provided, try to get from task config
        if not use_system_time and task_name:
            try:
                from utils.config_loader import build_runtime_config
                runtime_config = build_runtime_config(task_name)
                self.use_system_time = runtime_config.get('collection', {}).get('use_system_time', False)
            except Exception:
                pass

        if camera_names:
            self.camera_names = camera_names
        else:
            enable_flags = self.camera_config.get('enable_flags', {})
            topic_map = self.camera_config.get('topic_map', {})
            self.camera_names = [
                cam for cam, enabled in enable_flags.items()
                if enabled and topic_map.get(cam)
            ]
            if not self.camera_names:
                self.camera_names = list(topic_map.keys())
            if not self.camera_names:
                detected = detect_realsense_cameras()
                if not detected:
                    raise RuntimeError("No RealSense cameras detected. Please ensure cameras are running.")
                self.camera_names = detected
        
        self._setup_recorder_state()
    
    def _resolve_camera_topics(self, task_name: str) -> Dict[str, Dict[str, Any]]:
        runtime_config = build_runtime_config(task_name) if task_name else {}
        cameras = runtime_config.get('cameras', {})

        config = {
            'topic_map': cameras.get('topic_map', {}),
            'info_topic_map': cameras.get('info_topic_map', {}),
            'stereo_topic_map': cameras.get('stereo_topic_map', {}),
            'stereo_info_topic_map': cameras.get('stereo_info_topic_map', {}),
            'color_format_map': cameras.get('color_format_map', {}),
            'camera_resolutions': cameras.get('camera_resolutions', {}),
            'enable_flags': cameras.get('enable_flags', {}),
            'preset': cameras.get('preset'),
            'stereo_mode': cameras.get('stereo_mode', False),
        }

        # Fallback resolution & topics from preset if mappings are missing
        preset_cameras = cameras.get('cameras', [])
        for cam in preset_cameras:
            name = cam.get('name')
            if not name:
                continue
            topic = cam.get('topic')
            info_topic = cam.get('topic_info')
            resolution = cam.get('resolution', [640, 480])
            fps = cam.get('fps', 30)
            config['topic_map'][name] = topic
            config['info_topic_map'][name] = info_topic
            config['camera_resolutions'][name] = {'width': resolution[0], 'height': resolution[1], 'fps': fps}

        return config
    
    def _setup_recorder_state(self):
        """
        Sets up the recorder state, including subscriptions and time synchronization.
        """
        # Initialize time synchronization with use_system_time parameter
        try:
            from ..time_sync import TimeSyncManager  # type: ignore
            self.time_sync_manager = TimeSyncManager(use_system_time=self.use_system_time)
        except ImportError:
            try:
                from utils.time_sync import TimeSyncManager
                self.time_sync_manager = TimeSyncManager(use_system_time=self.use_system_time)
            except ImportError:
                self.time_sync_manager = None
                print("Warning: TimeSyncManager not available")
        
        # If no node is provided, we cannot create one to avoid conflicts
        # The caller must provide a node
        if self.node is None:
            raise ValueError("RealSenseROS2Recorder requires a ROS2 node to be provided. Cannot create new node to avoid conflicts.")
        
        # Initialize CV bridge
        if CvBridge is not None:
            self.bridge = CvBridge()
        else:
            raise ImportError("cv_bridge not available. Install with: sudo apt install ros-jazzy-cv-bridge")
        
        # Storage for images and timestamps
        self._image_data = {}
        self._timestamp_data = {}
        self._camera_info_data = {}
        
        # Thread locks for thread-safe access
        self._data_locks = {}
        
        # Initialize data storage for each camera
        for cam_name in self.camera_names:
            self._data_locks[cam_name] = threading.Lock()
            self._image_data[cam_name] = {
                'left': None,
                'right': None if self.stereo_mode else None
            }
            self._timestamp_data[cam_name] = {
                'left': None,
                'right': None if self.stereo_mode else None
            }
            self._camera_info_data[cam_name] = {
                'left': None,
                'right': None if self.stereo_mode else None
            }
        
        # Subscribe to camera topics
        self._create_subscriptions()
        
        # Allow time for subscriptions to be established and camera to stabilize
        time.sleep(2.0)
        
        # Don't wait here - let the caller handle waiting
        # This prevents blocking during initialization
    
    def _create_subscriptions(self):
        """Create ROS2 subscriptions for camera topics."""
        if not self.node:
            return

        mapping = self.camera_config.get('topic_map', {})
        info_mapping = self.camera_config.get('info_topic_map', {})
        stereo_mapping = self.camera_config.get('stereo_topic_map', {})
        stereo_info_mapping = self.camera_config.get('stereo_info_topic_map', {})
        
        # Set up QoS profile for reliable image transport
        # Use SYSTEM_DEFAULT to match the publisher's QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.SYSTEM_DEFAULT,
            history=HistoryPolicy.SYSTEM_DEFAULT,
            depth=10  # Increase buffer size
        )
        
        for cam_name in self.camera_names:
            if self.stereo_mode:
                # Stereo mode: subscribe to both left and right streams
                # Map camera names to actual RealSense topics
                stereo_topics = stereo_mapping.get(cam_name, {})
                stereo_info_topics = stereo_info_mapping.get(cam_name, {})
                
                if stereo_topics:
                    # Subscribe to left stream (color)
                    if 'left' in stereo_topics:
                        left_topic = stereo_topics['left']
                        self.node.create_subscription(
                            Image, left_topic,
                            lambda msg, cam=cam_name, stream='left': self.image_callback(cam, stream, msg),
                            qos_profile
                        )
                    
                    # Subscribe to right stream (infrared)
                    if 'right' in stereo_topics:
                        right_topic = stereo_topics['right']
                        self.node.create_subscription(
                            Image, right_topic,
                            lambda msg, cam=cam_name, stream='right': self.image_callback(cam, stream, msg),
                            qos_profile
                        )
                    
                    # Use the stereo_info_topics we already defined above
                    if 'left' in stereo_info_topics:
                        left_info_topic = stereo_info_topics['left']
                        self.node.create_subscription(
                            CameraInfo, left_info_topic,
                            lambda msg, cam=cam_name, stream='left': self.camera_info_callback(cam, stream, msg),
                            qos_profile
                        )
                    
                    if 'right' in stereo_info_topics:
                        right_info_topic = stereo_info_topics['right']
                        self.node.create_subscription(
                            CameraInfo, right_info_topic,
                            lambda msg, cam=cam_name, stream='right': self.camera_info_callback(cam, stream, msg),
                            qos_profile
                        )
                else:
                    print(f"Warning: No stereo topics found for {cam_name}")
            else:
                # Mono mode: subscribe to single stream
                # Map camera names to actual RealSense topics
                topic = mapping.get(cam_name)
                info_topic = info_mapping.get(cam_name)
                
                if topic:
                    subscription = self.node.create_subscription(
                        Image, topic,
                        lambda msg, cam=cam_name, stream='left': self.image_callback(cam, stream, msg),
                        qos_profile
                    )
                else:
                    print(f"Warning: No topic found for {cam_name}")
                    continue
                
                if info_topic:
                    self.node.create_subscription(
                        CameraInfo, info_topic,
                        lambda msg, cam=cam_name, stream='left': self.camera_info_callback(cam, stream, msg),
                        qos_profile
                    )
    
    def image_callback(self, cam_name: str, stream: str, msg: Image):
        """Callback for image messages."""
        try:
            # Track first message received (no verbose output)
            if f"{cam_name}_{stream}" not in self._first_log_done:
                self._first_log_done.add(f"{cam_name}_{stream}")
            
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Convert rgb8 to rgb24 format for video encoding
            if cv_image.dtype == np.uint8:
                # rgb8 is already uint8, but we need to ensure it's in the right format for rgb24
                # rgb8 and rgb24 are essentially the same for uint8 arrays
                pass  # No conversion needed, rgb8 is already compatible with rgb24
            
            # Get synchronized timestamp
            if self.time_sync_manager:
                # Use ROS message timestamp for better synchronization
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            else:
                # Use ROS message timestamp
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # Store image and timestamp
            with self._data_locks[cam_name]:
                self._image_data[cam_name][stream] = cv_image.copy()
                self._timestamp_data[cam_name][stream] = timestamp
            
            # Track first successful capture (no verbose output)
            if f"{cam_name}_{stream}_captured" not in self._first_log_done:
                self._first_log_done.add(f"{cam_name}_{stream}_captured")
                
        except Exception as e:
            print(f"❌ Error processing image from {cam_name}_{stream}: {e}")
            import traceback
            traceback.print_exc()
    
    def camera_info_callback(self, cam_name: str, stream: str, msg: CameraInfo):
        """Callback for camera info messages."""
        try:
            # Store camera info
            with self._data_locks[cam_name]:
                self._camera_info_data[cam_name][stream] = {
                    'width': msg.width,
                    'height': msg.height,
                    'distortion_model': msg.distortion_model,
                    'D': list(msg.d),
                    'K': list(msg.k),
                    'R': list(msg.r),
                    'P': list(msg.p)
                }
                
        except Exception as e:
            print(f"Error processing camera info from {cam_name}_{stream}: {e}")
    
    def get_images(self) -> Dict[str, np.ndarray]:
        """Get current images from all cameras."""
        images = {}
        
        for cam_name in self.camera_names:
            with self._data_locks[cam_name]:
                if self.stereo_mode:
                    # Return both left and right images
                    left_img = self._image_data[cam_name]['left']
                    right_img = self._image_data[cam_name]['right']
                    
                    if left_img is not None:
                        images[f'{cam_name}_left'] = left_img
                    if right_img is not None:
                        images[f'{cam_name}_right'] = right_img
                else:
                    # Return only left image
                    left_img = self._image_data[cam_name]['left']
                    if left_img is not None:
                        images[cam_name] = left_img
        
        return images
    
    def get_timestamps(self) -> Dict[str, float]:
        """Get current timestamps from all cameras."""
        timestamps = {}
        
        for cam_name in self.camera_names:
            with self._data_locks[cam_name]:
                if self.stereo_mode:
                    # Return timestamps for both streams
                    left_ts = self._timestamp_data[cam_name]['left']
                    right_ts = self._timestamp_data[cam_name]['right']
                    
                    if left_ts is not None:
                        timestamps[f'{cam_name}_left'] = left_ts
                    if right_ts is not None:
                        timestamps[f'{cam_name}_right'] = right_ts
                else:
                    # Return only left timestamp
                    left_ts = self._timestamp_data[cam_name]['left']
                    if left_ts is not None:
                        timestamps[cam_name] = left_ts
        
        return timestamps
    
    def get_camera_info(self) -> Dict[str, Dict]:
        """Get camera information for all cameras."""
        return self._camera_info_data.copy()
    
    def wait_until_ready(self, required_names=None, timeout_sec: float = 2.0):
        """Block until required cameras have received at least one frame or timeout."""
        if required_names is None:
            required_names = self.camera_names
        
        pending = set(required_names)
        start = time.time()
        
        while pending and (time.time() - start) < timeout_sec:
            for name in list(pending):
                with self._data_locks[name]:
                    # Only check left image since that's what we use for recording
                    left_image = self._image_data[name]['left']
                    if left_image is not None:
                        pending.discard(name)
            
            # Spin the node to process callbacks more aggressively
            if self.node and rclpy is not None:
                try:
                    # Spin multiple times to ensure we process all available messages
                    for _ in range(5):
                        rclpy.spin_once(self.node, timeout_sec=0.02)
                except Exception as e:
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
        
        if pending:
            print(f"❌ RealSense camera timeout - still pending: {sorted(list(pending))}")
            # Print diagnostic information only on error
            for name in pending:
                with self._data_locks[name]:
                    left_img = self._image_data[name]['left']
                    print(f"  {name}: left image = {left_img is not None}")
        # Remove success message to reduce verbosity
    
    def validate_synchronization(self):
        """Validate that all camera timestamps are synchronized."""
        if not self.time_sync_manager:
            return True, {"warning": "Time sync manager not available"}
        
        timestamps = self.get_timestamps()
        if not timestamps:
            return False, {"error": "No timestamps available"}
        
        return self.time_sync_manager.validate_timestamps(timestamps)
    
    def print_diagnostics(self):
        """Print diagnostic information about camera performance."""
        for cam_name in self.camera_names:
            with self._data_locks[cam_name]:
                left_ts = self._timestamp_data[cam_name]['left']
                if left_ts is not None:
                    print(f'{cam_name}: Last capture at {left_ts:.6f}')
                else:
                    print(f'{cam_name}: No data')
        print()
    
    def shutdown(self):
        """Cleanup resources."""
        print("Shutting down RealSense ROS2 recorder...")
        
        # Don't cleanup the node - it's owned by the caller
        # Just clear our references
        
        # Clear data references
        self._image_data.clear()
        self._timestamp_data.clear()
        self._camera_info_data.clear()
        self._data_locks.clear()
        
        print("RealSense ROS2 recorder shutdown complete")


class RealSenseCameraResourceManager:
    """
    Singleton resource manager for RealSense cameras to handle reuse across episodes.
    This helps prevent camera initialization issues when recording multiple episodes.
    """
    _instance = None
    _cameras_initialized = False
    _camera_names = None
    _recorder = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(RealSenseCameraResourceManager, cls).__new__(cls)
        return cls._instance
    
    def initialize_cameras(self, camera_names: List[str], node=None, stereo_mode=False, task_name: str = "", use_system_time: bool = False):
        """Initialize cameras if not already done."""
        if self._cameras_initialized:
            return True
        
        try:
            self._camera_names = camera_names
            self.stereo_mode = stereo_mode
            
            # Get use_system_time from task config if not explicitly provided
            if not use_system_time and task_name:
                try:
                    from utils.config_loader import build_runtime_config
                    runtime_config = build_runtime_config(task_name)
                    use_system_time = runtime_config.get('collection', {}).get('use_system_time', False)
                except Exception:
                    pass
            
            # Create recorder with proper error handling
            self._recorder = RealSenseROS2Recorder(
                camera_names=camera_names,
                node=node,
                stereo_mode=stereo_mode,
                task_name=task_name,
                use_system_time=use_system_time
            )
            
            # Wait a bit for subscriptions to be established
            time.sleep(2.0)
            
            self._cameras_initialized = True
            print("✅ RealSense cameras initialized successfully")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize RealSense cameras: {e}")
            import traceback
            traceback.print_exc()
            self.cleanup_cameras()
            return False
    
    def get_images(self) -> Dict[str, np.ndarray]:
        """Get current images from all cameras."""
        if self._recorder:
            return self._recorder.get_images()
        return {}
    
    def get_timestamps(self) -> Dict[str, float]:
        """Get current timestamps from all cameras."""
        if self._recorder:
            return self._recorder.get_timestamps()
        return {}
    
    def get_camera_info(self) -> Dict[str, Dict]:
        """Get camera information for all cameras."""
        if self._recorder:
            return self._recorder.get_camera_info()
        return {}
    
    def wait_until_ready(self, required_names=None, timeout_sec: float = 3.0):
        """Block until required cameras have received at least one frame or timeout."""
        if self._recorder:
            return self._recorder.wait_until_ready(required_names, timeout_sec)
    
    def cleanup_cameras(self):
        """Cleanup camera resources."""
        print("Cleaning up RealSense camera resources...")
        
        if self._recorder:
            self._recorder.shutdown()
            self._recorder = None
        
        # Reset state
        self._cameras_initialized = False
        self._camera_names = None
        
        print("RealSense camera cleanup complete")


def detect_realsense_cameras() -> List[str]:
    """
    Detect available RealSense cameras by checking ROS2 topics.
    
    Returns:
        List of detected camera names
    """
    if rclpy is None:
        print("ROS2 not available for camera detection")
        return []
    
    try:
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init(args=None)
        
        # Create a temporary node to check topics
        temp_node = rclpy.create_node('camera_detector')
        
        # Get list of available topics
        topic_names_and_types = temp_node.get_topic_names_and_types()
        
        # Look for RealSense camera topics
        detected_cameras = []
        for topic_name, topic_types in topic_names_and_types:
            if '/camera/color/image_raw' in topic_name or '/camera/color/image_rect_raw' in topic_name:
                # Extract camera name from topic
                parts = topic_name.split('/')
                if len(parts) >= 4:  # /camera/d455_camera/color/image_raw has 4 parts
                    # For RealSense D455, the topic structure is /camera/d455_camera/color/image_raw
                    # For RealSense D405, the topic structure is /camera_left_wrist/d405_left_camera/color/image_rect_raw
                    if parts[1] == 'camera' and parts[2] == 'd455_camera':
                        camera_name = 'cam_top'  # Map to standard camera name
                        if camera_name not in detected_cameras:
                            detected_cameras.append(camera_name)
                    elif 'camera_left_wrist' in topic_name and 'd405_left_camera' in topic_name:
                        camera_name = 'cam_left_wrist'  # Map to standard camera name
                        if camera_name not in detected_cameras:
                            detected_cameras.append(camera_name)
                    elif 'camera_right_wrist' in topic_name and 'd405_right_camera' in topic_name:
                        camera_name = 'cam_right_wrist'  # Map to standard camera name
                        if camera_name not in detected_cameras:
                            detected_cameras.append(camera_name)
        
        # Clean up temporary node
        temp_node.destroy_node()
        
        if detected_cameras:
            print(f"✅ Detected {len(detected_cameras)} RealSense camera(s): {detected_cameras}")
        return detected_cameras
        
    except Exception as e:
        print(f"Error detecting RealSense cameras: {e}")
        return []


def check_camera_availability(camera_names: List[str], timeout_sec: float = 5.0) -> bool:
    """
    Check if RealSense cameras are available by checking topic publishing.
    
    Args:
        camera_names: List of camera names to check
        timeout_sec: Timeout for camera availability check
        
    Returns:
        True if all cameras are available, False otherwise
    """
    if rclpy is None:
        print("ROS2 not available for camera availability check")
        return False
    
    if not camera_names:
        return True
    
    # Skip verbose availability check message
    
    try:
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init(args=None)
        
        # Create a temporary node to check topics
        temp_node = rclpy.create_node('camera_availability_checker')
        
        # Check each camera
        for cam_name in camera_names:
            # Map camera names to actual RealSense topics
            if cam_name == 'cam_top':
                topic_name = "/camera/d455_camera/color/image_raw"
            elif cam_name == 'cam_left_wrist':
                topic_name = "/camera_left_wrist/d405_left_camera/color/image_rect_raw"
            elif cam_name == 'cam_right_wrist':
                topic_name = "/camera_right_wrist/d405_right_camera/color/image_rect_raw"
            else:
                topic_name = f"/{cam_name}/camera/color/image_raw"
            
            # Create a subscription to check if topic is publishing
            received_message = False
            
            def test_callback(msg):
                nonlocal received_message
                received_message = True
            
            try:
                subscription = temp_node.create_subscription(
                    Image, topic_name, test_callback, 1
                )
                
                # Wait for a message or timeout
                start_time = time.time()
                while not received_message and (time.time() - start_time) < timeout_sec:
                    rclpy.spin_once(temp_node, timeout_sec=0.1)
                
                if not received_message:
                    print(f"❌ Camera {cam_name} is not publishing on {topic_name}")
                    temp_node.destroy_node()
                    return False
                    
            except Exception as e:
                print(f"Error checking camera {cam_name}: {e}")
                temp_node.destroy_node()
                return False
        
        # Clean up temporary node
        temp_node.destroy_node()
        
        # Skip success message to reduce verbosity
        return True
        
    except Exception as e:
        print(f"Error during camera availability check: {e}")
        return False
