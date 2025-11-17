import time,sys,os
import numpy as np
import collections
import matplotlib.pyplot as plt
import dm_env
from pyquaternion import Quaternion
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
from recorders.arm_recorder import ArmRecorder
# Legacy image recorder moved to legacy/ directory
# from .recorders.image_recorder import ImageRecorder
from recorders.linkerhand_recorder import LinkerHandRecorder
# Import ExHand recorder for exoskeleton glove support
try:
    from recorders.exhand_recorder import ExHandRecorder
    EXHAND_AVAILABLE = True
except ImportError as e:
    print(f"Warning: ExHand recorder not available: {e}")
    EXHAND_AVAILABLE = False
    ExHandRecorder = None
# Import PiperRecorder adapter for Piper arm support
try:
    from recorders.piper_recorder_adapter import PiperRecorderAdapter
    PIPER_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Piper recorder adapter not available: {e}")
    PIPER_AVAILABLE = False
    PiperRecorderAdapter = None
# Legacy camera info recorder moved to legacy/ directory
# from recorders.camera_info_recorder import CameraInfoRecorder
import sys
import os

from utils.config_loader import (
    LEFT_HAND_JOINT,
    RIGHT_HAND_JOINT,
    build_runtime_config,
)

DT = 0.04  # Default value, can be overridden by task config
import IPython
e = IPython.embed

class RealEnv:
    def __init__(self, camera_names=[], setup_left_arm=True, setup_right_arm=False, setup_left_hand=True, setup_right_hand=False, node=None, stereo_mode=False, left_arm_dof=None, right_arm_dof=None, left_hand_dof=None, right_hand_dof=None, task_name: str = "", dt: float = None, max_timestamp_diff: float = None, use_system_time: bool = None):
        """
        Initialize RealEnv with configurable DoF for arms and hands.
        
        Args:
            left_arm_dof: Left arm degrees of freedom (default: from runtime config)
            right_arm_dof: Right arm degrees of freedom (default: from runtime config)
            left_hand_dof: Left hand degrees of freedom (default: from runtime config)
            right_hand_dof: Right hand degrees of freedom (default: from runtime config)
            dt: Time step duration in seconds (default: from task config or 0.04)
            max_timestamp_diff: Maximum timestamp difference for sync validation (from task config)
            use_system_time: Force using system time instead of ROS time (from task config)
        """
        # Store joint counts (use provided values or defaults from runtime config)
        runtime_defaults = build_runtime_config(task_name) if task_name else None
        default_arm = runtime_defaults.get('hardware', {}).get('arm_joints', {}) if runtime_defaults else {}
        default_hand = runtime_defaults.get('hardware', {}).get('hand_joints', {}) if runtime_defaults else {}
        
        # Set dt from parameter, task config, or default
        if dt is not None:
            self.dt = dt
        elif runtime_defaults:
            self.dt = runtime_defaults.get('collection', {}).get('dt', DT)
        else:
            self.dt = DT
        
        # Set max_timestamp_diff from parameter, task config, or default
        if max_timestamp_diff is not None:
            self.max_timestamp_diff = max_timestamp_diff
        elif runtime_defaults:
            self.max_timestamp_diff = runtime_defaults.get('collection', {}).get('max_timestamp_diff', 0.1)
        else:
            self.max_timestamp_diff = 0.1
        
        # Set use_system_time from parameter, task config, or default
        if use_system_time is not None:
            self.use_system_time = use_system_time
        elif runtime_defaults:
            self.use_system_time = runtime_defaults.get('collection', {}).get('use_system_time', False)
        else:
            self.use_system_time = False

        self.left_arm_joint = left_arm_dof if left_arm_dof is not None else default_arm.get('left_arm_dof', 7)
        self.right_arm_joint = right_arm_dof if right_arm_dof is not None else default_arm.get('right_arm_dof', 7)
        self.left_hand_joint = left_hand_dof if left_hand_dof is not None else LEFT_HAND_JOINT
        self.right_hand_joint = right_hand_dof if right_hand_dof is not None else RIGHT_HAND_JOINT
        
        # Print configuration for verification
        print(f"RealEnv配置: 左臂{self.left_arm_joint}DoF, 右臂{self.right_arm_joint}DoF, 左手{self.left_hand_joint}DoF, 右手{self.right_hand_joint}DoF (总计{self.left_arm_joint + self.right_arm_joint + self.left_hand_joint + self.right_hand_joint}DoF)")
        
        # Initialize time synchronization with configured tolerance and time source
        try:
            from time_sync import TimeSyncManager
            # Use tolerance and time source from task config
            self.time_sync_manager = TimeSyncManager(
                max_timestamp_diff=self.max_timestamp_diff,
                use_system_time=self.use_system_time
            )
            time_source = "系统时间" if self.use_system_time else "ROS2时间"
            print(f"时间同步已初始化 (容差: {self.max_timestamp_diff*1000}ms, 时间源: {time_source})")
        except ImportError:
            self.time_sync_manager = None
            # Only print warning if time sync is critical
        
        # Arm recorders - choose between Piper SDK and ROS2 topics based on task
        self.recorder_left = None
        self.recorder_right = None
        
        # Determine if this is a Piper task
        runtime_defaults = build_runtime_config(task_name) if task_name else None
        hardware_preset = runtime_defaults.get('hardware', {}).get('preset', '') if runtime_defaults else ''
        is_piper_task = 'piper' in hardware_preset.lower() or 'piper' in task_name.lower()
        
        if setup_left_arm:
            try:
                if is_piper_task and PIPER_AVAILABLE:
                    # Use Piper SDK recorder
                    self.recorder_left = PiperRecorderAdapter('left')
                    print(f"✓ Using Piper SDK recorder for left arm")
                else:
                    # Use ROS2 topic-based recorder
                    self.recorder_left = ArmRecorder('left', node=node, task_name=task_name)
            except Exception as e:
                print(f"❌ Warning: Could not initialize left arm recorder: {e}")
                self.recorder_left = None
        
        if setup_right_arm:
            try:
                if is_piper_task and PIPER_AVAILABLE:
                    # Use Piper SDK recorder
                    self.recorder_right = PiperRecorderAdapter('right')
                    print(f"✓ Using Piper SDK recorder for right arm")
                else:
                    # Use ROS2 topic-based recorder
                    self.recorder_right = ArmRecorder('right', node=node, task_name=task_name)
            except Exception as e:
                print(f"❌ Warning: Could not initialize right arm recorder: {e}")
                self.recorder_right = None
        
        # LinkerHand recorders
        self.left_hand_recorder = None
        self.right_hand_recorder = None
        
        # ExHand recorders (exoskeleton glove)
        self.left_exhand_recorder = None
        self.right_exhand_recorder = None
        
        # Get tactile collection setting from runtime config (controlled by task_config.json)
        runtime_defaults = build_runtime_config(task_name) if task_name else None
        collect_tactile = runtime_defaults.get('hardware', {}).get('collect_tactile', False) if runtime_defaults else False
        
        # Check if ExHand should be used (from task config or hardware preset)
        use_exhand = runtime_defaults.get('hardware', {}).get('use_exhand', False) if runtime_defaults else False
        hardware_preset = runtime_defaults.get('hardware', {}).get('preset', '') if runtime_defaults else ''
        # Auto-detect if exhand is mentioned in preset or task name
        if not use_exhand:
            use_exhand = 'exhand' in hardware_preset.lower() or 'exhand' in task_name.lower()
        
        if setup_left_hand:
            try:
                if use_exhand and EXHAND_AVAILABLE:
                    # Use ExHand recorder for exoskeleton glove data
                    self.left_exhand_recorder = ExHandRecorder('left', node=node, task_name=task_name)
                    print(f"✓ Using ExHand recorder for left hand")
                else:
                    # Use LinkerHand recorder
                    self.left_hand_recorder = LinkerHandRecorder(
                        'left', node=node, enable_tactile=collect_tactile, task_name=task_name
                    )
            except Exception as e:
                print(f"❌ Warning: Could not initialize left hand recorder: {e}")
                self.left_hand_recorder = None
                self.left_exhand_recorder = None
        
        if setup_right_hand:
            try:
                if use_exhand and EXHAND_AVAILABLE:
                    # Use ExHand recorder for exoskeleton glove data
                    self.right_exhand_recorder = ExHandRecorder('right', node=node, task_name=task_name)
                    print(f"✓ Using ExHand recorder for right hand")
                else:
                    # Use LinkerHand recorder
                    self.right_hand_recorder = LinkerHandRecorder(
                        'right', node=node, enable_tactile=collect_tactile, task_name=task_name
                    )
            except Exception as e:
                print(f"❌ Warning: Could not initialize right hand recorder: {e}")
                self.right_hand_recorder = None
                self.right_exhand_recorder = None
        

        
        # Cameras - Intel RealSense ROS2 Topics
        self.stereo_mode = stereo_mode
        
        # Use Intel RealSense cameras via ROS2 topics
        try:
            from recorders.realsense_ros2_recorder import RealSenseROS2Recorder, RealSenseCameraResourceManager, detect_realsense_cameras, check_camera_availability
            
            # Get or create camera resource manager (singleton)
            self.camera_resource_manager = RealSenseCameraResourceManager()
            
            # Check if cameras are already initialized
            if self.camera_resource_manager._cameras_initialized:
                # Skip verbose output when cameras are already running
                self.image_recorder = self._create_image_recorder_wrapper()
            else:
                # Initialize cameras for the first time
                if camera_names:
                    camera_list = camera_names
                elif runtime_defaults:
                    cameras_config = runtime_defaults.get('cameras', {})
                    enable_flags = {
                        'cam_top': cameras_config.get('enable_top_camera'),
                        'cam_left_wrist': cameras_config.get('enable_left_wrist_camera'),
                        'cam_right_wrist': cameras_config.get('enable_right_wrist_camera'),
                    }

                    preset_name = cameras_config.get('preset')
                    preset_camera_list = self._camera_list_from_preset(runtime_defaults, preset_name)

                    if any(flag is True for flag in enable_flags.values() if flag is not None):
                        topic_map = cameras_config.get('topic_map') or {}
                        camera_list = [
                            cam for cam, enabled in enable_flags.items()
                            if enabled is True and topic_map.get(cam)
                        ]
                    else:
                        camera_list = preset_camera_list

                    if not camera_list:
                        camera_list = preset_camera_list

                    if not camera_list:
                        raise RuntimeError(
                            "No cameras selected. Please check camera enable flags and preset configuration."
                        )
                else:
                    detected_cameras = detect_realsense_cameras()
                    if detected_cameras:
                        camera_list = detected_cameras
                    else:
                        raise RuntimeError(
                            "No RealSense cameras detected. Please ensure RealSense cameras are connected and ROS2 topics are publishing."
                        )
                
                # Initialize cameras for the first time
                # Ensure we have a node to pass to the recorder
                if node is None:
                    raise RuntimeError("RealSense recorder requires a ROS2 node. Please ensure the main service node is properly initialized.")
                
                success = self.camera_resource_manager.initialize_cameras(
                    camera_list, 
                    node=node, 
                    stereo_mode=stereo_mode, 
                    task_name=task_name,
                    use_system_time=self.use_system_time
                )

                if success:
                    print("Using RealSense ROS2 resource manager for image capture")
                    # Create a wrapper that uses the resource manager
                    self.image_recorder = self._create_image_recorder_wrapper()
                else:
                    raise RuntimeError("Failed to initialize RealSense camera resource manager")

        except ImportError as e:
            raise ImportError(f"RealSense ROS2 recorder not available: {e}. Please ensure ROS2 and realsense2_camera packages are installed.")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize RealSense ROS2 recorder: {e}. Please ensure RealSense cameras are connected and ROS2 topics are publishing.")

        # Camera info is provided by RealSense ROS2 recorder
        self.camera_info_recorder = None

        # Wait for cameras to be ready (only if not already initialized)
        if camera_names and not self.camera_resource_manager._cameras_initialized:
            try:
                # Wait with reduced timeout for faster startup
                success = self.image_recorder.wait_until_ready(timeout_sec=5.0)
                if not success:
                    print("Warning: Cameras not fully ready, but continuing...")
            except Exception as e:
                print(f"Warning: Camera ready check failed: {e}")
                # Continue anyway - don't block the entire system


    def _camera_list_from_preset(self, runtime_defaults, preset_name: str) -> list:
        """Resolve camera names from preset definition in runtime config."""
        if not preset_name:
            return []

        preset_config = (runtime_defaults or {}).get('cameras', {})
        preset_cameras = preset_config.get('cameras', []) if preset_config.get('preset') == preset_name else []

        if not preset_cameras:
            try:
                from utils.config_loader import ConfigLoader
                preset_details = ConfigLoader().get_camera_preset(preset_name)
                preset_cameras = preset_details.get('cameras', [])
            except Exception as e:
                print(f"Warning: Could not load camera preset '{preset_name}': {e}")
                preset_cameras = []

        return [cam.get('name') for cam in preset_cameras if cam.get('name')]


    def get_qpos(self):
        """Get concatenated qpos: [left_arm, right_arm, left_hand, right_hand] with configured DoF."""
        # Arms - using arm recorder methods
        left_arm_qpos = self.recorder_left.get_qpos() if self.recorder_left else [0.0] * self.left_arm_joint
        right_arm_qpos = self.recorder_right.get_qpos() if self.recorder_right else [0.0] * self.right_arm_joint
        # Hands - support both LinkerHand and ExHand recorders
        if self.left_exhand_recorder:
            # Use ExHand mapping data (15 float values, normalized 0-1)
            left_hand_qpos = self.left_exhand_recorder.get_mapping_data()
            # Pad or truncate to match expected DoF
            if len(left_hand_qpos) < self.left_hand_joint:
                left_hand_qpos = list(left_hand_qpos) + [0.0] * (self.left_hand_joint - len(left_hand_qpos))
            else:
                left_hand_qpos = left_hand_qpos[:self.left_hand_joint]
        else:
            left_hand_qpos = self.left_hand_recorder.get_qpos() if self.left_hand_recorder else [0.0] * self.left_hand_joint
        
        if self.right_exhand_recorder:
            # Use ExHand mapping data (15 float values, normalized 0-1)
            right_hand_qpos = self.right_exhand_recorder.get_mapping_data()
            # Pad or truncate to match expected DoF
            if len(right_hand_qpos) < self.right_hand_joint:
                right_hand_qpos = list(right_hand_qpos) + [0.0] * (self.right_hand_joint - len(right_hand_qpos))
            else:
                right_hand_qpos = right_hand_qpos[:self.right_hand_joint]
        else:
            right_hand_qpos = self.right_hand_recorder.get_qpos() if self.right_hand_recorder else [0.0] * self.right_hand_joint
        
        return np.concatenate([left_arm_qpos, right_arm_qpos, left_hand_qpos, right_hand_qpos])

    def get_qvel(self):
        # Arms - using arm recorder methods
        left_arm_qvel = self.recorder_left.get_qvel() if self.recorder_left else [0.0] * self.left_arm_joint
        right_arm_qvel = self.recorder_right.get_qvel() if self.recorder_right else [0.0] * self.right_arm_joint
        # Hands - ExHand doesn't provide velocity, use zeros
        if self.left_exhand_recorder:
            left_hand_qvel = [0.0] * self.left_hand_joint
        else:
            left_hand_qvel = self.left_hand_recorder.get_qvel() if self.left_hand_recorder else [0.0] * self.left_hand_joint
        
        if self.right_exhand_recorder:
            right_hand_qvel = [0.0] * self.right_hand_joint
        else:
            right_hand_qvel = self.right_hand_recorder.get_qvel() if self.right_hand_recorder else [0.0] * self.right_hand_joint
        
        return np.concatenate([left_arm_qvel, right_arm_qvel, left_hand_qvel, right_hand_qvel])

    def get_effort(self):
        # Arms - using arm recorder methods
        left_arm_effort = self.recorder_left.get_effort() if self.recorder_left else [0.0] * self.left_arm_joint
        right_arm_effort = self.recorder_right.get_effort() if self.recorder_right else [0.0] * self.right_arm_joint
        # Hands - ExHand doesn't provide effort, use zeros
        if self.left_exhand_recorder:
            left_hand_effort = [0.0] * self.left_hand_joint
        else:
            left_hand_effort = self.left_hand_recorder.get_effort() if self.left_hand_recorder else [0.0] * self.left_hand_joint
        
        if self.right_exhand_recorder:
            right_hand_effort = [0.0] * self.right_hand_joint
        else:
            right_hand_effort = self.right_hand_recorder.get_effort() if self.right_hand_recorder else [0.0] * self.right_hand_joint
        
        return np.concatenate([left_arm_effort, right_arm_effort, left_hand_effort, right_hand_effort])
    
    def get_action(self):
        """Get current teleop actions: arms (from command positions) + hands (from last commanded positions)."""
        total_len = self.left_arm_joint + self.right_arm_joint + self.left_hand_joint + self.right_hand_joint
        action = np.zeros(total_len)
        # Arms - using arm recorder command position methods
        action[:self.left_arm_joint] = (
            self.recorder_left.get_cmd_pos()
            if self.recorder_left else [0.0] * self.left_arm_joint
        )
        arm_right_start = self.left_arm_joint
        action[arm_right_start:arm_right_start+self.right_arm_joint] = (
            self.recorder_right.get_cmd_pos()
            if self.recorder_right else [0.0] * self.right_arm_joint
        )
        # Hands from last command positions - support both LinkerHand and ExHand
        hand_left_start = self.left_arm_joint + self.right_arm_joint
        if self.left_exhand_recorder:
            # Use ExHand mapping data as action (15 float values)
            left_hand_action = self.left_exhand_recorder.get_mapping_data()
            if len(left_hand_action) < self.left_hand_joint:
                left_hand_action = list(left_hand_action) + [0.0] * (self.left_hand_joint - len(left_hand_action))
            else:
                left_hand_action = left_hand_action[:self.left_hand_joint]
            action[hand_left_start:hand_left_start+self.left_hand_joint] = left_hand_action
        else:
            action[hand_left_start:hand_left_start+self.left_hand_joint] = (
                self.left_hand_recorder.get_cmd_pos() if self.left_hand_recorder else [0.0] * self.left_hand_joint
            )
        
        hand_right_start = hand_left_start + self.left_hand_joint
        if self.right_exhand_recorder:
            # Use ExHand mapping data as action (15 float values)
            right_hand_action = self.right_exhand_recorder.get_mapping_data()
            if len(right_hand_action) < self.right_hand_joint:
                right_hand_action = list(right_hand_action) + [0.0] * (self.right_hand_joint - len(right_hand_action))
            else:
                right_hand_action = right_hand_action[:self.right_hand_joint]
            action[hand_right_start:hand_right_start+self.right_hand_joint] = right_hand_action
        else:
            action[hand_right_start:hand_right_start+self.right_hand_joint] = (
                self.right_hand_recorder.get_cmd_pos() if self.right_hand_recorder else [0.0] * self.right_hand_joint
            )
        return action
    
    def get_tactile_data(self):
        """Get tactile data from both hands.
        
        Returns:
            Dict containing tactile data for left and right hands
        """
        tactile_data = {}
        
        # Left hand tactile data
        if self.left_hand_recorder:
            tactile_data['left_hand'] = self.left_hand_recorder.get_tactile_data()
        else:
            # Return zero matrices if recorder not available
            tactile_data['left_hand'] = {
                'thumb_matrix': np.zeros((12, 6), dtype=np.float32),
                'index_matrix': np.zeros((12, 6), dtype=np.float32),
                'middle_matrix': np.zeros((12, 6), dtype=np.float32),
                'ring_matrix': np.zeros((12, 6), dtype=np.float32),
                'little_matrix': np.zeros((12, 6), dtype=np.float32)
            }
        
        # Right hand tactile data
        if self.right_hand_recorder:
            tactile_data['right_hand'] = self.right_hand_recorder.get_tactile_data()
        else:
            # Return zero matrices if recorder not available
            tactile_data['right_hand'] = {
                'thumb_matrix': np.zeros((12, 6), dtype=np.float32),
                'index_matrix': np.zeros((12, 6), dtype=np.float32),
                'middle_matrix': np.zeros((12, 6), dtype=np.float32),
                'ring_matrix': np.zeros((12, 6), dtype=np.float32),
                'little_matrix': np.zeros((12, 6), dtype=np.float32)
            }
        
        return tactile_data
    
    def get_observation(self):
        obs = collections.OrderedDict()
        obs['qpos'] = self.get_qpos()
        obs['qvel'] = self.get_qvel()
        obs['effort'] = self.get_effort()
        # obs['end_pos'] = self.get_end_pose()  # Commented out: end_pos recording disabled
        obs['images'] = self.image_recorder.get_images()
        
        # Add tactile data
        obs['tactile'] = self.get_tactile_data()
        
        # Add camera_info to observation (RealSense ROS2 only)
        if hasattr(self.image_recorder, 'get_camera_info'):
            obs['camera_info'] = self.image_recorder.get_camera_info()
        else:
            obs['camera_info'] = {}
        
        # Add timestamp validation if time sync manager is available
        if self.time_sync_manager and hasattr(self, 'image_recorder'):
            # Get camera timestamps (these should be from ROS message headers)
            camera_timestamps = self.image_recorder.get_timestamps()
            
            # Get joint state timestamps from actual data sources
            all_timestamps = camera_timestamps.copy()
            
            # Add arm timestamps (from ROS message headers if available)
            if self.recorder_left and hasattr(self.recorder_left, 'get_representative_timestamp'):
                left_arm_ts = self.recorder_left.get_representative_timestamp()
                all_timestamps['left_arm'] = left_arm_ts
            if self.recorder_right and hasattr(self.recorder_right, 'get_representative_timestamp'):
                right_arm_ts = self.recorder_right.get_representative_timestamp()
                all_timestamps['right_arm'] = right_arm_ts
            
            # Add hand timestamps (from ROS message headers)
            if self.left_hand_recorder:
                left_hand_ts = self.left_hand_recorder.get_representative_timestamp()
                all_timestamps['left_hand'] = left_hand_ts
            if self.right_hand_recorder:
                right_hand_ts = self.right_hand_recorder.get_representative_timestamp()
                all_timestamps['right_hand'] = right_hand_ts
            
            # Validate synchronization
            is_valid, validation_info = self.time_sync_manager.validate_timestamps(all_timestamps)
            
            # Store validation info in observation
            obs['timestamp_validation'] = {
                'is_valid': is_valid,
                'info': validation_info,
                'timestamps': all_timestamps
            }
            
            # Log synchronization status (only if invalid to avoid spam)
            if not is_valid:
                from time_sync import log_synchronization_info
                log_synchronization_info(all_timestamps, (is_valid, validation_info))
                # Continue anyway - don't block data collection
                print("⚠️  Continuing data collection despite sync issues...")
        
        return obs
    
    ## End pos collection (not used in training), kept for future applications
    # def get_end_pose(self):
    #     """Concatenate [left_arm_ee(6), right_arm_ee(6), left_hand_qpos(10), right_hand_qpos(10)]."""
    #     left_end_pose = self.recorder_left.get_end_pose() if self.recorder_left else [0.0] * 6
    #     right_end_pose = self.recorder_right.get_end_pose() if self.recorder_right else [0.0] * 6
    #     left_hand_qpos = self.left_hand_recorder.get_qpos() if self.left_hand_recorder else [0.0] * LEFT_HAND_JOINT
    #     right_hand_qpos = self.right_hand_recorder.get_qpos() if self.right_hand_recorder else [0.0] * RIGHT_HAND_JOINT
    #     return np.concatenate([left_end_pose, right_end_pose, left_hand_qpos, right_hand_qpos])
    
    # def get_piper_master_end_pose(self):
    #     """Kept for compatibility: returns only arm end-poses (12), hands omitted."""
    #     master_end_pose = np.zeros(LEFT_ARM_JOINT + RIGHT_ARM_JOINT)
    #     if self.recorder_left is not None:
    #         master_end_pose[:LEFT_ARM_JOINT] = self.recorder_left.get_master_end_pose()
    #     if self.recorder_right is not None:
    #         start = LEFT_ARM_JOINT
    #         master_end_pose[start:start+RIGHT_ARM_JOINT] = self.recorder_right.get_master_end_pose()
    #     return master_end_pose


    def step(self, action):
        # This env is read-only for data capture; no actuation is performed here.
        time.sleep(self.dt)
        return dm_env.TimeStep(
            step_type=dm_env.StepType.MID,
            reward=0,
            discount=None,
            observation=self.get_observation())
    
    def reset(self, fake=False):
        if not fake:
            # Reboot puppet robot gripper motors
            pass
        return dm_env.TimeStep(
            step_type=dm_env.StepType.FIRST,
            reward=0,
            discount=None,
            observation=self.get_observation())
    
    def cleanup(self):
        """Clean up resources and properly close cameras."""
        print("Cleaning up RealEnv resources...")
        
        # Don't shutdown RealSense cameras here - they're managed by the resource manager
        # and should persist across episodes to avoid reinitialization issues
        if hasattr(self, 'image_recorder') and self.image_recorder:
            try:
                # Just call the wrapper's shutdown method (which does nothing)
                self.image_recorder.shutdown()
                print("RealSense camera recorder wrapper cleaned up")
            except Exception as e:
                print(f"Error cleaning up RealSense camera recorder wrapper: {e}")
        
        # Cleanup hand recorders
        if hasattr(self, 'left_hand_recorder') and self.left_hand_recorder:
            try:
                # Add cleanup method to LinkerHandRecorder if needed
                pass
            except Exception as e:
                print(f"Error cleaning up left hand recorder: {e}")
        
        if hasattr(self, 'right_hand_recorder') and self.right_hand_recorder:
            try:
                # Add cleanup method to LinkerHandRecorder if needed
                pass
            except Exception as e:
                print(f"Error cleaning up right hand recorder: {e}")
        
        # Cleanup arm recorders
        if hasattr(self, 'recorder_left') and self.recorder_left:
            try:
                # Add cleanup method to ArmRecorder if needed
                pass
            except Exception as e:
                print(f"Error cleaning up left arm recorder: {e}")
        
        if hasattr(self, 'recorder_right') and self.recorder_right:
            try:
                # Add cleanup method to ArmRecorder if needed
                pass
            except Exception as e:
                print(f"Error cleaning up right arm recorder: {e}")
        
        print("RealEnv cleanup complete")
    
    def _create_image_recorder_wrapper(self):
        """Create a wrapper object that uses the camera resource manager."""
        class ImageRecorderWrapper:
            def __init__(self, resource_manager):
                self.resource_manager = resource_manager
            
            def get_images(self):
                return self.resource_manager.get_images()
            
            def get_timestamps(self):
                return self.resource_manager.get_timestamps()
            
            def get_camera_info(self):
                return self.resource_manager.get_camera_info()
            
            def wait_until_ready(self, required_names=None, timeout_sec=5.0):
                return self.resource_manager.wait_until_ready(required_names, timeout_sec)
            
            def shutdown(self):
                # Don't shutdown the resource manager here - it's shared across episodes
                pass
        
        return ImageRecorderWrapper(self.camera_resource_manager)
    
def make_real_env(init_node, camera_names=[], setup_robots=True, setup_base=False, setup_left_arm=True, setup_right_arm=True, setup_left_hand=True, setup_right_hand=True, node=None, stereo_mode=False, left_arm_dof=None, right_arm_dof=None, left_hand_dof=None, right_hand_dof=None, task_name: str = "", dt: float = None, max_timestamp_diff: float = None, use_system_time: bool = None):
    """
    Create RealEnv with configurable DoF for arms and hands.
    
    Args:
        left_arm_dof: Left arm degrees of freedom (from task config)
        right_arm_dof: Right arm degrees of freedom (from task config)
        left_hand_dof: Left hand degrees of freedom (from task config or auto-detect)
        right_hand_dof: Right hand degrees of freedom (from task config or auto-detect)
        dt: Time step duration in seconds (from task config)
        max_timestamp_diff: Maximum timestamp difference for sync validation (from task config)
        use_system_time: Force using system time instead of ROS time (from task config)
    """
    env = RealEnv(
        camera_names=camera_names,
        setup_left_arm=setup_left_arm,
        setup_right_arm=setup_right_arm, 
        setup_left_hand=setup_left_hand,
        setup_right_hand=setup_right_hand,
        node=node,
        left_arm_dof=left_arm_dof,
        right_arm_dof=right_arm_dof,
        left_hand_dof=left_hand_dof,
        right_hand_dof=right_hand_dof,
        stereo_mode=stereo_mode,
        task_name=task_name,
        dt=dt,
        max_timestamp_diff=max_timestamp_diff,
        use_system_time=use_system_time
    )
    return env


def cleanup_realsense_cameras():
    """Global function to cleanup RealSense camera resources when data collection is complete."""
    try:
        from recorders.realsense_ros2_recorder import RealSenseCameraResourceManager
        resource_manager = RealSenseCameraResourceManager()
        resource_manager.cleanup_cameras()
        print("Global RealSense camera cleanup completed")
    except Exception as e:
        print(f"Error during global RealSense camera cleanup: {e}")