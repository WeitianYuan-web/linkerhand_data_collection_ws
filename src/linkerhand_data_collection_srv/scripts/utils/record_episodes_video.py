import os
import time
import h5py
import argparse
import numpy as np
from tqdm import tqdm
from pathlib import Path

from utils.config_loader import build_runtime_config
from recorders.episode_writer import EpisodeWriter
from real_env import make_real_env

import IPython
e = IPython.embed


def get_camera_resolution(camera_info_data, cam_name):
    """Extract camera resolution from camera_info data"""
    if not camera_info_data or cam_name not in camera_info_data:
        return None
    
    info = camera_info_data[cam_name]
    if 'height' in info and 'width' in info:
        return (info['height'], info['width'])
    else:
        return None


def capture_one_episode_video(dt, max_timesteps, camera_names, episode_dir, episode_name,
                             task_config=None, node=None, stereo_mode=False, stop_callback=None):
    """
    Capture one episode using the new video compression format.
    
    Args:
        dt: Time step duration
        max_timesteps: Maximum number of timesteps
        camera_names: List of camera names
        episode_dir: Directory to save episode data
        episode_name: Name for the episode (e.g., 'episode_001')
        task_config: Task configuration dictionary
        RealSense cameras are auto-detected via ROS2 topics
        node: ROS2 node
        stereo_mode: Whether to use stereo recording
        
    Returns:
        bool: True if successful, False otherwise
    """
    # Create episode directory
    episode_path = os.path.join(episode_dir, episode_name)
    
    runtime_config = task_config or {}
    cameras_config = runtime_config.get('cameras', {})
    camera_resolutions_map = cameras_config.get('camera_resolutions', {})
    video_encoding = cameras_config.get('video_encoding', {})
    # Get dt from task config, use parameter as fallback
    dt_value = runtime_config.get('collection', {}).get('dt', dt)
    # Get max_timestamp_diff from task config
    max_timestamp_diff = runtime_config.get('collection', {}).get('max_timestamp_diff', 0.1)
    # Get use_system_time from task config
    use_system_time = runtime_config.get('collection', {}).get('use_system_time', False)
    camera_resolutions = {}
    
    for cam_name in camera_names:
        if cam_name in camera_resolutions_map:
            camera_resolutions[cam_name] = camera_resolutions_map[cam_name]
        else:
            camera_resolutions[cam_name] = (640, 480)
    
    # Get video quality from runtime config
    video_quality = video_encoding.get('quality', 'medium')

    # Get arm DoF configuration
    arm_joint_config = runtime_config.get('hardware', {}).get('arm_joints', {})
    hand_joint_config = runtime_config.get('hardware', {}).get('hand_joints', {})
    
    # Determine DoF values
    left_arm_dof = arm_joint_config.get('left_arm_dof', 0)
    right_arm_dof = arm_joint_config.get('right_arm_dof', 0)
    left_hand_dof = hand_joint_config.get('left_hand_dof', None)
    right_hand_dof = hand_joint_config.get('right_hand_dof', None)
    
    print(f"任务配置: 左臂{left_arm_dof}DoF, 右臂{right_arm_dof}DoF, 左手{left_hand_dof}DoF, 右手{right_hand_dof}DoF")
    
    # Extract active_sides early for metadata
    active_sides = runtime_config.get('hardware', {}).get('active_sides', {})
    setup_left_arm = active_sides.get('left_arm', False)
    setup_right_arm = active_sides.get('right_arm', False)
    setup_left_hand = active_sides.get('left_hand', True)
    setup_right_hand = active_sides.get('right_hand', True)
    
    # Build comprehensive hardware configuration metadata
    hardware_config = {
        'preset': runtime_config.get('hardware', {}).get('preset', 'unknown'),
        'active_sides': {
            'left_arm': setup_left_arm,
            'right_arm': setup_right_arm,
            'left_hand': setup_left_hand,
            'right_hand': setup_right_hand
        },
        'arm_joints': {
            'left_arm_dof': left_arm_dof if setup_left_arm else 0,
            'right_arm_dof': right_arm_dof if setup_right_arm else 0
        },
        'hand_joints': {
            'left_hand_dof': left_hand_dof if setup_left_hand else 0,
            'right_hand_dof': right_hand_dof if setup_right_hand else 0
        },
        'total_dof': (left_arm_dof if setup_left_arm else 0) + \
                     (right_arm_dof if setup_right_arm else 0) + \
                     (left_hand_dof if setup_left_hand else 0) + \
                     (right_hand_dof if setup_right_hand else 0),
        'collect_tactile': runtime_config.get('hardware', {}).get('collect_tactile', False)
    }
    
    # Extract device information from config
    device_config = runtime_config.get('device', {})
    
    # Create episode writer
    with EpisodeWriter(
        episode_dir=episode_path,
        camera_configs={
            cam_name: {
                'width': camera_resolutions[cam_name][0],
                'height': camera_resolutions[cam_name][1],
                'fps': video_encoding.get('fps', 30),
                'quality': video_quality,
                'preset': video_encoding.get('preset', 'fast'),
                'pixel_format': video_encoding.get('pixel_format', 'rgb24')
            }
            for cam_name in camera_names
        },
        episode_metadata={
            'task_name': task_config.get('task_name', 'unknown') if task_config else 'unknown',
            'task_description': runtime_config.get('description', ''),
            'episode_length': max_timesteps,
            'dt': dt_value,  # Use the dt_value from task config
            'max_timestamp_diff': max_timestamp_diff,  # Time sync tolerance from task config
            'use_system_time': use_system_time,  # Time source from task config
            'camera_names': camera_names,
            'stereo_mode': stereo_mode,
            'hardware': hardware_config,
            'device': device_config,  # Device information (device_number, device_type, hardware_specs)
            'created_at': time.strftime('%Y-%m-%d %H:%M:%S')
        }
    ) as episode_writer:
        
        # Create environment with task-specific DoF configuration
        # (active_sides already extracted above for metadata)
        env = make_real_env(
            camera_names=camera_names, 
            init_node=False, 
            setup_robots=False, 
            node=node, 
            stereo_mode=stereo_mode,
            setup_left_arm=setup_left_arm,
            setup_right_arm=setup_right_arm,
            setup_left_hand=setup_left_hand,
            setup_right_hand=setup_right_hand,
            left_arm_dof=left_arm_dof,
            right_arm_dof=right_arm_dof,
            left_hand_dof=left_hand_dof,
            right_hand_dof=right_hand_dof,
            task_name=runtime_config.get('task_name', ''),
            dt=dt_value,
            max_timestamp_diff=max_timestamp_diff,
            use_system_time=use_system_time
        )
        
        # Wait for time synchronization to be ready
        if hasattr(env, 'time_sync_manager') and env.time_sync_manager:
            print("Waiting for time synchronization...")
            if not env.time_sync_manager.wait_for_synchronized_time(timeout=1.0):  # Reduced timeout
                print("Warning: Time synchronization not ready, proceeding anyway")
        
        # Block until cameras are ready (or timeout) before starting capture
        try:
            env.image_recorder.wait_until_ready(timeout_sec=1.0)  # Reduced timeout for faster startup
        except Exception:
            pass
        
        # Reset environment
        ts = env.reset()
        
        # Data collection loop
        for t in tqdm(range(max_timesteps), desc=f"Recording {episode_name}"):
            # Check for manual stop signal
            if stop_callback and stop_callback():
                print(f"手动停止录制，共录制 {t} 步")
                break
                
            t0 = time.time()
            
            # Get action and step environment
            action = env.get_action()
            ts = env.step(action)
            
            # Write observation to episode
            success = episode_writer.write_observation(
                obs=ts.observation,
                action=action,
                timestamp=t0
            )
            
            if not success:
                print(f"Warning: Failed to write observation at timestep {t}")
            
            # Sleep to maintain timing
            elapsed = time.time() - t0
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
        
        # Get final statistics
        stats = episode_writer.get_episode_stats()
        print(f"Episode completed: {stats['frame_count']} frames in {stats['duration']:.2f}s")
        print(f"Video writer healthy: {stats['video_writer_healthy']}")
        
        if not stats['video_writer_healthy']:
            print(f"Video writer errors: {stats['video_writer_errors']}")
            return False
        
        return True


def create_episode_symlink(session_dir, episode_name, task_base_dir):
    """Create symlink in all_episodes directory for easy training access"""
    all_episodes_dir = os.path.join(task_base_dir, 'all_episodes')
    if not os.path.exists(all_episodes_dir):
        os.makedirs(all_episodes_dir, exist_ok=True)
    
    source_path = os.path.join(session_dir, episode_name)
    symlink_path = os.path.join(all_episodes_dir, episode_name)
    
    # Remove existing symlink if it exists
    if os.path.exists(symlink_path) or os.path.islink(symlink_path):
        os.unlink(symlink_path)
    
    # Create relative symlink
    rel_source = os.path.relpath(source_path, all_episodes_dir)
    os.symlink(rel_source, symlink_path)
    print(f'Created symlink: {symlink_path} -> {rel_source}')


def main(args, node=None):
    """
    Main function for video-based episode recording.
    
    Args:
        args: Dictionary containing task configuration
        node: Optional ROS2 node
    """
    task_name = args['task_configs_name']
    runtime_config = build_runtime_config(task_name)
    
    # New structure: task_path is the session directory within the task
    session_dir = args['task_path']  # This is now the session directory
    task_base_dir = os.path.dirname(session_dir)  # This is the task base directory
    
    # For manual recording with stop_callback, use episode_len from config
    # (user can stop manually before reaching the limit)
    max_timesteps = runtime_config['collection']['episode_len']

    camera_preset = runtime_config['cameras'].get('cameras', [])
    camera_names = [cam['name'] for cam in camera_preset] or runtime_config['cameras'].get('camera_names', [])
    stereo_mode = runtime_config['cameras'].get('stereo_mode', args.get('stereo_mode', False))
    
    # Handle episode naming
    if args['episode_idx'] is not None:
        episode_idx = args['episode_idx']
        # Handle timestamp-based naming
        if isinstance(episode_idx, str) and episode_idx.startswith("timestamp_"):
            episode_name = episode_idx  # Use timestamp directly as episode name
        else:
            episode_name = f'episode_{episode_idx:06d}'  # Zero-padded episode number
    else:
        episode_idx = get_auto_index(session_dir)
        episode_name = f'episode_{episode_idx:06d}'
    
    # RealSense cameras are auto-detected via ROS2 topics, no configuration needed
    
    # Get stop_callback from args if available
    stop_callback = args.get('stop_callback', None)
    
    # Check if episode already exists (only for numeric episode_idx)
    if isinstance(episode_idx, int):
        episode_path = os.path.join(session_dir, episode_name)
        if os.path.exists(episode_path):
            print(f"Episode {episode_name} already exists in {session_dir}")
            return False
    
    # Display appropriate message based on naming strategy
    if isinstance(episode_idx, str) and episode_idx.startswith("timestamp_"):
        print(f"Starting video recording for task {task_name} with timestamp: {episode_idx}")
    else:
        print(f"Starting video recording for task {task_name}, episode: {episode_name}")
    
    # Record episode
    is_healthy = capture_one_episode_video(
        dt=runtime_config['collection']['dt'],
        max_timesteps=max_timesteps,
        camera_names=camera_names,
        episode_dir=session_dir,
        episode_name=episode_name,
        task_config=runtime_config,
        node=node,
        stereo_mode=stereo_mode,
        stop_callback=stop_callback
    )
    
    # Create symlink in all_episodes directory for easy training access
    if is_healthy:
        create_episode_symlink(session_dir, episode_name, task_base_dir)
        if isinstance(episode_idx, str) and episode_idx.startswith("timestamp_"):
            print(f'Timestamp episode {episode_idx} recorded successfully in session: {os.path.basename(session_dir)}')
        else:
            print(f'Episode {episode_name} recorded successfully in session: {os.path.basename(session_dir)}')
    
    return is_healthy


def get_auto_index(dataset_dir, dataset_name_prefix='', data_suffix=''):
    """
    Get auto-generated episode index for video format.
    
    Args:
        dataset_dir: Directory to check for existing episodes
        dataset_name_prefix: Prefix for episode names
        data_suffix: Suffix for episode names (not used for video format)
        
    Returns:
        int: Next available episode index
    """
    max_idx = 10000  # Increased limit for better scalability
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir, exist_ok=True)
    
    # Look for the highest episode number across all sessions
    all_episodes_dir = os.path.join(os.path.dirname(dataset_dir), 'all_episodes')
    if os.path.exists(all_episodes_dir):
        # Find the highest episode number in all_episodes directory
        existing_episodes = []
        for filename in os.listdir(all_episodes_dir):
            if filename.startswith(f'{dataset_name_prefix}episode_') and os.path.isdir(os.path.join(all_episodes_dir, filename)):
                try:
                    episode_num = int(filename.split('_')[1])
                    existing_episodes.append(episode_num)
                except (ValueError, IndexError):
                    continue
        
        if existing_episodes:
            start_idx = max(existing_episodes) + 1
        else:
            start_idx = 0
    else:
        start_idx = 0
    
    # Find next available index starting from start_idx
    for i in range(start_idx, max_idx + 1):
        episode_name = f'{dataset_name_prefix}episode_{i:06d}'
        episode_path = os.path.join(dataset_dir, episode_name)
        if not os.path.exists(episode_path):
            return i
    
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")


if __name__ == "__main__":
    # Test the video recording functionality
    import tempfile
    import shutil
    
    # Create test arguments
    test_args = {
        'task_configs_name': 'linker_hand_grasp',
        'task_path': '/tmp/test_session',
        'episode_idx': 1,
        'stereo_mode': False
    }
    
    # Create test directory
    os.makedirs(test_args['task_path'], exist_ok=True)
    
    try:
        # Run test
        result = main(test_args)
        print(f"Test result: {result}")
        
        # Check output
        episode_path = os.path.join(test_args['task_path'], 'episode_000001')
        if os.path.exists(episode_path):
            print(f"Episode directory created: {episode_path}")
            for root, dirs, files in os.walk(episode_path):
                for file in files:
                    file_path = os.path.join(root, file)
                    file_size = os.path.getsize(file_path)
                    rel_path = os.path.relpath(file_path, episode_path)
                    print(f"  {rel_path}: {file_size} bytes")
        else:
            print("Episode directory not created")
    
    finally:
        # Cleanup
        if os.path.exists(test_args['task_path']):
            shutil.rmtree(test_args['task_path'])
        print("Test completed and cleaned up")
