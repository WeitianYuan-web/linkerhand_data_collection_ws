import os
import time
import h5py
import argparse
import numpy as np
from tqdm import tqdm

from utils.config_loader import (
    build_runtime_config,
    LEFT_HAND_JOINT,
    RIGHT_HAND_JOINT,
    LEFT_HAND_TACTILE_SUPPORT,
    RIGHT_HAND_TACTILE_SUPPORT,
)
from utils.time_sync import TimeSyncManager
from utils.recorders.arm_recorder import ArmRecorder
# Legacy image recorder moved to legacy/ directory
# from utils.recorders.image_recorder import ImageRecorder
from utils.real_env import RealEnv, make_real_env

import IPython
e = IPython.embed




def get_camera_resolution(camera_info_data, cam_name):
    """Extract camera resolution from camera_info data"""
    if not camera_info_data or cam_name not in camera_info_data:
        return (360, 640)  # Default fallback resolution
    
    info = camera_info_data[cam_name]
    if 'height' in info and 'width' in info:
        return (info['height'], info['width'])
    else:
        return (360, 640)  # Default fallback resolution

def capture_one_episode(dt, max_timesteps, camera_names, dataset_dir, dataset_name, overwrite, node=None, stereo_mode=False, task_config=None, stop_callback=None):
    dataset_path = os.path.join(dataset_dir, dataset_name)
    
    # Get arm DoF configuration from task YAML (if available)
    runtime_config = task_config or {}
    # Get dt from task config, use parameter as fallback
    dt_value = runtime_config.get('collection', {}).get('dt', dt)
    # Get max_timestamp_diff from task config
    max_timestamp_diff = runtime_config.get('collection', {}).get('max_timestamp_diff', 0.1)
    # Get use_system_time from task config
    use_system_time = runtime_config.get('collection', {}).get('use_system_time', False)
    hardware = runtime_config.get('hardware', {})
    arm_joint_config = hardware.get('arm_joints', {})
    
    left_arm_dof = task_config.get('left_arm_dof') if task_config else None
    if left_arm_dof is None:
        left_arm_dof = arm_joint_config.get('left_arm_dof', 0)
    right_arm_dof = task_config.get('right_arm_dof') if task_config else None
    if right_arm_dof is None:
        right_arm_dof = arm_joint_config.get('right_arm_dof', 0)

    hand_joint_config = hardware.get('hand_joints', {})
    left_hand_dof = task_config.get('left_hand_dof') if task_config else None
    if left_hand_dof is None:
        left_hand_dof = hand_joint_config.get('left_hand_dof', LEFT_HAND_JOINT)
    right_hand_dof = task_config.get('right_hand_dof') if task_config else None
    if right_hand_dof is None:
        right_hand_dof = hand_joint_config.get('right_hand_dof', RIGHT_HAND_JOINT)
    
    print(f"任务配置: 左臂{left_arm_dof}DoF, 右臂{right_arm_dof}DoF, 左手{left_hand_dof}DoF, 右手{right_hand_dof}DoF")
    
    # Extract active_sides from runtime_config
    active_sides = runtime_config.get('hardware', {}).get('active_sides', {})
    setup_left_arm = active_sides.get('left_arm', False)
    setup_right_arm = active_sides.get('right_arm', False)
    setup_left_hand = active_sides.get('left_hand', True)
    setup_right_hand = active_sides.get('right_hand', True)
    
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
        if not env.time_sync_manager.wait_for_synchronized_time(timeout=5.0):
            print("⚠️  Warning: Time synchronization not ready, proceeding anyway")
    
    # Block until cameras are ready (or timeout) before starting capture
    try:
        env.image_recorder.wait_until_ready(timeout_sec=5.0)
    except Exception:
        pass
    
    ts = env.reset()
    actions = []
    timesteps = [ts]
    actions = []
    # action_end_pos = []  # Commented out: end_pos recording disabled
    actual_dt_history = []
    sync_validation_history = []
    
    # Get camera resolution from first timestep's camera_info
    first_camera_info = ts.observation.get('camera_info', {})
    camera_resolutions = {}
    for cam_name in camera_names:
        height, width = get_camera_resolution(first_camera_info, cam_name)
        if width == 0 or height == 0:
            width, height = 640, 480
        camera_resolutions[cam_name] = (height, width)
        # Warn only if resolution deviates from expected 640x480
        if width != 640 or height != 480:
            print(f"⚠️  Unexpected resolution for {cam_name}: {width}x{height}")
    
    for t in tqdm(range(max_timesteps)):
        # Check for manual stop signal
        if stop_callback and stop_callback():
            print(f"手动停止录制，共录制 {t} 步")
            break
            
        t0 = time.time() #
            
        action = env.get_action()
        t1 = time.time() #
        ts = env.step(action)
        t2 = time.time() #
        timesteps.append(ts)
        actions.append(action)
        actual_dt_history.append([t0, t1, t2])

        # action_end_pos.append(action_end)  # Commented out: end_pos recording disabled
        # action_end = env.get_piper_master_end_pose()  # Commented out: end_pos recording disabled
        
        # Store synchronization validation info
        if 'timestamp_validation' in ts.observation:
            sync_validation_history.append(ts.observation['timestamp_validation'])
    """
    For each timestep:
    observations
    - images
        - cam_high          (H, W, 3) 'uint8'  # Resolution detected from camera_info
        - cam_low           (H, W, 3) 'uint8'  # Resolution detected from camera_info
        - cam_left_wrist    (H, W, 3) 'uint8'  # Resolution detected from camera_info
        - cam_right_wrist   (H, W, 3) 'uint8'  # Resolution detected from camera_info
    - qpos                  (14,)         'float64'
    - qvel                  (14,)         'float64'
    
    action                  (14,)         'float64'
    """

    arm_doF_left = runtime_config.get('hardware', {}).get('arm_joints', {}).get('left_arm_dof', left_arm_dof)
    arm_doF_right = runtime_config.get('hardware', {}).get('arm_joints', {}).get('right_arm_dof', right_arm_dof)
    hand_dof_left = runtime_config.get('hardware', {}).get('hand_joints', {}).get('left_hand_dof', left_hand_dof)
    hand_dof_right = runtime_config.get('hardware', {}).get('hand_joints', {}).get('right_hand_dof', right_hand_dof)
    qpos_dim = arm_doF_left + arm_doF_right + hand_dof_left + hand_dof_right

    data_dict = {
        '/observations/qpos': [],
        '/observations/qvel': [],
        '/observations/effort': [],
        '/action': []
    }
    
    # Initialize tactile data arrays for each hand and finger
    for hand_side in ['left_hand', 'right_hand']:
        for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
            key = f'/observations/tactile_{hand_side}_{finger}'
            data_dict[key] = []
    
    # Handle image data based on stereo mode
    if stereo_mode:
        for cam_name in camera_names:
            data_dict[f'/observations/images/{cam_name}_left'] = []
            data_dict[f'/observations/images/{cam_name}_right'] = []
    else:
        for cam_name in camera_names:
            data_dict[f'/observations/images/{cam_name}'] = []
    
    # Add camera_info data (stored once per episode)
    data_dict['/camera_info'] = []
    
    # Add synchronization validation data
    data_dict['/sync_validation/is_valid'] = []
    data_dict['/sync_validation/max_diff'] = []
    data_dict['/sync_validation/timestamp_sensors'] = []
    data_dict['/sync_validation/timestamp_values'] = []

    def _ensure_length(array, target_len):
        if array is None:
            return np.zeros(target_len)
        array = np.asarray(array)
        flat = array.reshape(-1)
        current_len = flat.shape[0]
        if current_len == target_len:
            return flat
        if current_len > target_len:
            return flat[:target_len]
        padded = np.zeros(target_len)
        padded[:current_len] = flat
        return padded

    # len(action): max_timesteps, len(time_steps): max_timesteps + 1
    while actions:
        action = actions.pop(0)
        ts = timesteps.pop(0)
        obs = ts.observation
        # Ensure joint arrays have consistent length
        data_dict['/observations/qpos'].append(_ensure_length(obs['qpos'], qpos_dim))
        data_dict['/observations/qvel'].append(_ensure_length(obs['qvel'], qpos_dim))
        data_dict['/observations/effort'].append(_ensure_length(obs['effort'], qpos_dim))
        
        # Process tactile data to make it HDF5-compatible
        tactile_data = obs['tactile']
        # Flatten tactile data into separate arrays for each hand and finger
        for hand_side in ['left_hand', 'right_hand']:
            if hand_side in tactile_data:
                hand_data = tactile_data[hand_side]
                for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                    if finger in hand_data:
                        key = f'/observations/tactile_{hand_side}_{finger}'
                        if key not in data_dict:
                            data_dict[key] = []
                        data_dict[key].append(hand_data[finger])
        
        data_dict['/action'].append(_ensure_length(action, qpos_dim))

        # data_dict['/observations/end_pos'].append(obs['end_pos'])  # Commented out: end_pos recording disabled
        # action_end = action_end_pos.pop(0)  # Commented out: end_pos recording disabled
        # data_dict['/action_end_pos'].append(action_end)  # Commented out: end_pos recording disabled

        # Handle image data based on stereo mode
        if stereo_mode:
            for cam_name in camera_names:
                # Handle left stream
                left_key = f'{cam_name}_left'
                if left_key in obs['images']:
                    data_dict[f'/observations/images/{left_key}'].append(obs['images'][left_key])
                else:
                    height, width = camera_resolutions[cam_name]
                    data_dict[f'/observations/images/{left_key}'].append(
                        np.zeros((height, width, 3), dtype=np.uint8)
                    )
                
                # Handle right stream
                right_key = f'{cam_name}_right'
                if right_key in obs['images']:
                    data_dict[f'/observations/images/{right_key}'].append(obs['images'][right_key])
                else:
                    height, width = camera_resolutions[cam_name]
                    data_dict[f'/observations/images/{right_key}'].append(
                        np.zeros((height, width, 3), dtype=np.uint8)
                    )
        else:
            for cam_name in camera_names:
                if cam_name in obs['images']:
                    data_dict[f'/observations/images/{cam_name}'].append(obs['images'][cam_name])
                else:
                    # fallback: fill zeros if a camera frame is missing
                    height, width = camera_resolutions[cam_name]
                    data_dict[f'/observations/images/{cam_name}'].append(
                        np.zeros((height, width, 3), dtype=np.uint8)
                    )
        
        # Store camera_info (only once per episode, so we'll store it in the first timestep)
        if len(data_dict['/camera_info']) == 0:
            data_dict['/camera_info'].append(obs.get('camera_info', {}))
        else:
            # For subsequent timesteps, append empty dict to maintain array length
            data_dict['/camera_info'].append({})
        
        # Process synchronization validation data
        if sync_validation_history:
            sync_data = sync_validation_history.pop(0)
            data_dict['/sync_validation/is_valid'].append(sync_data['is_valid'])
            data_dict['/sync_validation/max_diff'].append(sync_data['info'].get('max_diff', 0.0))
            
            # Store timestamps as separate arrays
            timestamps = sync_data['timestamps']
            if timestamps:
                # Store the first timestamp (most representative)
                first_sensor = list(timestamps.keys())[0]
                first_timestamp = timestamps[first_sensor]
                data_dict['/sync_validation/timestamp_sensors'].append(first_sensor.encode('utf-8'))
                data_dict['/sync_validation/timestamp_values'].append(first_timestamp)
            else:
                data_dict['/sync_validation/timestamp_sensors'].append(b'none')
                data_dict['/sync_validation/timestamp_values'].append(0.0)

    num_steps = len(data_dict['/action'])
    if num_steps == 0:
        print('No data captured; skipping file save')
        return False

    # HDF5

    t0 = time.time()
    with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024**2*2) as root:
        # Basic metadata
        root.attrs['sim'] = False
        root.attrs['task_name'] = runtime_config.get('task_name', 'unknown')
        root.attrs['task_description'] = runtime_config.get('description', '')
        root.attrs['dt'] = runtime_config.get('collection', {}).get('dt', 0.04)
        root.attrs['created_at'] = time.strftime('%Y-%m-%d %H:%M:%S')
        
        # Hardware configuration metadata
        root.attrs['hardware_preset'] = runtime_config.get('hardware', {}).get('preset', 'unknown')
        root.attrs['left_arm_active'] = setup_left_arm
        root.attrs['right_arm_active'] = setup_right_arm
        root.attrs['left_hand_active'] = setup_left_hand
        root.attrs['right_hand_active'] = setup_right_hand
        root.attrs['left_arm_dof'] = left_arm_dof if setup_left_arm else 0
        root.attrs['right_arm_dof'] = right_arm_dof if setup_right_arm else 0
        root.attrs['left_hand_dof'] = left_hand_dof if setup_left_hand else 0
        root.attrs['right_hand_dof'] = right_hand_dof if setup_right_hand else 0
        root.attrs['total_dof'] = qpos_dim
        root.attrs['collect_tactile'] = runtime_config.get('hardware', {}).get('collect_tactile', False)
        
        # Camera metadata
        root.attrs['camera_names'] = ','.join(camera_names)
        root.attrs['stereo_mode'] = stereo_mode
        
        # Device metadata (hardware specs of the collection machine)
        device_config = runtime_config.get('device', {})
        root.attrs['device_number'] = device_config.get('device_number', 'unknown')
        root.attrs['device_type'] = device_config.get('device_type', 'unknown')
        hardware_specs = device_config.get('hardware_specs', {})
        root.attrs['device_gpu'] = hardware_specs.get('gpu', 'unknown')
        root.attrs['device_cpu'] = hardware_specs.get('cpu', 'unknown')
        root.attrs['device_memory'] = hardware_specs.get('memory', 'unknown')
        root.attrs['device_storage'] = hardware_specs.get('storage', 'unknown')
        
        obs = root.create_group('observations')
        image = obs.create_group('images')
        
        # Create image datasets based on stereo mode
        if stereo_mode:
            for cam_name in camera_names:
                height, width = camera_resolutions[cam_name]
                # Create left stream dataset
                _ = image.create_dataset(f'{cam_name}_left', (num_steps, height, width, 3), dtype='uint8',
                                         chunks=(1, height, width, 3))
                # Create right stream dataset
                _ = image.create_dataset(f'{cam_name}_right', (num_steps, height, width, 3), dtype='uint8',
                                         chunks=(1, height, width, 3))
        else:
            for cam_name in camera_names:
                height, width = camera_resolutions[cam_name]
                _ = image.create_dataset(cam_name, (num_steps, height, width, 3), dtype='uint8',
                                         chunks=(1, height, width, 3))
        _ = obs.create_dataset('qpos', (num_steps, qpos_dim))
        _ = obs.create_dataset('qvel', (num_steps, qpos_dim))
        _ = obs.create_dataset('effort', (num_steps, qpos_dim))
        _ = root.create_dataset('action', (num_steps, qpos_dim))
        
        # Create tactile data datasets
        tactile_group = obs.create_group('tactile')
        for hand_side in ['left_hand', 'right_hand']:
            hand_group = tactile_group.create_group(hand_side)
            for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                # Each finger has a 12x6 matrix, so shape is (max_timesteps, 12, 6)
                _ = hand_group.create_dataset(finger, (num_steps, 12, 6), dtype='float32')

        # end_pos_dim = LEFT_HAND_JOINT + RIGHT_HAND_JOINT  # ee pose kept as 12; hands appended separately in obs['end_pos']  # Commented out: end_pos recording disabled
        # _ = obs.create_dataset('end_pos', (max_timesteps, end_pos_dim + LEFT_HAND_JOINT + RIGHT_HAND_JOINT))  # Commented out: end_pos recording disabled
        # _ = root.create_dataset('action_end_pos', (max_timesteps, LEFT_HAND_JOINT + RIGHT_HAND_JOINT))  # Commented out: end_pos recording disabled
        
        # Create camera_info dataset (stored as attributes since it's constant per episode)
        camera_info_group = root.create_group('camera_info')
        
        # Create synchronization validation datasets
        sync_group = root.create_group('sync_validation')
        _ = sync_group.create_dataset('is_valid', (num_steps,), dtype='bool')
        _ = sync_group.create_dataset('max_diff', (num_steps,), dtype='float64')
        # Store timestamps as separate arrays instead of structured array
        _ = sync_group.create_dataset('timestamp_sensors', (num_steps,), dtype='S50')  # Sensor names
        _ = sync_group.create_dataset('timestamp_values', (num_steps,), dtype='float64')  # Timestamp values

        for name, array in data_dict.items():
            if name == '/camera_info':
                # Handle camera_info specially - store as attributes on the camera_info group
                if array and array[0]:  # If we have camera_info data
                    camera_info_data = array[0]
                    for cam_name, info in camera_info_data.items():
                        if cam_name in camera_info_group:
                            # Update existing group
                            cam_group = camera_info_group[cam_name]
                        else:
                            # Create new group
                            cam_group = camera_info_group.create_group(cam_name)
                        
                        # Store camera_info as attributes
                        for key, value in info.items():
                            if isinstance(value, dict):
                                # Handle nested dictionaries (like header, roi)
                                for sub_key, sub_value in value.items():
                                    if isinstance(sub_value, dict):
                                        # Handle deeply nested (like header.stamp)
                                        for deep_key, deep_value in sub_value.items():
                                            cam_group.attrs[f'{key}_{sub_key}_{deep_key}'] = deep_value
                                    else:
                                        cam_group.attrs[f'{key}_{sub_key}'] = sub_value
                            elif isinstance(value, list):
                                # Handle lists (like K, D, R, P matrices)
                                cam_group.attrs[key] = np.array(value)
                            else:
                                cam_group.attrs[key] = value
            elif name.startswith('/observations/tactile_'):
                # Handle tactile data - write to the appropriate tactile group
                # Parse the key: /observations/tactile_{hand_side}_{finger}
                # Example: /observations/tactile_left_hand_thumb_matrix
                parts = name.split('_')
                if len(parts) >= 6:
                    hand_side = f"{parts[3]}_{parts[4]}"  # left_hand or right_hand
                    finger = f"{parts[5]}_{parts[6]}"  # thumb_matrix, index_matrix, etc.
                    tactile_group = root['observations']['tactile']
                    hand_group = tactile_group[hand_side]
                    hand_group[finger][...] = np.array(array)
            else:
                root[name][...] = array
    print(f'Saving: {time.time() - t0:.1f} secs')

    # Clean up resources after recording
    try:
        env.cleanup()
        print("Episode recording cleanup completed")
        # Add a longer delay to ensure cameras are fully released and ready for next episode
        time.sleep(2.0)
    except Exception as e:
        print(f"Warning: Error during cleanup: {e}")

    return True


def create_episode_symlink(session_dir, episode_filename, task_base_dir):
    """Create symlink in all_episodes directory for easy training access"""
    all_episodes_dir = os.path.join(task_base_dir, 'all_episodes')
    if not os.path.exists(all_episodes_dir):
        os.makedirs(all_episodes_dir, exist_ok=True)
    
    source_path = os.path.join(session_dir, episode_filename)
    symlink_path = os.path.join(all_episodes_dir, episode_filename)
    
    # Remove existing symlink if it exists
    if os.path.exists(symlink_path) or os.path.islink(symlink_path):
        os.unlink(symlink_path)
    
    # Create relative symlink
    rel_source = os.path.relpath(source_path, all_episodes_dir)
    os.symlink(rel_source, symlink_path)
    print(f'Created symlink: {symlink_path} -> {rel_source}')


def main(args, node=None):
    task_name = args['task_configs_name']
    runtime_config = build_runtime_config(task_name)
    
    # New structure: task_path is the session directory within the task
    session_dir = args['task_path']  # This is now the session directory
    task_base_dir = os.path.dirname(session_dir)  # This is the task base directory
    
    # For manual recording, use a much larger limit or check if stop_callback is provided
    if args.get('stop_callback') is not None:
        max_timesteps = runtime_config['collection'].get('episode_len', 5000)
    else:
        max_timesteps = runtime_config['collection']['episode_len']

    camera_preset = runtime_config['cameras'].get('cameras', [])
    camera_names = [cam['name'] for cam in camera_preset]
    if not camera_names:
        camera_names = runtime_config['cameras'].get('camera_names', [])

    stereo_mode = runtime_config['cameras'].get('stereo_mode', False)
    
    if args['episode_idx'] is not None:
        episode_idx = args['episode_idx']
        # Handle timestamp-based naming
        if isinstance(episode_idx, str) and episode_idx.startswith("timestamp_"):
            dataset_name = episode_idx  # Use timestamp directly as dataset name
        else:
            dataset_name = f'episode_{episode_idx}'
    else:
        episode_idx = get_auto_index(session_dir)
        dataset_name = f'episode_{episode_idx}'
    overwrite = True
    
    # RealSense cameras are auto-detected via ROS2 topics, no configuration needed
    
    # Get stop_callback from args if available
    stop_callback = args.get('stop_callback', None)
    
    is_healthy = capture_one_episode(
        runtime_config['collection']['dt'],
        max_timesteps,
        camera_names,
        session_dir,
        dataset_name,
        overwrite,
        node=node,
        stereo_mode=stereo_mode,
        task_config=runtime_config,
        stop_callback=stop_callback
    )
    
    # Create symlink in all_episodes directory for easy training access
    if is_healthy:
        episode_filename = f'{dataset_name}.hdf5'
        create_episode_symlink(session_dir, episode_filename, task_base_dir)
        if isinstance(episode_idx, str) and episode_idx.startswith("timestamp_"):
            print(f'Timestamp episode {episode_idx} recorded successfully in session: {os.path.basename(session_dir)}')
        else:
            print(f'Episode {episode_idx} recorded successfully in session: {os.path.basename(session_dir)}')
    
    return is_healthy
    # while True:
    #     is_healthy = capture_one_episode(DT, max_timesteps, camera_names, dataset_dir, dataset_name, overwrite)
    #     if is_healthy:
    #         break


def get_auto_index(dataset_dir, dataset_name_prefix = '', data_suffix = 'hdf5'):
    max_idx = 10000  # Increased limit for better scalability
    if not os.path.isdir(dataset_dir):
        os.makedirs(dataset_dir, exist_ok=True)
    
    # Look for the highest episode number across all sessions
    all_episodes_dir = os.path.join(os.path.dirname(dataset_dir), 'all_episodes')
    if os.path.exists(all_episodes_dir):
        # Find the highest episode number in all_episodes directory
        existing_episodes = []
        for filename in os.listdir(all_episodes_dir):
            if filename.startswith(f'{dataset_name_prefix}episode_') and filename.endswith(f'.{data_suffix}'):
                try:
                    episode_num = int(filename.split('_')[1].split('.')[0])
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
        if not os.path.isfile(os.path.join(dataset_dir, f'{dataset_name_prefix}episode_{i}.{data_suffix}')):
            return i
    raise Exception(f"Error getting auto index, or more than {max_idx} episodes")






