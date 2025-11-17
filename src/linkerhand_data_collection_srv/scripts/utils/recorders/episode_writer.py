#!/usr/bin/env python3
# -*-coding:utf8-*-
"""
EpisodeWriter class for the new compressed video data format.

This module provides utilities for writing episodes in the new format:
episode_000123/
├── telemetry.npz
├── cameras/
│   ├── cam0.mp4
│   ├── cam0.timestamps.npy
│   └── ...
├── camera_info.json
└── manifest.json
"""

import os
import json
import time
import numpy as np
from typing import Dict, List, Optional, Any, Union
from pathlib import Path
import logging

from .video_writer import MultiCameraVideoWriter

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class EpisodeWriter:
    """
    Writer for the new compressed video episode format.
    
    Manages:
    - Video encoding with FFmpeg
    - Telemetry data in NPZ format
    - Camera information and metadata
    - Episode manifest
    """
    
    def __init__(self, 
                 episode_dir: str,
                 camera_configs: Dict[str, Dict],
                 episode_metadata: Optional[Dict] = None):
        """
        Initialize episode writer.
        
        Args:
            episode_dir: Directory path for episode (e.g., 'episode_000123')
            camera_configs: Camera configurations for video encoding
            episode_metadata: Optional metadata for the episode
        """
        self.episode_dir = Path(episode_dir)
        self.episode_dir.mkdir(parents=True, exist_ok=True)
        
        # Create cameras subdirectory
        self.cameras_dir = self.episode_dir / 'cameras'
        self.cameras_dir.mkdir(exist_ok=True)
        
        self.camera_configs = camera_configs
        self.episode_metadata = episode_metadata or {}
        
        # Initialize video writer
        self.video_writer = MultiCameraVideoWriter(
            output_dir=str(self.episode_dir),
            camera_configs=camera_configs
        )
        
        # Extract tactile collection configuration from metadata
        hardware_config = episode_metadata.get('hardware', {}) if episode_metadata else {}
        self.collect_tactile = hardware_config.get('collect_tactile', False)
        active_sides = hardware_config.get('active_sides', {})
        self.collect_left_hand_tactile = self.collect_tactile and active_sides.get('left_hand', False)
        self.collect_right_hand_tactile = self.collect_tactile and active_sides.get('right_hand', False)
        
        # Telemetry data storage - base data always initialized
        self.telemetry_data = {
            'timestamps': [],
            'qpos': [],
            'qvel': [],
            'effort': [],
            'actions': [],
            'sync_validation_is_valid': [],
            'sync_validation_max_diff': [],
            'sync_validation_timestamp_sensors': [],
            'sync_validation_timestamp_values': []
        }
        
        # Conditionally initialize tactile data lists based on configuration
        if self.collect_left_hand_tactile:
            for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                self.telemetry_data[f'tactile_left_hand_{finger}'] = []
        
        if self.collect_right_hand_tactile:
            for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                self.telemetry_data[f'tactile_right_hand_{finger}'] = []
        
        # Camera info storage
        self.camera_info = {}
        
        # Episode statistics
        self.frame_count = 0
        self.start_time = time.time()
        
        # Log tactile collection configuration
        if self.collect_tactile:
            sides = []
            if self.collect_left_hand_tactile:
                sides.append('左手')
            if self.collect_right_hand_tactile:
                sides.append('右手')
            logger.info(f"EpisodeWriter initialized for {episode_dir} - 触觉数据采集: {' + '.join(sides)}")
        else:
            logger.info(f"EpisodeWriter initialized for {episode_dir} - 触觉数据采集: 未启用")
    
    def write_observation(self, 
                         obs: Dict[str, Any], 
                         action: Optional[np.ndarray] = None,
                         timestamp: Optional[float] = None) -> bool:
        
        """
        Write observation data to episode.
        
        Args:
            obs: Observation dictionary containing sensor data
            action: Optional action array
            timestamp: Optional timestamp (uses current time if None)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Use current time if no timestamp provided
            if timestamp is None:
                timestamp = time.time()
            
            # Store telemetry data
            self.telemetry_data['timestamps'].append(timestamp)
            self.telemetry_data['qpos'].append(obs.get('qpos', []))
            self.telemetry_data['qvel'].append(obs.get('qvel', []))
            self.telemetry_data['effort'].append(obs.get('effort', []))
            
            if action is not None:
                self.telemetry_data['actions'].append(action)
            else:
                # Use zeros if no action provided
                action_dim = len(obs.get('qpos', []))
                self.telemetry_data['actions'].append(np.zeros(action_dim))
            
            # Store tactile data - only if collection is enabled for the corresponding hand
            tactile_data = obs.get('tactile', {})
            
            # Process left hand tactile data if enabled
            if self.collect_left_hand_tactile:
                hand_side = 'left_hand'
                if hand_side in tactile_data:
                    hand_data = tactile_data[hand_side]
                    for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                        key = f'tactile_{hand_side}_{finger}'
                        if finger in hand_data:
                            self.telemetry_data[key].append(hand_data[finger])
                        else:
                            # Use zeros if no tactile data
                            self.telemetry_data[key].append(np.zeros((12, 6), dtype=np.float32))
                else:
                    # Use zeros if no hand data
                    for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                        key = f'tactile_{hand_side}_{finger}'
                        self.telemetry_data[key].append(np.zeros((12, 6), dtype=np.float32))
            
            # Process right hand tactile data if enabled
            if self.collect_right_hand_tactile:
                hand_side = 'right_hand'
                if hand_side in tactile_data:
                    hand_data = tactile_data[hand_side]
                    for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                        key = f'tactile_{hand_side}_{finger}'
                        if finger in hand_data:
                            self.telemetry_data[key].append(hand_data[finger])
                        else:
                            # Use zeros if no tactile data
                            self.telemetry_data[key].append(np.zeros((12, 6), dtype=np.float32))
                else:
                    # Use zeros if no hand data
                    for finger in ['thumb_matrix', 'index_matrix', 'middle_matrix', 'ring_matrix', 'little_matrix']:
                        key = f'tactile_{hand_side}_{finger}'
                        self.telemetry_data[key].append(np.zeros((12, 6), dtype=np.float32))
            
            # Store synchronization validation data
            sync_validation = obs.get('timestamp_validation', {})
            self.telemetry_data['sync_validation_is_valid'].append(sync_validation.get('is_valid', True))
            self.telemetry_data['sync_validation_max_diff'].append(sync_validation.get('info', {}).get('max_diff', 0.0))
            
            timestamps = sync_validation.get('timestamps', {})
            if timestamps:
                first_sensor = list(timestamps.keys())[0]
                first_timestamp = timestamps[first_sensor]
                self.telemetry_data['sync_validation_timestamp_sensors'].append(first_sensor)
                self.telemetry_data['sync_validation_timestamp_values'].append(first_timestamp)
            else:
                self.telemetry_data['sync_validation_timestamp_sensors'].append('none')
                self.telemetry_data['sync_validation_timestamp_values'].append(0.0)
            
            # Write video frames
            images = obs.get('images', {})
            if images:
                # Prepare frames for video writer
                frames = {}
                timestamps = {}
                
                for image_key, image in images.items():
                    # Handle stereo mode: remove _left suffix to get base camera name
                    if image_key.endswith('_left'):
                        base_cam_name = image_key[:-5]  # Remove '_left' suffix
                    else:
                        base_cam_name = image_key
                    
                    if base_cam_name in self.camera_configs:
                        frames[base_cam_name] = image
                        timestamps[base_cam_name] = timestamp
                
                # Write frames to video
                if frames:
                    results = self.video_writer.write_frames(frames, timestamps)
                    failed_cameras = [name for name, success in results.items() if not success]
                    if failed_cameras:
                        logger.warning(f"Failed to write frames for cameras: {failed_cameras}")
            
            # Store camera info (only once)
            if not self.camera_info and 'camera_info' in obs:
                self.camera_info = obs['camera_info']
            
            self.frame_count += 1
            return True
            
        except Exception as e:
            logger.error(f"Error writing observation: {e}")
            return False
    
    def finalize_episode(self) -> bool:
        """
        Finalize episode by writing all data files.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            logger.info(f"Finalizing episode with {self.frame_count} frames")
            
            # Close video writer
            self.video_writer.close()
            
            # Write telemetry data
            self._write_telemetry_data()
            
            # Write camera info
            self._write_camera_info()
            
            # Write manifest
            self._write_manifest()
            
            # Write video timestamps
            self._write_video_timestamps()
            
            logger.info(f"Episode finalized successfully: {self.episode_dir}")
            return True
            
        except Exception as e:
            logger.error(f"Error finalizing episode: {e}")
            return False
    
    def _write_telemetry_data(self):
        """Write telemetry data to NPZ file."""
        telemetry_path = self.episode_dir / 'telemetry.npz'
        
        # Convert lists to numpy arrays - only save keys that exist in telemetry_data
        npz_data = {}
        for key, data_list in self.telemetry_data.items():
            if data_list:
                npz_data[key] = np.array(data_list)
            else:
                # Create empty array with appropriate shape for non-tactile data
                # Tactile data keys only exist if collection is enabled, so we handle empty lists
                if key == 'timestamps':
                    npz_data[key] = np.array([])
                elif key.startswith('tactile_'):
                    # Only create empty tactile array if the key exists (i.e., collection is enabled)
                    npz_data[key] = np.array([]).reshape(0, 12, 6)
                else:
                    npz_data[key] = np.array([]).reshape(0, -1)
        
        # Add episode metadata as attributes
        metadata = {
            'episode_length': self.frame_count,
            'duration': time.time() - self.start_time,
            'camera_names': list(self.camera_configs.keys()),
            'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
            **self.episode_metadata
        }
        
        # Save NPZ file
        np.savez_compressed(telemetry_path, **npz_data)
        
        # Save metadata as separate file for easy access
        metadata_path = self.episode_dir / 'metadata.json'
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        logger.info(f"Telemetry data written to {telemetry_path}")
    
    def _write_camera_info(self):
        """Write camera information to JSON file."""
        if not self.camera_info:
            logger.warning("No camera info available")
            return
        
        camera_info_path = self.episode_dir / 'camera_info.json'
        
        with open(camera_info_path, 'w') as f:
            json.dump(self.camera_info, f, indent=2)
        
        logger.info(f"Camera info written to {camera_info_path}")
    
    def _write_manifest(self):
        """Write episode manifest."""
        manifest = {
            'episode_id': self.episode_dir.name,
            'format_version': '1.0',
            'created_at': time.strftime('%Y-%m-%d %H:%M:%S'),
            'duration': time.time() - self.start_time,
            'frame_count': self.frame_count,
            'files': {
                'telemetry': 'telemetry.npz',
                'camera_info': 'camera_info.json',
                'cameras': {}
            },
            'metadata': self.episode_metadata
        }
        
        # Add camera files
        for cam_name in self.camera_configs.keys():
            manifest['files']['cameras'][cam_name] = {
                'video': f'cameras/{cam_name}.mp4',
                'timestamps': f'cameras/{cam_name}.timestamps.npy'
            }
        
        manifest_path = self.episode_dir / 'manifest.json'
        with open(manifest_path, 'w') as f:
            json.dump(manifest, f, indent=2)
        
        logger.info(f"Manifest written to {manifest_path}")
    
    def _write_video_timestamps(self):
        """Write video timestamps to separate files."""
        all_timestamps = self.video_writer.get_timestamps()
        
        for cam_name, timestamps in all_timestamps.items():
            if timestamps:
                timestamp_path = self.cameras_dir / f'{cam_name}.timestamps.npy'
                np.save(timestamp_path, np.array(timestamps))
                logger.info(f"Timestamps written for {cam_name}: {len(timestamps)} frames")
    
    def get_episode_stats(self) -> Dict[str, Any]:
        """Get episode statistics."""
        return {
            'frame_count': self.frame_count,
            'duration': time.time() - self.start_time,
            'camera_count': len(self.camera_configs),
            'video_writer_healthy': self.video_writer.is_healthy(),
            'video_writer_errors': self.video_writer.get_errors(),
            'frame_counts': self.video_writer.get_frame_counts()
        }
    
    def is_healthy(self) -> bool:
        """Check if episode writer is healthy."""
        return self.video_writer.is_healthy()
    
    def close(self):
        """Close episode writer and finalize data."""
        self.finalize_episode()
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


def create_episode_writer(episode_dir: str, 
                         camera_names: List[str],
                         camera_resolutions: Dict[str, tuple[int, int]],
                         fps: int = 30,
                         quality: str = 'medium') -> EpisodeWriter:
    """
    Create an EpisodeWriter with standard camera configurations.
    
    Args:
        episode_dir: Directory for episode data
        camera_names: List of camera names
        camera_resolutions: Dictionary mapping camera names to (width, height)
        fps: Frames per second for video encoding
        quality: Video quality setting
        
    Returns:
        Configured EpisodeWriter instance
    """
    camera_configs = {}
    
    for cam_name in camera_names:
        if cam_name in camera_resolutions:
            width, height = camera_resolutions[cam_name]
            camera_configs[cam_name] = {
                'width': width,
                'height': height,
                'fps': fps,
                'quality': quality,
                'pixel_format': 'rgb24'
            }
        else:
            logger.warning(f"No resolution specified for camera {cam_name}, using default 640x480@30fps")
            camera_configs[cam_name] = {
                'width': 640,   # ✅ 统一默认分辨率为640x480
                'height': 480,  # ✅ 与RealSense相机配置一致
                'fps': fps,
                'quality': quality,
                'pixel_format': 'rgb24'
            }
    
    return EpisodeWriter(episode_dir, camera_configs)


def test_episode_writer():
    """Test function for EpisodeWriter class."""
    import tempfile
    import shutil
    
    # Create temporary directory
    temp_dir = tempfile.mkdtemp()
    episode_dir = os.path.join(temp_dir, 'test_episode')
    
    try:
        # Test camera configurations
        camera_configs = {
            'cam_left': {'width': 320, 'height': 240, 'fps': 30, 'quality': 'medium'},
            'cam_right': {'width': 320, 'height': 240, 'fps': 30, 'quality': 'medium'}
        }
        
        # Test episode writer
        with EpisodeWriter(episode_dir, camera_configs) as writer:
            
            # Generate test data
            for i in range(50):
                # Create test observation
                obs = {
                    'qpos': np.random.randn(32),  # 6+6+10+10 joints
                    'qvel': np.random.randn(32),
                    'effort': np.random.randn(32),
                    'images': {
                        'cam_left': np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8),
                        'cam_right': np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
                    },
                    'tactile': {
                        'left_hand': {
                            'thumb_matrix': np.random.randn(12, 6).astype(np.float32),
                            'index_matrix': np.random.randn(12, 6).astype(np.float32),
                            'middle_matrix': np.random.randn(12, 6).astype(np.float32),
                            'ring_matrix': np.random.randn(12, 6).astype(np.float32),
                            'little_matrix': np.random.randn(12, 6).astype(np.float32)
                        },
                        'right_hand': {
                            'thumb_matrix': np.random.randn(12, 6).astype(np.float32),
                            'index_matrix': np.random.randn(12, 6).astype(np.float32),
                            'middle_matrix': np.random.randn(12, 6).astype(np.float32),
                            'ring_matrix': np.random.randn(12, 6).astype(np.float32),
                            'little_matrix': np.random.randn(12, 6).astype(np.float32)
                        }
                    },
                    'camera_info': {
                        'cam_left': {'width': 320, 'height': 240, 'fps': 30},
                        'cam_right': {'width': 320, 'height': 240, 'fps': 30}
                    }
                }
                
                action = np.random.randn(32)
                
                success = writer.write_observation(obs, action)
                if not success:
                    print(f"Failed to write observation {i}")
                    break
                
                time.sleep(0.01)  # Small delay
            
            # Get stats
            stats = writer.get_episode_stats()
            print(f"Episode stats: {stats}")
        
        # Check output files
        episode_path = Path(episode_dir)
        print(f"Episode directory: {episode_path}")
        print(f"Files created:")
        for file_path in episode_path.rglob('*'):
            if file_path.is_file():
                print(f"  {file_path.relative_to(episode_path)}")
        
    finally:
        # Cleanup
        shutil.rmtree(temp_dir)
        print("Test completed and cleaned up")


if __name__ == "__main__":
    test_episode_writer()
