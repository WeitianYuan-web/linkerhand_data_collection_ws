# ZED Direct Recorder with ROS2 Time Synchronization

This document explains how to use the ZED Direct Recorder for higher frequency image capture while maintaining synchronization with the ROS2 system clock.

## Overview

The ZED Direct Recorder provides direct access to ZED cameras using the ZED SDK, bypassing ROS2 topics for higher frequency image capture. It maintains full compatibility with your existing time synchronization system and data collection pipeline.

## Key Benefits

- **Higher Frequency**: Direct SDK access provides higher frame rates than ROS2 topics
- **Time Synchronization**: Maintains synchronization with ROS2 system clock
- **Backward Compatibility**: Drop-in replacement for existing ImageRecorder
- **Auto-Detection**: Automatically detects and configures connected ZED cameras
- **Thread-Safe**: Multi-threaded capture with thread-safe data access
- **Stereo Support**: Full support for stereo image capture

## Installation

### 1. Install ZED SDK

First, install the ZED SDK on your system:

```bash
# Download and install ZED SDK from Stereolabs
# https://www.stereolabs.com/developers/release/

# Install Python wrapper
pip install pyzed
```

### 2. Verify Installation

Test that the ZED SDK is working:

```bash
cd src/linkerhand_data_collection_srv/scripts
python3 test_zed_direct.py
```

## Usage

### Basic Usage (Auto-Detect)

The simplest way to use the ZED Direct Recorder is with auto-detection:

```python
from utils.real_env import RealEnv
import rclpy

# Initialize ROS2
rclpy.init()
node = rclpy.create_node('data_collection')

# Create environment with ZED direct recorder
env = RealEnv(
    camera_names=[],  # Not needed for ZED direct
    setup_left_arm=True,
    setup_right_arm=True,
    setup_left_hand=True,
    setup_right_hand=True,
    node=node,
    stereo_mode=False,
    use_zed_direct=True,  # Enable ZED direct recorder
    zed_configs=None      # Auto-detect cameras
)

# Use the environment normally
obs = env.get_observation()
images = obs['images']
timestamps = obs.get('timestamp_validation', {})

print(f"Captured {len(images)} images")
print(f"Time sync valid: {timestamps.get('is_valid', False)}")
```

### Manual Configuration

For precise control over camera settings:

```python
import pyzed.sl as sl
from utils.real_env import RealEnv

# Define camera configurations
zed_configs = {
    'cam_top': {
        'serial_number': '12345678',  # Your camera's serial number
        'resolution': sl.RESOLUTION.HD720,
        'fps': 30,
        'coordinate_units': sl.UNIT.METER,
        'coordinate_system': sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP,
        'depth_mode': sl.DEPTH_MODE.NONE,  # No depth for faster capture
        'sdk_verbose': 0  # 0 = False, 1 = True
    },
    'cam_left_wrist': {
        'serial_number': '87654321',  # Your camera's serial number
        'resolution': sl.RESOLUTION.HD720,
        'fps': 30,
        'coordinate_units': sl.UNIT.METER,
        'coordinate_system': sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP,
        'depth_mode': sl.DEPTH_MODE.NONE,
        'sdk_verbose': 0  # 0 = False, 1 = True
    }
}

# Create environment with manual configuration
env = RealEnv(
    camera_names=[],
    setup_left_arm=True,
    setup_right_arm=True,
    setup_left_hand=True,
    setup_right_hand=True,
    node=node,
    stereo_mode=False,
    use_zed_direct=True,
    zed_configs=zed_configs
)
```

### Stereo Mode

Enable stereo capture for both left and right camera streams:

```python
env = RealEnv(
    camera_names=[],
    setup_left_arm=True,
    setup_right_arm=True,
    setup_left_hand=True,
    setup_right_hand=True,
    node=node,
    stereo_mode=True,  # Enable stereo mode
    use_zed_direct=True,
    zed_configs=None
)

# In stereo mode, images are named with _left and _right suffixes
obs = env.get_observation()
images = obs['images']
# images will contain: {'cam_top_left': ..., 'cam_top_right': ..., ...}
```

## Configuration Options

### Resolution Settings

```python
import pyzed.sl as sl

# Available resolutions:
# sl.RESOLUTION.VGA        # 672x376
# sl.RESOLUTION.HD720      # 1280x720
# sl.RESOLUTION.HD1080     # 1920x1080
# sl.RESOLUTION.HD2K       # 2208x1242

zed_configs = {
    'cam_top': {
        'serial_number': '12345678',
        'resolution': sl.RESOLUTION.HD720,  # Choose your resolution
        'fps': 30,
        # ... other settings
    }
}
```

### Frame Rate Settings

```python
zed_configs = {
    'cam_top': {
        'serial_number': '12345678',
        'resolution': sl.RESOLUTION.HD720,
        'fps': 60,  # Higher FPS for faster capture
        # ... other settings
    }
}
```

### Performance vs Quality Trade-offs

**High Performance (Lower Resolution, Higher FPS):**
```python
'resolution': sl.RESOLUTION.VGA,  # 672x376
'fps': 60,
'depth_mode': sl.DEPTH_MODE.NONE,
```

**High Quality (Higher Resolution, Lower FPS):**
```python
'resolution': sl.RESOLUTION.HD1080,  # 1920x1080
'fps': 15,
'depth_mode': sl.DEPTH_MODE.NONE,
```

## Time Synchronization

The ZED Direct Recorder automatically integrates with your existing time synchronization system:

### Validation

```python
# Check if timestamps are synchronized
is_valid, info = env.image_recorder.validate_synchronization()
print(f"Synchronization valid: {is_valid}")
if not is_valid:
    print(f"Sync info: {info}")
```

### Timestamp Access

```python
# Get timestamps for all cameras
timestamps = env.image_recorder.get_timestamps()
print(f"Camera timestamps: {timestamps}")

# Get current ROS2 time
current_time = env.time_sync_manager.get_current_time()
print(f"Current ROS2 time: {current_time}")
```

## Integration with Data Collection

The ZED Direct Recorder is fully compatible with your existing data collection pipeline:

### Episode Recording

```python
from utils.record_episodes import capture_one_episode

# Record episode with ZED direct recorder
capture_one_episode(
    dt=0.04,
    max_timesteps=800,
    camera_names=[],  # Not needed for ZED direct
    dataset_dir='/path/to/dataset',
    dataset_name='episode_001',
    overwrite=True,
    node=node,
    stereo_mode=False,
    task_config={
        'setup_left_arm': True,
        'setup_right_arm': True,
        'setup_left_hand': True,
        'setup_right_hand': True,
        'use_zed_direct': True,  # Enable ZED direct recorder
        'zed_configs': None      # Auto-detect
    }
)
```

### Data Format

The recorded data format remains the same:

```python
# HDF5 structure:
# /observations/images/cam_top          # (H, W, 3) uint8
# /observations/images/cam_left_wrist   # (H, W, 3) uint8
# /observations/images/cam_right_wrist  # (H, W, 3) uint8
# /observations/qpos                    # (32,) float64
# /observations/qvel                    # (32,) float64
# /observations/effort                  # (32,) float64
# /action                               # (32,) float64
# /camera_info                          # Camera information
# /sync_validation/is_valid            # Boolean array
# /sync_validation/max_diff            # Maximum timestamp difference
# /sync_validation/timestamp_sensors   # Sensor names
# /sync_validation/timestamp_values    # Timestamp values
```

## Troubleshooting

### Common Issues

1. **"ZED SDK not available"**
   ```bash
   # Install ZED SDK
   pip install pyzed
   ```

2. **"No ZED cameras detected"**
   - Check USB connections
   - Verify camera power
   - Run camera detection test:
   ```python
   from utils.recorders.zed_direct_recorder import detect_zed_cameras
   serial_numbers = detect_zed_cameras()
   print(f"Detected cameras: {serial_numbers}")
   ```

3. **"Failed to open ZED camera"**
   - Check serial number is correct
   - Verify camera is not in use by another application
   - Try different USB ports

4. **"Time synchronization failed"**
   - Check ROS2 clock is running
   - Verify time sync manager is initialized
   - Increase tolerance if needed:
   ```python
   from utils.time_sync import TimeSyncManager
   time_sync = TimeSyncManager(max_timestamp_diff=0.1)  # 100ms tolerance
   ```

### Performance Optimization

1. **Reduce Resolution**: Use `sl.RESOLUTION.VGA` for higher FPS
2. **Disable Depth**: Set `depth_mode=sl.DEPTH_MODE.NONE`
3. **Lower FPS**: Reduce FPS if CPU/GPU is overloaded
4. **Close Other Applications**: Free up system resources

### Debug Information

Enable verbose logging:

```python
zed_configs = {
    'cam_top': {
        'serial_number': '12345678',
        'sdk_verbose': 1,  # Enable verbose logging (1 = True, 0 = False)
        # ... other settings
    }
}
```

## Migration from ROS2 Topics

### Before (ROS2 Topics)

```python
env = RealEnv(
    camera_names=['cam_top', 'cam_left_wrist', 'cam_right_wrist'],
    setup_left_arm=True,
    setup_right_arm=True,
    setup_left_hand=True,
    setup_right_hand=True,
    node=node,
    stereo_mode=False,
    use_zed_direct=False  # Use ROS2 topics
)
```

### After (ZED Direct)

```python
env = RealEnv(
    camera_names=[],  # Not needed for ZED direct
    setup_left_arm=True,
    setup_right_arm=True,
    setup_left_hand=True,
    setup_right_hand=True,
    node=node,
    stereo_mode=False,
    use_zed_direct=True,  # Use ZED SDK directly
    zed_configs=None      # Auto-detect
)
```

## Examples

See the following example files:

- `test_zed_direct.py` - Basic testing and validation
- `zed_direct_config_example.py` - Configuration examples
- `README_time_sync.md` - Time synchronization details

## Support

For issues with the ZED Direct Recorder:

1. Check the troubleshooting section above
2. Run the test script: `python3 test_zed_direct.py`
3. Verify ZED SDK installation: `python -c "import pyzed.sl; print('ZED SDK OK')"`
4. Check camera connections and serial numbers

For ZED SDK issues, refer to the official Stereolabs documentation.
