# LinkerHand Data Collection ROS2 Service

A ROS2-based robotics data collection system for LinkerHand dexterous hands with multiple robot arms and Intel RealSense D455 depth cameras, designed for imitation learning and robotic manipulation research.

## 🚀 Overview

This repository contains a complete ROS2 implementation of the LinkerHand Data Collection service, which provides:
- **Multi-robot arm control** via various robot SDK integrations
- **Multi-camera support** from Intel RealSense D455 depth cameras
- **Data recording** in HDF5 and video formats for imitation learning
- **Manual recording control** with easy-to-use helper scripts
- **Teleoperation support** with hand gloves and retargeting
- **Modular architecture** with separate packages for different functionalities

## 📋 Table of Contents

- [Prerequisites](#-prerequisites)
- [Architecture](#️-architecture)
- [Installation](#️-installation)
- [Configuration](#-configuration)
- [Data Collection Pipeline](#-data-collection-pipeline)
  - [Double Hand Setup](#1-double-hand-data-collection-recommended)
  - [Single Hand Setup](#2-single-hand-data-collection)
- [Manual Recording Control](#-manual-recording-control)
- [Data Structure](#-data-structure)
- [Troubleshooting](#-troubleshooting)
- [Advanced Usage](#-advanced-usage)

## 📋 Prerequisites

- **Ubuntu 24.04** (recommended)
- **ROS2 Jazzy** installed and configured
- **Python 3.12** with virtual environment support
- **LinkerHand SDK** (included in this repo)
- **Intel RealSense SDK 2.0** (for D455 depth camera integration)
- **Various robot arm SDKs** (depending on your robot configuration, e.g., Piper SDK)
- **ExHand exoskeleton glove** (optional, for data collection)

## 🏗️ Architecture

The system consists of several main packages:

### 1. `linkerhand_data_collection_srv` - Main Service Package
- **Core service orchestration** for data collection
- **Data recording** and episode management
- **Camera management** (auto-start/stop)
- **Manual recording control** via ROS2 services

### 2. `linkerhand_cl` - LinkerHand Control
- **CAN bus interface** directly connecting to LinkerHand robotic hands
- **Multi-hand configuration** support (L10, L20, L21, L7, L25, O6, L24, etc.)
- **ExHand integration** supporting exoskeleton glove data input
- **Feedback data reception** supporting CAN feedback data reading and publishing
- **Control command publishing** publishing standard topics for data collection

### 3. `exhand_read` - ExHand Exoskeleton Glove
- **Serial communication** reading exoskeleton glove data
- **Data mapping** converting sensor data to normalized values
- **ROS2 topic publishing** publishing mapped data for hand control

### 4. `realsense2_camera` - Intel RealSense Integration
- **RGB + Depth data collection** via ROS2 topics
- **Multi-camera support**
- **Auto-detection and configuration**

## 🛠️ Installation

### Step 1: Install System Dependencies

```bash
# Install Intel RealSense SDK 2.0
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

sudo apt-get install apt-transport-https
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y librealsense2 librealsense2-utils librealsense2-dev librealsense2-gl

# Install ROS2 RealSense wrapper
sudo apt install ros-jazzy-realsense2-camera

# Verify RealSense installation
rs-enumerate-devices

# Install other system dependencies
sudo apt install python3-tabulate python3-can can-utils ethtool
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

### Step 2: Setup CAN Device Rules

```bash
cd ~/linkerhand_data_collection_ws/scripts
sudo bash install_can_udev_rules.sh
```

### Step 3: Set Up Python Virtual Environment

```bash
# Create virtual environment
python3.12 -m venv ~/.venv/data_collection

# Activate environment
source ~/.venv/data_collection/bin/activate
```

### Step 4: Install Python Dependencies

```bash
cd ~/linkerhand_data_collection_ws
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### Step 5: Build the Workspace

```bash
source ~/.venv/data_collection/bin/activate
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## 🔧 Configuration

### Key Configuration Files

1. **Task Configuration**: `src/linkerhand_data_collection_srv/scripts/utils/constants.py`
   - Define task names, camera settings, episode length
   - Configure hardware setup (arms/hands)

2. **LinkerHand Configuration**: `task_config.json`
   - Configure hand model (handModel: L10, L20, L21, L7, L25, O6, L24, etc.)
   - Configure left/right hand (handSide: left/right)
   - System automatically reads configuration and applies to `linkerhand_cl` node

3. **Camera Serial Numbers**: `src/linkerhand_data_collection_srv/configs/camera_serial_numbers.yaml`
   - Map camera names to serial numbers
   - Auto-detected by `./scripts/detect_cameras.sh`

### Detect and Configure Cameras

```bash
cd ~/linkerhand_data_collection_ws/scripts
./detect_cameras.sh
cd ..
```

This will update `camera_serial_numbers.yaml` with detected RealSense cameras.

## 🎬 Data Collection Pipeline

Choose the appropriate setup based on your hardware configuration:

**Recommended**: Use the provided startup scripts, which automatically handle most configuration:

- **Single hand mode**: `bash start_linkerhand_piper_grasp.sh`
- **Double hand mode**: `bash start_double_linkerhand_grasp.sh`

These scripts automatically:
- Read `task_config.json` configuration
- Detect and configure CAN interfaces
- Launch LinkerHand control nodes
- Launch data collection service

### 1. Double Hand Data Collection (Recommended)

This setup uses dual LinkerHand devices, supporting ExHand exoskeleton gloves or traditional data gloves.

**Quick Start** (Recommended):
```bash
bash start_double_linkerhand_grasp.sh
```

**Manual Launch** (if customization needed):

#### Terminal 1: Environment Setup

```bash
source ~/.venv/data_collection/bin/activate
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Verify CAN interfaces
ip link show type can
```

#### Terminal 2: Setup CAN Interfaces

```bash
# Install CAN rules (first time only)
sudo bash scripts/install_can_udev_rules.sh

# Setup dual hands on CAN interfaces
ros2 launch linkerhand_data_collection_srv setup_can.launch.py \
  hand_left_enable:=true  hand_left_if:=can2  hand_left_name:=hand_left  hand_left_bitrate:=1000000 \
  hand_right_enable:=true hand_right_if:=can3 hand_right_name:=hand_right hand_right_bitrate:=1000000
```

**Note**: Adjust `can2` and `can3` to match your actual CAN interface names. Use `ip link show type can` to verify.

#### Terminal 3: Launch ExHand Exoskeleton Glove (if using)

```bash
# If using ExHand exoskeleton glove, launch data reading node
ros2 launch exhand_read exhand.launch.py \
    port:=/dev/ttyUSB0 \
    baudrate:=1152000 \
    enable_mapping_push:=true \
    enable_sensor_push:=true
```

**Note**: If using traditional data gloves, this step can be skipped.

#### Terminal 4: Launch LinkerHand Control Node

```bash
# Use quick start script (auto-detect CAN interfaces and launch)
bash scripts/quick_start_hand.sh <sudo_password>

# Or manually launch (requires CAN interface configuration first)
ros2 launch linkerhand_cl linker_hand_double.launch.py \
    left_can:=can3 \
    right_can:=can1 \
    enable_can:=true \
    can_bitrate:=1000000
```

This launches the dual-hand LinkerHand control node, automatically reading configuration from `task_config.json`.

#### Terminal 5: Launch Data Collection Service

```bash
ros2 run linkerhand_data_collection_srv linkerhand_data_collection.py
```

This service auto-starts cameras and provides recording control.

#### Terminal 6: Create Task and Record Data

```bash
# Create a new data collection task
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"create_task\", \"id\": 121212, 
    \"params\": {\"task_name\":\"double_linkerhand_grasp\",
                 \"task_id\":2,
                 \"task_configs_name\":\"double_linkerhand_grasp\"}}'"

# Use the manual recording control helper script (RECOMMENDED)
# Start recording
./manual_recording_control.sh start double_linkerhand_grasp

# ... Perform the task demonstration ...

# Stop recording when done
./manual_recording_control.sh stop

# Check recording status
./manual_recording_control.sh status
```

#### Optional: Visualization

```bash
# Terminal 7: Launch RViz for visualization
rviz2

# Or launch multi-camera viewer
ros2 launch linkerhand_data_collection_srv multi_camera_launch.py
```

---

### 2. Single Hand Data Collection

This setup uses a single LinkerHand with one robot arm.

**Quick Start** (Recommended):
```bash
bash start_linkerhand_piper_grasp.sh
```

**Manual Launch** (if customization needed):

#### Terminal 1: Environment Setup

```bash
source ~/.venv/data_collection/bin/activate
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Verify CAN interfaces
ip link show type can
```

#### Terminal 2: Setup CAN Interfaces

```bash
# Install CAN rules (first time only)
sudo bash scripts/install_can_udev_rules.sh

# Example: Setup arm on can0, hand on can1
ros2 launch linkerhand_data_collection_srv setup_can.launch.py \
  arm_right_enable:=true  arm_right_if:=can0  arm_right_name:=arm_right  arm_right_bitrate:=1000000 \
  hand_right_enable:=true hand_right_if:=can1 hand_right_name:=hand_right hand_right_bitrate:=1000000
```

#### Terminal 3: Launch ExHand Exoskeleton Glove (if using)

```bash
# If using ExHand exoskeleton glove, launch data reading node
ros2 launch exhand_read exhand.launch.py \
    port:=/dev/ttyUSB0 \
    baudrate:=1152000 \
    enable_mapping_push:=true
```

**Note**: If using traditional data gloves, this step can be skipped.

#### Terminal 4: Launch LinkerHand Control Node

```bash
# Use quick start script (auto-detect CAN interfaces and launch)
bash scripts/quick_start_hand.sh <sudo_password>

# Or manually launch (requires CAN interface configuration first)
ros2 launch linkerhand_cl linker_hand.launch.py \
    hand_type:=left \
    hand_joint:=L10 \
    can:=can1 \
    enable_can:=true \
    can_bitrate:=1000000
```

This launches the single-hand LinkerHand control node, automatically reading configuration from `task_config.json`.

#### Terminal 5: Launch Data Collection Service

```bash
ros2 run linkerhand_data_collection_srv linkerhand_data_collection.py
```

#### Terminal 6: Create Task and Record Data

```bash
# Create task
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"create_task\", \"id\": 121212, 
    \"params\": {\"task_name\":\"linkerhand_piper_grasp\",
                 \"task_id\":1,
                 \"task_configs_name\":\"linkerhand_piper_grasp\"}}'"

# Start recording
./manual_recording_control.sh start linkerhand_piper_grasp

# ... Perform demonstrations ...

# Stop recording
./manual_recording_control.sh stop
```

---

## 📹 Manual Recording Control

The `manual_recording_control.sh` script provides an easy interface for controlling data recording.

### Basic Usage

```bash
# Start recording for a task
./manual_recording_control.sh start <task_name> [use_timestamp]

# Stop recording
./manual_recording_control.sh stop

# Check recording status
./manual_recording_control.sh status

# Camera management
./manual_recording_control.sh camera start <task_name>
./manual_recording_control.sh camera stop
./manual_recording_control.sh camera status

# Show help
./manual_recording_control.sh help
```

### Examples

```bash
# Start recording with automatic episode numbering
./manual_recording_control.sh start double_linkerhand_grasp

# Start recording with timestamp-based naming
./manual_recording_control.sh start double_linkerhand_grasp true

# Stop current recording
./manual_recording_control.sh stop

# Check if recording is active
./manual_recording_control.sh status

# Manually start cameras for a specific task
./manual_recording_control.sh camera start double_linkerhand_grasp

# Stop cameras when done
./manual_recording_control.sh camera stop
```

### Recording Workflow

1. **Prepare**: Make sure all hardware is connected and services are running
2. **Start**: `./manual_recording_control.sh start <task_name>`
3. **Demonstrate**: Perform the task demonstration with teleoperation
4. **Stop**: `./manual_recording_control.sh stop` when the demonstration is complete
5. **Verify**: Check the episode was saved in `collection_data/<task_name>/session_<timestamp>/`

---

## 🗂️ Data Structure

Collected data is organized as follows:

```
collection_data/
└── <task_name>/
    └── session_<timestamp>/
        ├── episode_000001/
        │   ├── telemetry.npz              # Joint positions, velocities, actions
        │   ├── cameras/
        │   │   ├── cam_top.mp4            # H.264 compressed video
        │   │   └── cam_top.timestamps.npy # Frame timestamps
        │   ├── camera_info.json           # Camera intrinsics
        │   ├── manifest.json              # Episode metadata
        │   └── metadata.json              # Episode statistics
        ├── episode_000002/
        └── ...
```

### Data Files

- **telemetry.npz**: NumPy compressed archive containing:
  - `qpos`: Joint positions (timesteps, joints)
  - `qvel`: Joint velocities (timesteps, joints)
  - `effort`: Joint efforts (timesteps, joints)
  - `actions`: Action commands (timesteps, joints)
  - `tactile_*`: Tactile sensor data (if available)

- **cameras/*.mp4**: H.264 compressed videos (30 FPS)
- **camera_info.json**: Camera intrinsics and extrinsics
- **manifest.json**: Episode metadata (task, duration, hardware)
- **metadata.json**: Episode statistics (min/max values, data quality)

---

## 🎯 Episode Naming Options

The system supports three episode naming strategies:

### 1. Automatic Episode Numbering (Default)
```bash
./manual_recording_control.sh start double_linkerhand_grasp
# Creates: episode_000001, episode_000002, etc.
```
- Automatically finds next available number
- No manual tracking needed
- **Recommended for most users**

### 2. Explicit Episode Numbering
```bash
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"gather_hdf5\", \"id\": 121212, 
    \"params\": {\"task_name\":\"double_linkerhand_grasp\",\"episode_idx\":5}}'"
# Creates: episode_000005
```

### 3. Timestamp-Based Naming
```bash
./manual_recording_control.sh start double_linkerhand_grasp true
# Creates: timestamp_20250117_143052_123
```
- Guaranteed unique names
- Includes precise timing information

---

## 🔍 System Monitoring

### Check Running Nodes

```bash
ros2 node list
# Expected nodes:
# - /linkerhand_data_collection_server
# - /hand_control_node (or hand_control_node_left/right)
# - /exhand_node (if using ExHand)
# - /realsense2_camera_node
```

### Check Topics

```bash
ros2 topic list

# Key topics:
# Cameras:
ros2 topic echo /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info

# LinkerHand state:
ros2 topic echo /cb_left_hand_state
ros2 topic echo /cb_right_hand_state

# LinkerHand control commands:
ros2 topic echo /cb_left_hand_control_cmd
ros2 topic echo /cb_right_hand_control_cmd

# ExHand data (if using):
ros2 topic echo /exhand/mapping_data_left
ros2 topic echo /exhand/mapping_data_right
```

### Monitor Recording

```bash
# Check recording status
./manual_recording_control.sh status

# Watch data directory
watch -n 1 "ls -lh collection_data/double_linkerhand_grasp/session_*/episode_*/"

# Monitor service logs
ros2 run linkerhand_data_collection_srv linkerhand_data_collection.py
```

---

## 🐛 Troubleshooting

### Camera Issues

**Problem**: Camera not detected
```bash
# Check if camera is connected
rs-enumerate-devices

# Check if topics are publishing
ros2 topic list | grep camera

# Restart camera service
./manual_recording_control.sh camera stop
./manual_recording_control.sh camera start <task_name>
```

**Problem**: Camera topics not publishing
```bash
# Check RealSense node
ros2 node list | grep realsense

# Manually launch camera
ros2 launch realsense2_camera rs_launch.py
```

### CAN Interface Issues

**Problem**: CAN devices not found
```bash
# Check CAN interfaces
ip link show type can

# Check if interfaces are up
ip -details link show can0

# Restart CAN setup
sudo bash scripts/install_can_udev_rules.sh
ros2 launch linkerhand_data_collection_srv setup_can.launch.py ...
```

**Problem**: LinkerHand not responding
```bash
# Check if control node is running
ros2 node list | grep hand_control

# Check joint states
ros2 topic echo /cb_left_hand_state
ros2 topic echo /cb_right_hand_state

# Check control commands
ros2 topic echo /cb_left_hand_control_cmd

# Restart LinkerHand control node
ros2 launch linkerhand_cl linker_hand_double.launch.py
```

### Recording Issues

**Problem**: Recording not starting
```bash
# Check if task exists
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"get_task_list\", \"id\": 121212, \"params\": {}}'"

# Check recording status
./manual_recording_control.sh status

# Check service logs for errors
# (Look in the terminal running linkerhand_data_collection.py)
```

**Problem**: No data in episodes
```bash
# Verify all topics are publishing
ros2 topic list
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /cb_left_hand_state
ros2 topic hz /cb_left_hand_control_cmd

# Check if recorders are initialized
# (Look for initialization messages in service logs)
```

### ExHand Exoskeleton Glove Issues

**Problem**: ExHand data not responding
```bash
# Check if ExHand node is running
ros2 node list | grep exhand

# Check serial port connection
ls -l /dev/ttyUSB*

# Check ExHand topics
ros2 topic echo /exhand/mapping_data_left
ros2 topic echo /exhand/status

# Restart ExHand node
ros2 launch exhand_read exhand.launch.py port:=/dev/ttyUSB0 baudrate:=1152000
```

**Problem**: Control commands not publishing
```bash
# Check if linkerhand_cl node is running
ros2 node list | grep hand_control

# Check control command topics
ros2 topic echo /cb_left_hand_control_cmd
ros2 topic echo /cb_right_hand_control_cmd

# Check if ExHand mapping data is publishing
ros2 topic hz /exhand/mapping_data_left
```

---

## 🔧 Advanced Usage

### Custom Task Configuration

Edit `src/linkerhand_data_collection_srv/scripts/utils/constants.py`:

```python
TASK_CONFIGS = {
    'my_custom_task': {
        'dataset_dir': DATA_DIR + '/my_custom_task/all_episodes',
        'num_episodes': 100,
        'episode_len': 500,  # Maximum timesteps
        'camera_names': ['cam_top', 'cam_wrist'],
        'stereo_mode': False,
        'setup_left_arm': False,
        'setup_right_arm': True,
        'setup_left_hand': False,
        'setup_right_hand': True,
        'camera_type': 'intel_d455',
        'use_video_compression': True,
        'video_quality': 'high'
    }
}
```

### Multi-Camera Setup

```bash
# Detect all connected cameras
cd scripts
./detect_cameras.sh

# Launch with multiple cameras
ros2 launch linkerhand_data_collection_srv multi_camera_launch.py

# View in RViz
rviz2
```

### Direct ROS2 Service Calls

```bash
# Create task
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"create_task\", \"id\": 121212, 
    \"params\": {\"task_name\":\"my_task\",\"task_id\":1,
                 \"task_configs_name\":\"my_task\"}}'"

# Start manual recording
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"start_manual_recording\", \"id\": 121212, 
    \"params\": {\"task_name\":\"my_task\",\"use_timestamp\":true}}'"

# Stop manual recording
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"stop_manual_recording\", \"id\": 121212, 
    \"params\": {}}'"

# Get recording status
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"recording_status\", \"id\": 121212, 
    \"params\": {}}'"

# Delete task
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"delete_task\", \"id\": 121212, 
    \"params\": {\"task_name\":\"my_task\"}}'"
```

### GUI Control for LinkerHand

For manual control and debugging:

```bash
ros2 launch gui_control gui_control.launch.py
```

This provides a graphical interface to control LinkerHand positions.

---

## 📚 Additional Resources

- **LinkerHand Control Package Documentation**: `src/linkerhand_cl/README.md`
- **RealSense User Guide**: `realsense/Intel_RealSense_D455_使用指南.md`
- **ExHand Integration Guide**: See `EXHAND_LINKERHAND_INTEGRATION.md` (if available)
- **Data Structure**: `data_structure.txt`
- **Ubuntu 24.04 Teleop Setup**: https://hs7ghlauag.feishu.cn/docx/JLAsdoZnBohFtHx8NE2cA13hnLd

---

## 🤝 Contributing

If you encounter issues or have improvements:

1. Check existing issues and documentation
2. Test your changes thoroughly
3. Update configuration examples
4. Document new features

---

## 📝 License

See LICENSE files in individual packages for details.

---

## 🎓 Quick Start Checklist

- [ ] Install system dependencies (RealSense SDK, ROS2, CAN utils)
- [ ] Setup Python virtual environment
- [ ] Build the workspace with `colcon build`
- [ ] Install CAN udev rules
- [ ] Detect cameras with `./scripts/detect_cameras.sh`
- [ ] Configure task in `constants.py`
- [ ] Follow the [Data Collection Pipeline](#-data-collection-pipeline)
- [ ] Use `./manual_recording_control.sh` for easy recording
- [ ] Verify data in `collection_data/<task_name>/`

---

**Happy Data Collecting! 🎉**

For questions or issues, refer to the [Troubleshooting](#-troubleshooting) section or check the documentation files.
