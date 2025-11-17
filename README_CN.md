# LinkerHand 数据采集 ROS2 服务

基于 ROS2 的机器人数据采集系统，适用于 LinkerHand 灵巧手配合多种机械臂和 Intel RealSense D455 深度相机，专为模仿学习和机器人操作研究设计。

## 🚀 概述

本仓库包含 LinkerHand 数据采集服务的完整 ROS2 实现，提供以下功能：
- **多机械臂控制** 通过各种机器人 SDK 集成
- **多相机支持** 来自 Intel RealSense D455 深度相机
- **数据记录** 支持 HDF5 和视频格式，用于模仿学习
- **手动录制控制** 提供易用的辅助脚本
- **遥操作支持** 支持手套和重定向
- **模块化架构** 针对不同功能的独立软件包

## 📋 目录

- [前置条件](#-前置条件)
- [系统架构](#️-系统架构)
- [安装](#️-安装)
- [配置](#-配置)
- [数据采集流程](#-数据采集流程)
  - [双手设置](#1-双手数据采集推荐)
  - [单手设置](#2-单手数据采集)
- [手动录制控制](#-手动录制控制)
- [数据结构](#️-数据结构)
- [故障排除](#-故障排除)
- [高级用法](#-高级用法)

## 📋 前置条件

- **Ubuntu 24.04**（推荐）
- **ROS2 Jazzy** 已安装并配置
- **Python 3.12** 支持虚拟环境
- **LinkerHand SDK**（包含在本仓库中）
- **Intel RealSense SDK 2.0**（用于 D455 深度相机集成）
- **各种机械臂 SDK**（根据您的机器人配置，如Piper SDK）
- **ExHand外骨骼手套**（可选，用于数据采集）

## 🏗️ 系统架构

系统由几个主要软件包组成：

### 1. `linkerhand_data_collection_srv` - 主服务包
- **核心服务编排** 用于数据采集
- **数据记录** 和回合管理
- **相机管理**（自动启动/停止）
- **手动录制控制** 通过 ROS2 服务

### 2. `linkerhand_cl` - LinkerHand 控制
- **CAN总线接口** 直接连接 LinkerHand 灵巧手
- **多手配置** 支持（L10、L20、L21、L7、L25、O6、L24 等）
- **ExHand集成** 支持外骨骼手套数据输入
- **反馈数据接收** 支持CAN反馈数据读取和发布
- **控制命令发布** 发布标准话题用于数据采集

### 3. `exhand_read` - ExHand外骨骼手套
- **串口通信** 读取外骨骼手套数据
- **数据映射** 将传感器数据映射为归一化值
- **ROS2话题发布** 发布映射数据供灵巧手控制使用

### 4. `realsense2_camera` - Intel RealSense 集成
- **RGB + 深度数据采集** 通过 ROS2 话题
- **多相机支持**
- **自动检测和配置**

## 🛠️ 安装

### 步骤 1：安装系统依赖

```bash
# 安装 Intel RealSense SDK 2.0
sudo mkdir -p /etc/apt/keyrings
sudo apt install curl
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

sudo apt-get install apt-transport-https
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt update
sudo apt install -y librealsense2 librealsense2-utils librealsense2-dev librealsense2-gl

sudo apt install librealsense2-utils

sudo apt install python3-tabulate python3-can can-utils ethtool

# 安装ros2 jazzy
# 设置 UTF-8 本地化
locale   # 看看是否是 UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale   # 再次确认

# 启用 Universe 仓库
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# 配置 ROS 2 APT 源
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo curl -sSL http://packages.ros.org/ros2/ubuntu/gpg | sudo tee /etc/apt/trusted.gpg.d/ros.asc
echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list

sudo apt update


# 安装 ROS 2
sudo apt -y upgrade
sudo apt install -y ros-jazzy-desktop

# 安装完成后，把 ROS 2 加入环境

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc


# 验证ros2 安装
ros2 run demo_nodes_cpp talker
# 若另开一个终端运行命令，能看到消息通信，就完全 OK
ros2 run demo_nodes_py listener

# 期望输出: jazzy
echo $ROS_DISTRO

# 装开发常用工具（colcon / rosdep / vcs 等）
sudo apt update
sudo apt install -y ros-dev-tools

# 初始化 rosdep（第一次用需要）
sudo rosdep init || true   # 若已初始化会提示已存在，忽略即可
rosdep update

sudo apt install ros-jazzy-rmw-cyclonedds-cpp

# 设置环境
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> ~/.bashrc
echo "export unset ROS_LOCALHOST_ONLY" >> ~/.bashrc
source ~/.bashrc

# 安装 ROS2 RealSense 封装
sudo apt install ros-jazzy-realsense2-camera

# 验证 RealSense 安装
rs-enumerate-devices

# 安装ffmpeg
sudo apt install ffmpeg

sudo apt install nlohmann-json3-dev

```


### 步骤 3：设置 Python 虚拟环境

```bash
sudo apt install python3.12-venv
# 创建虚拟环境
python3.12 -m venv ~/.venv/data_collection

echo "alias data_collection='source ~/.venv/data_collection/bin/activate'" >> ~/.bashrc
source ~/.bashrc

# 激活环境
data_collection
```

### 步骤 4：安装 Python 依赖

```bash
cd ~/linkerhand_data_collection_ws
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple -r requirements.txt
```



### 步骤 5：编译工作空间

```bash
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash && source install/setup.bash

echo "source /opt/ros/jazzy/setup.bash && source install/setup.bash" >> ~/.venv/data_collection/bin/activate
deactivate
data_collection 
```

## 🔧 配置

### 关键配置文件

1. **任务配置**：`src/linkerhand_data_collection_srv/scripts/utils/constants.py`
   - 定义任务名称、相机设置、回合长度
   - 配置硬件设置（机械臂/灵巧手）

2. **LinkerHand 配置**：`task_config.json`
   - 配置手型（handModel: L10、L20、L21、L7、L25、O6、L24 等）
   - 配置左右手（handSide: left/right）
   - 系统自动读取配置并应用到 `linkerhand_cl` 节点

3. **相机序列号**：`src/linkerhand_data_collection_srv/configs/camera_serial_numbers.yaml`
   - 将相机名称映射到序列号
   - 通过 `./scripts/detect_cameras.sh` 自动检测

### 检测和配置相机

```bash
cd ~/linkerhand_data_collection_ws/scripts
./detect_cameras.sh
cd ..
```

这将使用检测到的 RealSense 相机更新 `camera_serial_numbers.yaml`。

## 🎬 数据采集流程

根据您的硬件配置选择合适的设置：

**推荐方式**：使用提供的启动脚本，它们会自动处理大部分配置：

- **单手模式**：`bash start_linkerhand_piper_grasp.sh`
- **双手模式**：`bash start_double_linkerhand_grasp.sh`

这些脚本会自动：
- 读取 `task_config.json` 配置
- 检测和配置CAN接口
- 启动LinkerHand控制节点
- 启动数据采集服务

### 1. 双手数据采集（推荐）

此设置使用双 LinkerHand 设备，支持ExHand外骨骼手套或传统数据手套。

**快速启动**（推荐）：
```bash
bash start_double_linkerhand_grasp.sh
```

**手动启动**（如需自定义）：

#### 终端 1：环境设置

```bash
source ~/.venv/data_collection/bin/activate
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# 验证 CAN 接口
ip link show type can
```

#### 终端 2：设置 CAN 接口

```bash
# 安装 CAN 规则（仅首次）
sudo bash scripts/install_can_udev_rules.sh

# 在 CAN 接口上设置双手
ros2 launch linkerhand_data_collection_srv setup_can.launch.py \
  hand_left_enable:=true  hand_left_if:=can2  hand_left_name:=hand_left  hand_left_bitrate:=1000000 \
  hand_right_enable:=true hand_right_if:=can3 hand_right_name:=hand_right hand_right_bitrate:=1000000
```

**注意**：调整 `can2` 和 `can3` 以匹配您实际的 CAN 接口名称。使用 `ip link show type can` 进行验证。

#### 终端 3：启动ExHand外骨骼手套（如果使用）

```bash
# 如果使用ExHand外骨骼手套，启动数据读取节点
ros2 launch exhand_read exhand.launch.py \
    port:=/dev/ttyUSB0 \
    baudrate:=1152000 \
    enable_mapping_push:=true \
    enable_sensor_push:=true
```

**注意**：如果使用传统数据手套，此步骤可跳过。

#### 终端 4：启动LinkerHand控制节点

```bash
# 使用快速启动脚本（自动检测CAN接口并启动）
bash scripts/quick_start_hand.sh <sudo密码>

# 或手动启动（需要先配置CAN接口）
ros2 launch linkerhand_cl linker_hand_double.launch.py \
    left_can:=can3 \
    right_can:=can1 \
    enable_can:=true \
    can_bitrate:=1000000
```

这将启动双手的LinkerHand控制节点，自动从 `task_config.json` 读取配置。

#### 终端 5：启动数据采集服务

```bash
ros2 run linkerhand_data_collection_srv linkerhand_data_collection.py
```

此服务自动启动相机并提供录制控制。

#### 终端 6：创建任务并录制数据

```bash
# 创建新的数据采集任务
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"create_task\", \"id\": 121212, 
    \"params\": {\"task_name\":\"double_linkerhand_grasp\",
                 \"task_id\":2,
                 \"task_configs_name\":\"double_linkerhand_grasp\"}}'"

# 使用手动录制控制辅助脚本（推荐）
# 开始录制
./manual_recording_control.sh start double_linkerhand_grasp

# ... 执行任务演示 ...

# 完成后停止录制
./manual_recording_control.sh stop

# 检查录制状态
./manual_recording_control.sh status
```

#### 可选：可视化

```bash
# 终端 7：启动 RViz 进行可视化
rviz2

# 或启动多相机查看器
ros2 launch linkerhand_data_collection_srv multi_camera_launch.py
```

---

### 2. 单手数据采集

此设置使用单个 LinkerHand 和一个机械臂。

**快速启动**（推荐）：
```bash
bash start_linkerhand_piper_grasp.sh
```

**手动启动**（如需自定义）：

#### 终端 1：环境设置

```bash
source ~/.venv/data_collection/bin/activate
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# 验证 CAN 接口
ip link show type can
```

#### 终端 2：设置 CAN 接口

```bash
# 安装 CAN 规则（仅首次）
sudo bash scripts/install_can_udev_rules.sh

# 示例：机械臂在 can0，灵巧手在 can1
ros2 launch linkerhand_data_collection_srv setup_can.launch.py \
  arm_right_enable:=true  arm_right_if:=can0  arm_right_name:=arm_right  arm_right_bitrate:=1000000 \
  hand_right_enable:=true hand_right_if:=can1 hand_right_name:=hand_right hand_right_bitrate:=1000000
```

#### 终端 3：启动ExHand外骨骼手套（如果使用）

```bash
# 如果使用ExHand外骨骼手套，启动数据读取节点
ros2 launch exhand_read exhand.launch.py \
    port:=/dev/ttyUSB0 \
    baudrate:=1152000 \
    enable_mapping_push:=true
```

**注意**：如果使用传统数据手套，此步骤可跳过。

#### 终端 4：启动LinkerHand控制节点

```bash
# 使用快速启动脚本（自动检测CAN接口并启动）
bash scripts/quick_start_hand.sh <sudo密码>

# 或手动启动（需要先配置CAN接口）
ros2 launch linkerhand_cl linker_hand.launch.py \
    hand_type:=left \
    hand_joint:=L10 \
    can:=can1 \
    enable_can:=true \
    can_bitrate:=1000000
```

这将启动单手的LinkerHand控制节点，自动从 `task_config.json` 读取配置。

#### 终端 5：启动数据采集服务

```bash
ros2 run linkerhand_data_collection_srv linkerhand_data_collection.py
```

#### 终端 6：创建任务并录制数据

```bash
# 创建任务
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"create_task\", \"id\": 121212, 
    \"params\": {\"task_name\":\"linkerhand_piper_grasp\",
                 \"task_id\":1,
                 \"task_configs_name\":\"linkerhand_piper_grasp\"}}'"

# 开始录制
./manual_recording_control.sh start linkerhand_piper_grasp

# ... 执行演示 ...

# 停止录制
./manual_recording_control.sh stop
```

---

## 📹 手动录制控制

`manual_recording_control.sh` 脚本为控制数据录制提供了便捷的界面。

### 基本用法

```bash
# 为任务开始录制
./manual_recording_control.sh start <任务名称> [use_timestamp]

# 停止录制
./manual_recording_control.sh stop

# 检查录制状态
./manual_recording_control.sh status

# 相机管理
./manual_recording_control.sh camera start <任务名称>
./manual_recording_control.sh camera stop
./manual_recording_control.sh camera status

# 显示帮助
./manual_recording_control.sh help
```

### 示例

```bash
# 使用自动回合编号开始录制
./manual_recording_control.sh start double_linkerhand_grasp

# 使用基于时间戳的命名开始录制
./manual_recording_control.sh start double_linkerhand_grasp true

# 停止当前录制
./manual_recording_control.sh stop

# 检查是否正在录制
./manual_recording_control.sh status

# 手动为特定任务启动相机
./manual_recording_control.sh camera start double_linkerhand_grasp

# 完成后停止相机
./manual_recording_control.sh camera stop
```

### 录制工作流程

1. **准备**：确保所有硬件已连接且服务正在运行
2. **开始**：`./manual_recording_control.sh start <任务名称>`
3. **演示**：使用遥操作执行任务演示
4. **停止**：演示完成后 `./manual_recording_control.sh stop`
5. **验证**：检查回合是否保存在 `collection_data/<任务名称>/session_<时间戳>/`

---

## 🗂️ 数据结构

收集的数据组织如下：

```
collection_data/
└── <任务名称>/
    └── session_<时间戳>/
        ├── episode_000001/
        │   ├── telemetry.npz              # 关节位置、速度、动作
        │   ├── cameras/
        │   │   ├── cam_top.mp4            # H.264 压缩视频
        │   │   └── cam_top.timestamps.npy # 帧时间戳
        │   ├── camera_info.json           # 相机内参
        │   ├── manifest.json              # 回合元数据
        │   └── metadata.json              # 回合统计
        ├── episode_000002/
        └── ...
```

### 数据文件

- **telemetry.npz**：NumPy 压缩归档，包含：
  - `qpos`：关节位置（时间步，关节）
  - `qvel`：关节速度（时间步，关节）
  - `effort`：关节力矩（时间步，关节）
  - `actions`：动作命令（时间步，关节）
  - `tactile_*`：触觉传感器数据（如果可用）

- **cameras/*.mp4**：H.264 压缩视频（30 FPS）
- **camera_info.json**：相机内参和外参
- **manifest.json**：回合元数据（任务、持续时间、硬件）
- **metadata.json**：回合统计（最小/最大值、数据质量）

---

## 🎯 回合命名选项

系统支持三种回合命名策略：

### 1. 自动回合编号（默认）
```bash
./manual_recording_control.sh start double_linkerhand_grasp
# 创建：episode_000001、episode_000002 等
```
- 自动找到下一个可用编号
- 无需手动跟踪
- **推荐大多数用户使用**

### 2. 显式回合编号
```bash
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"gather_hdf5\", \"id\": 121212, 
    \"params\": {\"task_name\":\"double_linkerhand_grasp\",\"episode_idx\":5}}'"
# 创建：episode_000005
```

### 3. 基于时间戳的命名
```bash
./manual_recording_control.sh start double_linkerhand_grasp true
# 创建：timestamp_20250117_143052_123
```
- 保证唯一名称
- 包含精确的时间信息

---

## 🔍 系统监控

### 检查运行的节点

```bash
ros2 node list
# 预期节点：
# - /linkerhand_data_collection_server
# - /hand_control_node（或 hand_control_node_left/right）
# - /exhand_node（如果使用ExHand）
# - /realsense2_camera_node
```

### 检查话题

```bash
ros2 topic list

# 关键话题：
# 相机：
ros2 topic echo /camera/camera/color/image_raw
ros2 topic echo /camera/camera/color/camera_info

# LinkerHand状态：
ros2 topic echo /cb_left_hand_state
ros2 topic echo /cb_right_hand_state

# LinkerHand控制命令：
ros2 topic echo /cb_left_hand_control_cmd
ros2 topic echo /cb_right_hand_control_cmd

# ExHand数据（如果使用）：
ros2 topic echo /exhand/mapping_data_left
ros2 topic echo /exhand/mapping_data_right
```

### 监控录制

```bash
# 检查录制状态
./manual_recording_control.sh status

# 观察数据目录
watch -n 1 "ls -lh collection_data/double_linkerhand_grasp/session_*/episode_*/"

# 监控服务日志
ros2 run linkerhand_data_collection_srv linkerhand_data_collection.py
```

---

## 🐛 故障排除

### 相机问题

**问题**：未检测到相机
```bash
# 检查相机是否已连接
rs-enumerate-devices

# 检查话题是否在发布
ros2 topic list | grep camera

# 重启相机服务
./manual_recording_control.sh camera stop
./manual_recording_control.sh camera start <任务名称>
```

**问题**：相机话题未发布
```bash
# 检查 RealSense 节点
ros2 node list | grep realsense

# 手动启动相机
ros2 launch realsense2_camera rs_launch.py
```

### CAN 接口问题

**问题**：未找到 CAN 设备
```bash
# 检查 CAN 接口
ip link show type can

# 检查接口是否启动
ip -details link show can0

# 重启 CAN 设置
sudo bash scripts/install_can_udev_rules.sh
ros2 launch linkerhand_data_collection_srv setup_can.launch.py ...
```

**问题**：LinkerHand 无响应
```bash
# 检查控制节点是否运行
ros2 node list | grep hand_control

# 检查关节状态
ros2 topic echo /cb_left_hand_state
ros2 topic echo /cb_right_hand_state

# 检查控制命令
ros2 topic echo /cb_left_hand_control_cmd

# 重启 LinkerHand 控制节点
ros2 launch linkerhand_cl linker_hand_double.launch.py
```

### 录制问题

**问题**：录制未开始
```bash
# 检查任务是否存在
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"get_task_list\", \"id\": 121212, \"params\": {}}'"

# 检查录制状态
./manual_recording_control.sh status

# 检查服务日志中的错误
#（查看运行 linkerhand_data_collection.py 的终端）
```

**问题**：回合中没有数据
```bash
# 验证所有话题是否在发布
ros2 topic list
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /cb_left_hand_state
ros2 topic hz /cb_left_hand_control_cmd

# 检查记录器是否已初始化
#（查看服务日志中的初始化消息）
```

### ExHand外骨骼手套问题

**问题**：ExHand数据无响应
```bash
# 检查ExHand节点是否运行
ros2 node list | grep exhand

# 检查串口连接
ls -l /dev/ttyUSB*

# 检查ExHand话题
ros2 topic echo /exhand/mapping_data_left
ros2 topic echo /exhand/status

# 重启ExHand节点
ros2 launch exhand_read exhand.launch.py port:=/dev/ttyUSB0 baudrate:=1152000
```

**问题**：控制命令未发布
```bash
# 检查linkerhand_cl节点是否运行
ros2 node list | grep hand_control

# 检查控制命令话题
ros2 topic echo /cb_left_hand_control_cmd
ros2 topic echo /cb_right_hand_control_cmd

# 检查ExHand映射数据是否在发布
ros2 topic hz /exhand/mapping_data_left
```

---

## 🔧 高级用法

### 自定义任务配置

编辑 `src/linkerhand_data_collection_srv/scripts/utils/constants.py`：

```python
TASK_CONFIGS = {
    'my_custom_task': {
        'dataset_dir': DATA_DIR + '/my_custom_task/all_episodes',
        'num_episodes': 100,
        'episode_len': 500,  # 最大时间步
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

### 多相机设置

```bash
# 检测所有连接的相机
cd scripts
./detect_cameras.sh

# 使用多个相机启动
ros2 launch linkerhand_data_collection_srv multi_camera_launch.py

# 在 RViz 中查看
rviz2
```

### 直接 ROS2 服务调用

```bash
# 创建任务
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"create_task\", \"id\": 121212, 
    \"params\": {\"task_name\":\"my_task\",\"task_id\":1,
                 \"task_configs_name\":\"my_task\"}}'"

# 开始手动录制
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"start_manual_recording\", \"id\": 121212, 
    \"params\": {\"task_name\":\"my_task\",\"use_timestamp\":true}}'"

# 停止手动录制
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"stop_manual_recording\", \"id\": 121212, 
    \"params\": {}}'"

# 获取录制状态
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"recording_status\", \"id\": 121212, 
    \"params\": {}}'"

# 删除任务
ros2 service call /linkerhand_data_collection_srv \
  linkerhand_data_collection_srv/srv/Internal \
  "req: '{\"method\": \"delete_task\", \"id\": 121212, 
    \"params\": {\"task_name\":\"my_task\"}}'"
```

### LinkerHand GUI 控制

用于手动控制和调试：

```bash
ros2 launch gui_control gui_control.launch.py
```

这提供了一个图形界面来控制 LinkerHand 位置。

---

## 📚 其他资源

- **LinkerHand控制包文档**：`src/linkerhand_cl/README.md`
- **RealSense 用户指南**：`realsense/Intel_RealSense_D455_使用指南.md`
- **ExHand集成说明**：查看 `EXHAND_LINKERHAND_INTEGRATION.md`（如存在）
- **数据结构**：`data_structure.txt`
- **Ubuntu 24.04 遥操作设置**：https://hs7ghlauag.feishu.cn/docx/JLAsdoZnBohFtHx8NE2cA13hnLd

---

## 🤝 贡献

如果您遇到问题或有改进建议：

1. 检查现有问题和文档
2. 彻底测试您的更改
3. 更新配置示例
4. 记录新功能

---

## 📝 许可证

有关详细信息，请参阅各个软件包中的 LICENSE 文件。

---

## 🎓 快速入门清单

- [ ] 安装系统依赖（RealSense SDK、ROS2、CAN 工具）
- [ ] 设置 Python 虚拟环境
- [ ] 使用 `colcon build` 编译工作空间
- [ ] 安装 CAN udev 规则
- [ ] 使用 `./scripts/detect_cameras.sh` 检测相机
- [ ] 在 `constants.py` 中配置任务
- [ ] 按照[数据采集流程](#-数据采集流程)操作
- [ ] 使用 `./manual_recording_control.sh` 便捷录制
- [ ] 在 `collection_data/<任务名称>/` 中验证数据

---

**祝您数据采集愉快！🎉**

如有问题，请参阅[故障排除](#-故障排除)部分或查看文档文件。

