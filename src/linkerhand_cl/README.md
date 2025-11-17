# LinkerHand_CL

LinkerHand控制功能包，用于订阅EXHand发布的映射数据话题，通过CAN总线控制灵巧手。

## 功能说明

本功能包提供了一个ROS2节点 `hand_control_node`，用于：

- 订阅EXHand发布的左右手映射数据话题（`exhand/mapping_data_right` 和 `exhand/mapping_data_left`）
- 将映射数据转换为手部控制命令
- 通过CAN总线（socketcan）发送控制数据到灵巧手设备
- 支持L10/L20/L21三种通信协议
- 支持左右手独立控制

### 订阅的话题

节点订阅两个独立的话题，分别控制左右手：

- **右手话题名称**: `exhand/mapping_data_right` (可通过参数 `mapping_topic_right` 配置)
- **左手话题名称**: `exhand/mapping_data_left` (可通过参数 `mapping_topic_left` 配置)
- **消息类型**: `std_msgs/Float32MultiArray`
- **数据格式**: 每个话题包含15个浮点数
  - 每个手指3个关节：Yaw, Pitch, Tip
  - 拇指: 索引 0, 1, 2
  - 食指: 索引 3, 4, 5
  - 中指: 索引 6, 7, 8
  - 无名指: 索引 9, 10, 11
  - 小指: 索引 12, 13, 14

### CAN通信

- **底层实现**: Linux socketcan
- **默认比特率**: 1Mbps (1000000 bps)
- **默认接口**: can0
- **帧格式**: 标准CAN帧（11位标识符）
- **数据格式**: 
  - `data[0]`: 帧类型（frame_header）
  - `data[1..7]`: 控制数据

### 支持的协议

- **L10协议**: 单帧7字节协议
  - 帧类型 0x01: Pitch帧
  - 帧类型 0x04: Yaw帧

- **L20协议**: 6帧协议（完整控制）
  - 帧类型 0x11: Pitch帧
  - 帧类型 0x12: Yaw帧
  - 帧类型 0x13: Roll帧
  - 帧类型 0x14: Tip帧
  - 帧类型 0x15: Speed帧
  - 帧类型 0x16: Current帧

- **L21协议**: 4帧协议（简化版，无速度电流）
  - 帧类型 0x01: Roll帧
  - 帧类型 0x02: Yaw帧
  - 帧类型 0x03: Pitch帧
  - 帧类型 0x06: Tip帧

## 系统要求

- ROS2 (Humble或更高版本)
- Linux系统（支持socketcan）
- CAN接口硬件（如USB-CAN适配器或板载CAN控制器）
- 已加载CAN相关内核模块（`can`, `can_raw`, `vcan`等）

### 检查CAN接口

```bash
# 查看CAN接口状态
ip link show can0

# 如果接口不存在，可能需要加载模块
sudo modprobe can
sudo modprobe can_raw
sudo modprobe vcan  # 虚拟CAN接口（用于测试）
```

## 构建和运行

### 构建功能包

```bash
cd /home/ywt/Ros2_WS
colcon build --packages-select linkerhand_cl
source install/setup.bash
```

### 运行节点

#### 基本运行（模拟模式，不启用CAN）

```bash
ros2 run linkerhand_cl hand_control_node
```

#### 启用CAN通信

```bash
ros2 run linkerhand_cl hand_control_node --ros-args \
  -p enable_can:=true \
  -p can_interface:=can0 \
  -p can_bitrate:=1000000
```

#### 使用调试日志

```bash
ros2 run linkerhand_cl hand_control_node --ros-args --log-level debug
```

#### 完整参数示例

```bash
ros2 run linkerhand_cl hand_control_node --ros-args \
  -p mapping_topic_right:=exhand/mapping_data_right \
  -p mapping_topic_left:=exhand/mapping_data_left \
  -p right_hand_id:=0x27 \
  -p left_hand_id:=0x28 \
  -p protocol:=1 \
  -p enable_can:=true \
  -p can_interface:=can0 \
  -p can_bitrate:=1000000
```

## 节点参数

### 话题配置

- `mapping_topic_right` (string, 默认: `exhand/mapping_data_right`)
  - 右手映射数据话题名称

- `mapping_topic_left` (string, 默认: `exhand/mapping_data_left`)
  - 左手映射数据话题名称

### 设备配置

- `right_hand_id` (int, 默认: `0x27`)
  - 右手设备CAN ID（十六进制）

- `left_hand_id` (int, 默认: `0x28`)
  - 左手设备CAN ID（十六进制）

### 协议配置

- `protocol` (int, 默认: `1`)
  - 通信协议类型
  - `0`: L20协议（6帧，完整控制）
  - `1`: L10协议（单帧7字节，默认）
  - `2`: L21协议（4帧，简化版）

### CAN配置

- `enable_can` (bool, 默认: `false`)
  - 是否启用CAN总线通信
  - `true`: 启用CAN通信，实际发送控制数据
  - `false`: 仅模拟模式，不发送CAN数据

- `can_interface` (string, 默认: `can0`)
  - CAN接口名称（如 `can0`, `can1`, `vcan0`等）

- `can_bitrate` (int, 默认: `1000000`)
  - CAN总线比特率（bps）
  - 默认值：1000000（1Mbps）
  - 常用值：500000（500Kbps）、1000000（1Mbps）

## 使用示例

### 1. 测试模式（使用虚拟CAN接口）

```bash
# 创建虚拟CAN接口
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 运行节点（使用虚拟接口）
ros2 run linkerhand_cl hand_control_node --ros-args \
  -p enable_can:=true \
  -p can_interface:=vcan0

# 在另一个终端监听CAN数据
candump vcan0
```

### 2. 实际硬件（USB-CAN适配器）

```bash
# 假设CAN接口为can0，先配置接口（如果未配置）
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 运行节点
ros2 run linkerhand_cl hand_control_node --ros-args \
  -p enable_can:=true \
  -p can_interface:=can0 \
  -p can_bitrate:=1000000
```

### 3. 使用Launch文件

创建launch文件 `launch/hand_control.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linkerhand_cl',
            executable='hand_control_node',
            name='hand_control_node',
            parameters=[{
                'mapping_topic_right': 'exhand/mapping_data_right',
                'mapping_topic_left': 'exhand/mapping_data_left',
                'right_hand_id': 0x27,
                'left_hand_id': 0x28,
                'protocol': 1,  # L10协议（默认）
                'enable_can': True,
                'can_interface': 'can0',
                'can_bitrate': 1000000,  # 1Mbps
            }]
        )
    ])
```

运行launch文件：

```bash
ros2 launch linkerhand_cl hand_control.launch.py
```

## 注意事项

1. **权限要求**: 设置CAN接口比特率可能需要root权限。如果使用sudo，建议配置sudo免密或使用CAP_NET_ADMIN权限。

2. **CAN接口状态**: 确保CAN接口已正确配置并处于UP状态。可以使用 `ip link show can0` 检查。

3. **比特率匹配**: 确保所有连接到CAN总线的设备使用相同的比特率。

4. **协议选择**: 
   - L10协议（默认）：最简单，单帧7字节，适合基本控制
   - L20协议：功能最全，6帧协议，包含速度和电流控制
   - L21协议：L20的简化版，4帧协议，不包含速度和电流控制

5. **数据同步**: 节点采用事件驱动模式，当收到映射数据时立即处理并发送CAN数据，无需额外的控制频率设置。

6. **错误处理**: 如果CAN接口打开失败，节点会记录错误日志并继续运行（模拟模式）。

7. **Yaw轴死区处理**: 对于除拇指外的其他手指，Yaw轴实现了中点死区处理（0.35-0.65范围），以提高控制稳定性。

8. **左右手独立控制**: 左右手使用独立的话题和控制逻辑，可以分别配置不同的设备ID和参数。

## 故障排查

### CAN接口无法打开

```bash
# 检查接口是否存在
ip link show can0

# 检查接口状态
ip -details link show can0

# 手动配置接口
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

### 权限问题

如果无法设置CAN接口，可能需要：

```bash
# 方式1: 使用sudo运行节点
sudo -E ros2 run linkerhand_cl hand_control_node --ros-args -p enable_can:=true

# 方式2: 配置用户权限（需要root）
sudo setcap cap_net_admin+ep $(which ros2)
```

### 查看CAN数据

```bash
# 使用candump监听CAN数据
sudo apt install can-utils
candump can0

# 或使用cansend发送测试数据
cansend can0 027#0102030405060708
```

## 开发说明

### 代码结构

- `hand_control_node.cpp`: ROS2节点主程序，处理话题订阅和数据回调
- `can_sender.cpp/hpp`: CAN发送功能（socketcan实现）
- `hand_control.cpp/hpp`: 手部控制逻辑和协议处理

### 数据流程

1. **订阅映射数据**: 节点订阅左右手两个独立的映射数据话题
2. **数据验证**: 检查接收到的数据长度是否为15（每个手指3个关节）
3. **数据转换**: 将映射数据转换为手部控制命令
   - 拇指：Yaw轴反向处理（仅右手），同时设置Roll轴
   - 其他手指：Yaw轴死区处理
4. **协议转换**: 将用户控制数据转换为CAN协议数据
5. **CAN发送**: 通过socketcan发送CAN帧到指定设备

### 扩展开发

如需添加新的协议或功能，可以：

1. 在 `can_sender.cpp` 中添加新的协议发送方法
2. 在 `hand_control.hpp` 中定义新的协议枚举
3. 更新节点参数以支持新协议
4. 在 `hand_control_node.cpp` 中调整数据处理逻辑

## 许可证

[根据项目实际情况填写]
