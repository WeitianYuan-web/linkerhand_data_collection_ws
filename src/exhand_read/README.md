# EXHand Read - 外骨骼手套 ROS2 功能包

这是一个用于外骨骼手套的 ROS2 功能包，提供串口通信 SDK 和 ROS2 节点，用于读取和发布手部控制器的传感器数据和映射数据。

## 功能特性

- ✅ 串口通信 SDK，支持完整的命令协议
- ✅ ROS2 节点，自动发布传感器数据和映射数据
- ✅ 可配置的参数（串口、波特率等）
- ✅ Launch 文件支持，方便启动

## 系统要求

- ROS2 (Humble/Iron/Jazzy)
- C++17 编译器（GCC 7+ 或 Clang 5+）
- Linux 系统（使用 termios API）

## 安装依赖

本包使用 C++ 实现，无需额外的 Python 依赖。只需要确保系统有串口访问权限。

## 编译

```bash
cd /home/ywt/Ros2_WS/EXHand_read
colcon build --packages-select exhand_read
source install/setup.bash
```

## 使用方法

### 方法 1: 使用 Launch 文件（推荐）

```bash
# 使用默认参数启动
ros2 launch exhand_read exhand.launch.py

# 指定串口设备
ros2 launch exhand_read exhand.launch.py port:=/dev/ttyUSB0

# 指定所有参数
ros2 launch exhand_read exhand.launch.py \
    port:=/dev/ttyUSB0 \
    baudrate:=1152000 \
    publish_rate:=20.0 \
    protocol:=0 \
    enable_can:=false \
    auto_enable:=true \
    enable_frame_mode:=true \
    enable_sensor_push:=true

# 使用 L10 协议并启用 CAN 控制
ros2 launch exhand_read exhand.launch.py \
    protocol:=1 \
    enable_can:=true

# 禁用数据帧模式，但启用传感器推送
ros2 launch exhand_read exhand.launch.py \
    enable_frame_mode:=false \
    enable_sensor_push:=true

# 禁用传感器推送，只启用数据帧模式
ros2 launch exhand_read exhand.launch.py \
    enable_frame_mode:=true \
    enable_sensor_push:=false
```

### 方法 2: 直接运行节点

```bash
# 使用默认参数
ros2 run exhand_read exhand_node

# 指定参数
ros2 run exhand_read exhand_node --ros-args \
    -p port:=/dev/ttyUSB0 \
    -p baudrate:=1152000 \
    -p timeout:=1.0 \
    -p publish_rate:=20.0 \
    -p auto_enable:=true \
    -p enable_frame_mode:=true \
    -p enable_sensor_push:=true
```

## 节点参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `port` | string | `/dev/ttyUSB0` | 串口设备路径（Linux: `/dev/ttyUSB0`, Windows: `COM3`） |
| `baudrate` | int | `1152000` | 串口波特率 |
| `timeout` | float | `1.0` | 串口超时时间（秒） |
| `publish_rate` | float | `20.0` | 数据发布频率（Hz） |
| `auto_enable` | bool | `true` | 是否自动启用配置（如果为false，则忽略enable_frame_mode和enable_sensor_push） |
| `enable_frame_mode` | bool | `true` | 是否启用数据帧模式（仅在auto_enable=true时有效） |
| `enable_sensor_push` | bool | `true` | 是否启用传感器数据推送（仅在auto_enable=true时有效） |
| `protocol` | int | `0` | 协议类型（0=L20, 1=L10, 2=L21） |
| `enable_can` | bool | `false` | 是否启用CAN总线控制 |

## 发布的话题

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/exhand/sensor_data_right` | `std_msgs/UInt16MultiArray` | 右手原始传感器数据（15个 uint16 值） |
| `/exhand/sensor_data_left` | `std_msgs/UInt16MultiArray` | 左手原始传感器数据（15个 uint16 值） |
| `/exhand/mapping_data_right` | `std_msgs/Float32MultiArray` | 右手映射后的数据（15个 float 值） |
| `/exhand/mapping_data_left` | `std_msgs/Float32MultiArray` | 左手映射后的数据（15个 float 值） |
| `/exhand/status` | `std_msgs/UInt8` | 系统状态信息 |

### 数据格式说明

传感器数据和映射数据分别发布到左右手的话题，每个话题包含 **15 个值**，对应一只手的关节数据。

每只手的 15 个关节顺序为：
1. Thumb-Yaw（拇指偏摆）
2. Thumb-Pitch（拇指俯仰）
3. Thumb-Tip（拇指尖端）
4. Index-Yaw（食指偏摆）
5. Index-Pitch（食指俯仰）
6. Index-Tip（食指尖端）
7. Middle-Yaw（中指偏摆）
8. Middle-Pitch（中指俯仰）
9. Middle-Tip（中指尖端）
10. Ring-Yaw（无名指偏摆）
11. Ring-Pitch（无名指俯仰）
12. Ring-Tip（无名指尖端）
13. Little-Yaw（小指偏摆）
14. Little-Pitch（小指俯仰）
15. Little-Tip（小指尖端）

### 查看话题数据

```bash
# 查看右手传感器数据
ros2 topic echo /exhand/sensor_data_right

# 查看左手传感器数据
ros2 topic echo /exhand/sensor_data_left

# 查看右手映射数据
ros2 topic echo /exhand/mapping_data_right

# 查看左手映射数据
ros2 topic echo /exhand/mapping_data_left

# 查看话题列表
ros2 topic list

# 查看话题信息
ros2 topic info /exhand/sensor_data_right
```

## 数据顺序说明

传感器数据和映射数据包含 15 个值，按以下顺序排列（用于理解数据含义）：

1. Thumb-Yaw (拇指-偏航)
2. Thumb-Pitch (拇指-俯仰)
3. Thumb-Tip (拇指-指尖)
4. Index-Yaw (食指-偏航)
5. Index-Pitch (食指-俯仰)
6. Index-Tip (食指-指尖)
7. Middle-Yaw (中指-偏航)
8. Middle-Pitch (中指-俯仰)
9. Middle-Tip (中指-指尖)
10. Ring-Yaw (无名指-偏航)
11. Ring-Pitch (无名指-俯仰)
12. Ring-Tip (无名指-指尖)
13. Little-Yaw (小指-偏航)
14. Little-Pitch (小指-俯仰)
15. Little-Tip (小指-指尖)

## SDK 使用示例（C++）

如果需要直接使用 SDK（不通过 ROS2 节点），可以参考以下代码：

```cpp
#include "exhand_read/uart_frame_command.hpp"
#include <iostream>

int main()
{
    // 创建 SDK 实例
    exhand_read::UartFrameCommand sdk("/dev/ttyUSB0", 1152000, 1.0);
    
    // 启用数据帧模式
    auto resp = sdk.enable();
    if (resp) {
        std::cout << "启用数据帧模式: result=" << (int)resp->result << std::endl;
    }
    
    // 启用传感器数据推送
    resp = sdk.sensorEnable();
    if (resp) {
        std::cout << "启用传感器推送: result=" << (int)resp->result << std::endl;
    }
    
    // 接收数据帧
    auto frame = sdk.receiveAnyDataFrame(0.1);
    if (frame) {
        if (frame->frame_type == exhand_read::UartFrameCommand::AnyDataFrame::FrameType::SENSOR) {
            std::cout << "收到传感器数据: hand=" << (int)frame->hand << std::endl;
        } else if (frame->frame_type == exhand_read::UartFrameCommand::AnyDataFrame::FrameType::MAPPING) {
            std::cout << "收到映射数据: hand=" << (int)frame->hand << std::endl;
        }
    }
    
    // 查询系统状态
    auto status = sdk.status();
    if (status) {
        std::cout << "系统状态查询成功" << std::endl;
    }
    
    // 关闭连接
    sdk.close();
    return 0;
}
```

## 常见问题

### 1. 串口权限问题

如果遇到权限错误，需要将用户添加到 dialout 组：

```bash
sudo usermod -a -G dialout $USER
# 然后重新登录
```

### 2. 串口被占用

确保没有其他程序正在使用该串口。可以使用以下命令检查：

```bash
# Linux
lsof /dev/ttyUSB0

# 或者
sudo fuser /dev/ttyUSB0
```

### 3. 找不到串口设备

列出所有可用串口：

```bash
# Linux
ls /dev/ttyUSB* /dev/ttyACM*

# 或者使用 Python
python3 -c "import serial.tools.list_ports; [print(p.device) for p in serial.tools.list_ports.comports()]"
```

## 文件结构

```
exhand_read/
├── include/exhand_read/
│   ├── uart_frame_command.hpp    # 串口通信 SDK 头文件
│   └── exhand_node.hpp            # ROS2 节点头文件
├── exhand_read/
│   ├── uart_frame_command.cpp    # 串口通信 SDK 实现
│   └── exhand_node.cpp            # ROS2 节点实现
├── launch/
│   └── exhand.launch.py           # Launch 文件
├── CMakeLists.txt                 # CMake 构建配置
├── package.xml                    # 包配置文件
└── README.md                      # 本文档
```

## 许可证

TODO: License declaration

## 维护者

ywt <ywt@todo.todo>

