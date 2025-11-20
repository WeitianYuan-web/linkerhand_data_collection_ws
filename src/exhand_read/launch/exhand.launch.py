"""
手部控制器节点 Launch 文件

启动 exhand_node 节点，支持参数配置。

注意：1. CAN总线控制默认启用，如果需要禁用，请在 launch 文件中设置 enable_can 参数为 false。
      2. 协议类型默认使用 L10，如果 task_config.json 中存在 handModel 字段，会自动从该文件读取。
      3. 串口设备路径默认使用 /dev/ttyUSB0，如果需要使用其他串口设备，请在 launch 文件中设置 port 参数为其他值。
"""

import json
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def read_protocol_from_config(context):
    """
    从 task_config.json 读取协议类型
    
    Args:
        context: Launch 上下文
        
    Returns:
        int: 协议类型ID（0=L20, 1=L10, 2=L21, 3=L7, 4=L25, 5=O6, 6=L24）
    """
    # 手部型号到协议类型的映射
    hand_model_to_protocol = {
        "L10": 1,      # HAND_PROTO_L10
        "L10V7": 1,    # HAND_PROTO_L10
        "L20": 0,      # HAND_PROTO_L20
        "L21": 2,      # HAND_PROTO_L21
        "L7": 3,       # HAND_PROTO_L7
        "O7": 3,       # HAND_PROTO_L7
        "L25": 4,      # HAND_PROTO_L25
        "T25": 4,      # HAND_PROTO_L25
        "O6": 5,       # HAND_PROTO_O6
        "L24": 6       # HAND_PROTO_L24
    }
    
    # 默认协议类型（L10）
    default_protocol = 1
    
    # 尝试从多个位置查找 task_config.json
    possible_paths = [
        os.path.join(os.path.expanduser('~'), 'linkerhand_data_collection_ws', 'task_config.json'),
        os.path.join(os.getcwd(), 'task_config.json'),
        '/home/ywt/linkerhand_data_collection_ws/task_config.json',
    ]
    
    # 如果设置了 task_config_path 参数，优先使用
    task_config_path = context.launch_configurations.get('task_config_path', None)
    if task_config_path:
        possible_paths.insert(0, task_config_path)
    
    # 尝试读取配置文件
    for config_path in possible_paths:
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    config_data = json.load(f)
                    hand_model = config_data.get('handModel', None)
                    if hand_model and hand_model in hand_model_to_protocol:
                        protocol_id = hand_model_to_protocol[hand_model]
                        print(f"[exhand.launch.py] 从 {config_path} 读取到 handModel: {hand_model}, 协议ID: {protocol_id}")
                        return protocol_id
                    elif hand_model:
                        print(f"[exhand.launch.py] 警告: 未知的 handModel: {hand_model}, 使用默认协议 L10")
            except Exception as e:
                print(f"[exhand.launch.py] 读取 {config_path} 时出错: {e}")
            break
    
    print(f"[exhand.launch.py] 未找到有效的 task_config.json，使用默认协议 L10")
    return default_protocol


def get_protocol_value(context):
    """
    获取协议类型值，优先从 task_config.json 读取，失败则使用参数值
    
    Args:
        context: Launch 上下文
        
    Returns:
        str: 协议类型字符串
    """
    try:
        protocol_from_config = read_protocol_from_config(context)
        return str(protocol_from_config)
    except Exception as e:
        print(f"[exhand.launch.py] 读取协议类型时出错: {e}，使用参数值")
        return LaunchConfiguration('protocol').perform(context)


def generate_launch_description():
    """
    生成 Launch 描述
    
    Returns:
        LaunchDescription: Launch 描述对象
    """
    
    # 声明 Launch 参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='串口设备路径（如 /dev/ttyUSB0 或 COM3）'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='1152000',
        description='串口波特率'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='1.0',
        description='串口超时时间（秒）'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='20.0',
        description='数据发布频率（Hz）'
    )
    
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        description='是否自动启用配置（如果为false，则忽略enable_sensor_push和enable_mapping_push）'
    )
    
    enable_sensor_push_arg = DeclareLaunchArgument(
        'enable_sensor_push',
        default_value='false',
        description='是否启用传感器数据推送（仅在auto_enable=true时有效）'
    )
    
    enable_mapping_push_arg = DeclareLaunchArgument(
        'enable_mapping_push',
        default_value='true',
        description='是否启用映射数据推送（仅在auto_enable=true时有效）'
    )
    
    task_config_path_arg = DeclareLaunchArgument(
        'task_config_path',
        default_value='',
        description='task_config.json 文件路径（可选，留空则自动查找）'
    )
    
    protocol_arg = DeclareLaunchArgument(
        'protocol',
        default_value='1',
        description='协议类型（0=L20, 1=L10, 2=L21, 3=L7, 4=L25, 5=O6, 6=L24）。如果 task_config.json 中存在 handModel，会自动覆盖此值'
    )
    
    enable_can_arg = DeclareLaunchArgument(
        'enable_can',
        default_value='false',
        description='是否启用CAN总线控制'
    )
    
    # 创建节点
    exhand_node = Node(
        package='exhand_read',
        executable='exhand_node',
        name='exhand_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'timeout': LaunchConfiguration('timeout'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'enable_sensor_push': LaunchConfiguration('enable_sensor_push'),
            'enable_mapping_push': LaunchConfiguration('enable_mapping_push'),
            'protocol': OpaqueFunction(function=get_protocol_value),
            'enable_can': LaunchConfiguration('enable_can'),
        }],
        remappings=[
            # 可以在这里添加话题重映射
            # ('/exhand/sensor_data', '/custom/sensor_data'),
        ]
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            '启动手部控制器节点\n',
            '  串口: ', LaunchConfiguration('port'), '\n',
            '  波特率: ', LaunchConfiguration('baudrate'), '\n',
            '  发布频率: ', LaunchConfiguration('publish_rate'), ' Hz\n',
            '  协议类型: ', OpaqueFunction(function=get_protocol_value), 
            ' (0=L20, 1=L10, 2=L21, 3=L7, 4=L25, 5=O6, 6=L24)\n',
            '  CAN总线控制: ', LaunchConfiguration('enable_can'), '\n',
            '  自动启用: ', LaunchConfiguration('auto_enable'), '\n',
            '  启用传感器推送: ', LaunchConfiguration('enable_sensor_push'), '\n',
            '  启用映射推送: ', LaunchConfiguration('enable_mapping_push')
        ]
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        timeout_arg,
        publish_rate_arg,
        auto_enable_arg,
        enable_sensor_push_arg,
        enable_mapping_push_arg,
        task_config_path_arg,
        protocol_arg,
        enable_can_arg,
        launch_info,
        exhand_node,
    ])

