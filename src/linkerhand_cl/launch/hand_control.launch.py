"""
LinkerHand控制节点 Launch 文件

启动 hand_control_node 节点，用于订阅EXHand映射数据话题来控制灵巧手。
支持从task_config.json读取配置。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    生成 Launch 描述
    
    Returns:
        LaunchDescription: Launch 描述对象
    """
    
    # 获取工作空间路径
    workspace_path = os.path.expanduser('~/LinkerOS/linkerhand_data_collection_ws')
    task_config_path = os.path.join(workspace_path, 'task_config.json')
    
    # 声明 Launch 参数
    mapping_topic_right_arg = DeclareLaunchArgument(
        'mapping_topic_right',
        default_value='exhand/mapping_data_right',
        description='右手映射数据话题'
    )
    
    mapping_topic_left_arg = DeclareLaunchArgument(
        'mapping_topic_left',
        default_value='exhand/mapping_data_left',
        description='左手映射数据话题'
    )
    
    right_hand_id_arg = DeclareLaunchArgument(
        'right_hand_id',
        default_value='39',  # 0x27
        description='右手设备ID（十六进制，如39表示0x27）'
    )
    
    left_hand_id_arg = DeclareLaunchArgument(
        'left_hand_id',
        default_value='40',  # 0x28
        description='左手设备ID（十六进制，如40表示0x28）'
    )
    
    protocol_arg = DeclareLaunchArgument(
        'protocol',
        default_value='1',  # L10
        description='协议类型（0=L20, 1=L10, 2=L21, 3=L7, 4=L25, 5=O6, 6=L24）'
    )
    
    right_hand_model_arg = DeclareLaunchArgument(
        'right_hand_model',
        default_value='',
        description='右手灵巧手型号（L10, L20, L21, L7, L25, O6, L24等），如果为空则从task_config.json读取'
    )
    
    left_hand_model_arg = DeclareLaunchArgument(
        'left_hand_model',
        default_value='',
        description='左手灵巧手型号（L10, L20, L21, L7, L25, O6, L24等），如果为空则从task_config.json读取'
    )
    
    task_config_path_arg = DeclareLaunchArgument(
        'task_config_path',
        default_value=task_config_path,
        description='task_config.json文件路径'
    )
    
    enable_can_arg = DeclareLaunchArgument(
        'enable_can',
        default_value='true',
        description='是否启用CAN总线控制'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN接口名称（如can0）'
    )
    
    can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value='1000000',
        description='CAN比特率（bps），默认1Mbps'
    )
    
    # 创建节点
    hand_control_node = Node(
        package='linkerhand_cl',
        executable='hand_control_node',
        name='hand_control_node',
        output='screen',
        parameters=[{
            'mapping_topic_right': LaunchConfiguration('mapping_topic_right'),
            'mapping_topic_left': LaunchConfiguration('mapping_topic_left'),
            'right_hand_id': LaunchConfiguration('right_hand_id'),
            'left_hand_id': LaunchConfiguration('left_hand_id'),
            'protocol': LaunchConfiguration('protocol'),
            'right_hand_model': LaunchConfiguration('right_hand_model'),
            'left_hand_model': LaunchConfiguration('left_hand_model'),
            'task_config_path': LaunchConfiguration('task_config_path'),
            'enable_can': LaunchConfiguration('enable_can'),
            'can_interface': LaunchConfiguration('can_interface'),
            'can_bitrate': LaunchConfiguration('can_bitrate'),
        }]
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            '启动LinkerHand控制节点\n',
            '  右手映射话题: ', LaunchConfiguration('mapping_topic_right'), '\n',
            '  左手映射话题: ', LaunchConfiguration('mapping_topic_left'), '\n',
            '  右手设备ID: ', LaunchConfiguration('right_hand_id'), '\n',
            '  左手设备ID: ', LaunchConfiguration('left_hand_id'), '\n',
            '  协议类型: ', LaunchConfiguration('protocol'), '\n',
            '  任务配置路径: ', LaunchConfiguration('task_config_path'), '\n',
            '  CAN总线控制: ', LaunchConfiguration('enable_can'), '\n',
            '  CAN接口: ', LaunchConfiguration('can_interface'), '\n',
            '  CAN比特率: ', LaunchConfiguration('can_bitrate'), ' bps'
        ]
    )
    
    return LaunchDescription([
        mapping_topic_right_arg,
        mapping_topic_left_arg,
        right_hand_id_arg,
        left_hand_id_arg,
        protocol_arg,
        right_hand_model_arg,
        left_hand_model_arg,
        task_config_path_arg,
        enable_can_arg,
        can_interface_arg,
        can_bitrate_arg,
        launch_info,
        hand_control_node,
    ])

