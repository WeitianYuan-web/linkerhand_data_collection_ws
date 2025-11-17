"""
LinkerHand 双手模式 Launch 文件

用于启动双手灵巧手控制节点，兼容原 linker_hand_ros2_sdk 的接口。
支持从 task_config.json 读取配置，或通过参数指定。
注意：如果左右手使用不同的 CAN 接口，需要启动两个节点。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    生成 Launch 描述（双手模式）
    
    Returns:
        LaunchDescription: Launch 描述对象
    """
    
    # 获取工作空间路径
    workspace_path = os.path.expanduser('~/LinkerOS/linkerhand_data_collection_ws')
    task_config_path = os.path.join(workspace_path, 'task_config.json')
    
    # 声明 Launch 参数（兼容原 SDK 接口）
    left_hand_joint_arg = DeclareLaunchArgument(
        'left_hand_joint',
        default_value='L10',
        description='左手灵巧手型号：L10, L20, L21, L7, L25, O6, L24 等'
    )
    
    right_hand_joint_arg = DeclareLaunchArgument(
        'right_hand_joint',
        default_value='L10',
        description='右手灵巧手型号：L10, L20, L21, L7, L25, O6, L24 等'
    )
    
    left_can_arg = DeclareLaunchArgument(
        'left_can',
        default_value='can3',
        description='左手CAN接口名称（如can0, can1等）'
    )
    
    right_can_arg = DeclareLaunchArgument(
        'right_can',
        default_value='can1',
        description='右手CAN接口名称（如can0, can1等）'
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
    
    can_bitrate_arg = DeclareLaunchArgument(
        'can_bitrate',
        default_value='1000000',
        description='CAN比特率（bps），默认1Mbps'
    )
    
    enable_feedback_arg = DeclareLaunchArgument(
        'enable_feedback',
        default_value='true',
        description='是否启用反馈数据接收和发布'
    )
    
    feedback_publish_rate_arg = DeclareLaunchArgument(
        'feedback_publish_rate',
        default_value='50.0',
        description='反馈数据发布频率（Hz）'
    )
    
    # 检查左右手是否使用相同的 CAN 接口
    # 如果相同，启动一个节点；如果不同，启动两个节点
    use_single_node_arg = DeclareLaunchArgument(
        'use_single_node',
        default_value='false',
        description='是否使用单个节点（当左右手使用相同CAN接口时）'
    )
    
    # 创建节点（默认使用两个节点，分别控制左右手）
    # 左手节点
    left_hand_control_node = Node(
        package='linkerhand_cl',
        executable='hand_control_node',
        name='hand_control_node_left',
        output='screen',
        parameters=[{
            'mapping_topic_right': '',  # 左手节点不订阅右手话题
            'mapping_topic_left': 'exhand/mapping_data_left',
            'right_hand_id': '0',  # 禁用右手
            'left_hand_id': '40',  # 0x28
            'right_hand_model': '',
            'left_hand_model': LaunchConfiguration('left_hand_joint'),
            'task_config_path': LaunchConfiguration('task_config_path'),
            'enable_can': LaunchConfiguration('enable_can'),
            'can_interface': LaunchConfiguration('left_can'),
            'can_bitrate': LaunchConfiguration('can_bitrate'),
            'enable_feedback': LaunchConfiguration('enable_feedback'),
            'feedback_publish_rate': LaunchConfiguration('feedback_publish_rate'),
        }]
    )
    
    # 右手节点
    right_hand_control_node = Node(
        package='linkerhand_cl',
        executable='hand_control_node',
        name='hand_control_node_right',
        output='screen',
        parameters=[{
            'mapping_topic_right': 'exhand/mapping_data_right',
            'mapping_topic_left': '',  # 右手节点不订阅左手话题
            'right_hand_id': '39',  # 0x27
            'left_hand_id': '0',  # 禁用左手
            'right_hand_model': LaunchConfiguration('right_hand_joint'),
            'left_hand_model': '',
            'task_config_path': LaunchConfiguration('task_config_path'),
            'enable_can': LaunchConfiguration('enable_can'),
            'can_interface': LaunchConfiguration('right_can'),
            'can_bitrate': LaunchConfiguration('can_bitrate'),
            'enable_feedback': LaunchConfiguration('enable_feedback'),
            'feedback_publish_rate': LaunchConfiguration('feedback_publish_rate'),
        }]
    )
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            '启动LinkerHand控制节点（双手模式）\n',
            '  左手型号: ', LaunchConfiguration('left_hand_joint'), '\n',
            '  右手型号: ', LaunchConfiguration('right_hand_joint'), '\n',
            '  左手CAN接口: ', LaunchConfiguration('left_can'), '\n',
            '  右手CAN接口: ', LaunchConfiguration('right_can'), '\n',
            '  任务配置路径: ', LaunchConfiguration('task_config_path'), '\n',
            '  CAN总线控制: ', LaunchConfiguration('enable_can'), '\n',
            '  CAN比特率: ', LaunchConfiguration('can_bitrate'), ' bps'
        ]
    )
    
    return LaunchDescription([
        left_hand_joint_arg,
        right_hand_joint_arg,
        left_can_arg,
        right_can_arg,
        task_config_path_arg,
        enable_can_arg,
        can_bitrate_arg,
        enable_feedback_arg,
        feedback_publish_rate_arg,
        use_single_node_arg,
        launch_info,
        left_hand_control_node,
        right_hand_control_node,
    ])

