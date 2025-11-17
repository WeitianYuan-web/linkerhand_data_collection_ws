"""
LinkerHand 单手模式 Launch 文件

用于启动单手灵巧手控制节点，兼容原 linker_hand_ros2_sdk 的接口。
支持从 task_config.json 读取配置，或通过参数指定。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_node(context, *args, **kwargs):
    """
    动态生成节点配置（根据 hand_type 参数）
    """
    hand_type = context.launch_configurations.get('hand_type', 'left')
    hand_joint = context.launch_configurations.get('hand_joint', 'L10')
    
    # 根据 hand_type 设置参数
    if hand_type == 'right':
        mapping_topic_right = 'exhand/mapping_data_right'
        mapping_topic_left = ''
        right_hand_id = '39'
        left_hand_id = '0'
        right_hand_model = hand_joint
        left_hand_model = ''
    else:  # left
        mapping_topic_right = ''
        mapping_topic_left = 'exhand/mapping_data_left'
        right_hand_id = '0'
        left_hand_id = '40'
        right_hand_model = ''
        left_hand_model = hand_joint
    
    workspace_path = os.path.expanduser('~/LinkerOS/linkerhand_data_collection_ws')
    task_config_path = os.path.join(workspace_path, 'task_config.json')
    
    return [
        Node(
            package='linkerhand_cl',
            executable='hand_control_node',
            name='hand_control_node',
            output='screen',
            parameters=[{
                'mapping_topic_right': mapping_topic_right,
                'mapping_topic_left': mapping_topic_left,
                'right_hand_id': right_hand_id,
                'left_hand_id': left_hand_id,
                'right_hand_model': right_hand_model,
                'left_hand_model': left_hand_model,
                'task_config_path': task_config_path,
                'enable_can': context.launch_configurations.get('enable_can', 'true'),
                'can_interface': context.launch_configurations.get('can', 'can0'),
                'can_bitrate': context.launch_configurations.get('can_bitrate', '1000000'),
                'enable_feedback': context.launch_configurations.get('enable_feedback', 'true'),
                'feedback_publish_rate': context.launch_configurations.get('feedback_publish_rate', '50.0'),
            }]
        )
    ]


def generate_launch_description():
    """
    生成 Launch 描述（单手模式）
    
    Returns:
        LaunchDescription: Launch 描述对象
    """
    
    # 获取工作空间路径
    workspace_path = os.path.expanduser('~/LinkerOS/linkerhand_data_collection_ws')
    task_config_path = os.path.join(workspace_path, 'task_config.json')
    
    # 声明 Launch 参数（兼容原 SDK 接口）
    hand_type_arg = DeclareLaunchArgument(
        'hand_type',
        default_value='left',
        description='手类型：left 或 right'
    )
    
    hand_joint_arg = DeclareLaunchArgument(
        'hand_joint',
        default_value='L10',
        description='灵巧手型号：L10, L20, L21, L7, L25, O6, L24 等'
    )
    
    can_arg = DeclareLaunchArgument(
        'can',
        default_value='can0',
        description='CAN接口名称（如can0, can1等）'
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
    
    # 启动信息
    launch_info = LogInfo(
        msg=[
            '启动LinkerHand控制节点（单手模式）\n',
            '  手类型: ', LaunchConfiguration('hand_type'), '\n',
            '  手型号: ', LaunchConfiguration('hand_joint'), '\n',
            '  CAN接口: ', LaunchConfiguration('can'), '\n',
            '  任务配置路径: ', LaunchConfiguration('task_config_path'), '\n',
            '  CAN总线控制: ', LaunchConfiguration('enable_can'), '\n',
            '  CAN比特率: ', LaunchConfiguration('can_bitrate'), ' bps'
        ]
    )
    
    return LaunchDescription([
        hand_type_arg,
        hand_joint_arg,
        can_arg,
        task_config_path_arg,
        enable_can_arg,
        can_bitrate_arg,
        enable_feedback_arg,
        feedback_publish_rate_arg,
        launch_info,
        OpaqueFunction(function=generate_node),
    ])

