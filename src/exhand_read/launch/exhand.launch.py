"""
手部控制器节点 Launch 文件

启动 exhand_node 节点，支持参数配置。

注意：1. CAN总线控制默认启用，如果需要禁用，请在 launch 文件中设置 enable_can 参数为 false。
      2. 协议类型默认使用 L20，如果需要使用其他协议，请在 launch 文件中设置 protocol 参数为其他值。
      3. 串口设备路径默认使用 /dev/ttyUSB0，如果需要使用其他串口设备，请在 launch 文件中设置 port 参数为其他值。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    
    protocol_arg = DeclareLaunchArgument(
        'protocol',
        default_value='1',
        description='协议类型（0=L20, 1=L10, 2=L21）'
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
            'protocol': LaunchConfiguration('protocol'),
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
            '  协议类型: ', LaunchConfiguration('protocol'), ' (0=L20, 1=L10, 2=L21)\n',
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
        protocol_arg,
        enable_can_arg,
        launch_info,
        exhand_node,
    ])

