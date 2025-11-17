#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    joint_topic_arg = DeclareLaunchArgument(
        'joint_topic',
        default_value='/cb_pos_vel_trajectory_controller/command',
        description='Joint trajectory topic'
    )
    
    yolov8_topic_arg = DeclareLaunchArgument(
        'yolov8_topic',
        default_value='/yolov8/detection_msg',
        description='YOLOv8 detection topic'
    )
    
    is_filtering_arg = DeclareLaunchArgument(
        'is_filtering',
        default_value='True',
        description='Enable filtering'
    )

    return LaunchDescription([
        # Declare arguments
        joint_topic_arg,
        yolov8_topic_arg,
        is_filtering_arg,
        
        # Include MQTT launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('linkerhand_data_collection_srv'),
                    'launch',
                    'mqtt.launch.py'
                ])
            ])
        ),
        
        # Main LinkerHand Data Collection Service Node
        Node(
            package='linkerhand_data_collection_srv',
            executable='linkerhand_data_collection.py',
            name='linkerhand_data_collection_server',
            output='screen',
            parameters=[{
                'joint_topic': LaunchConfiguration('joint_topic'),
                'yolov8_topic': LaunchConfiguration('yolov8_topic'),
                'is_filtering': LaunchConfiguration('is_filtering'),
            }]
        )
    ])
