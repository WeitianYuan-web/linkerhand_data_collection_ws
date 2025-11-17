#!/usr/bin/env python3
"""
RealSense multi-camera launch.

读取 configs/camera_serial_numbers.yaml，根据 enabled 标记决定启动哪些节点。
"""

from pathlib import Path
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

CONFIG_PATH = Path(__file__).resolve().parent.parent / "configs" / "camera_serial_numbers.yaml"


def load_camera_config():
    config = {
        "d455": {"enabled": True, "serial": "", "namespace": "camera", "name": "d455_camera"},
        "d405_left": {"enabled": True, "serial": "", "namespace": "camera_left_wrist", "name": "d405_left_camera"},
        "d405_right": {"enabled": True, "serial": "", "namespace": "camera_right_wrist", "name": "d405_right_camera"},
    }
    if not CONFIG_PATH.exists():
        print(f"Warning: {CONFIG_PATH} not found, using defaults")
        return config

    data = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8")) or {}
    camera_table = data.get("camera_serial_numbers", {})

    def populate(key, cfg_key):
        cam = camera_table.get(cfg_key, {})
        config[key]["enabled"] = cam.get("enabled", True)
        config[key]["serial"] = cam.get("serial_number", "") or ""
        config[key]["namespace"] = cam.get("namespace", config[key]["namespace"])
        config[key]["name"] = cam.get("camera_name", config[key]["name"])

    populate("d455", "d455_top")
    populate("d405_left", "d405_left_wrist")
    populate("d405_right", "d405_right_wrist")
    return config


def make_node(cam_key, cfg, condition):
    serial = cfg["serial"]
    if not serial:
        print(f"Warning: no serial provided for {cam_key}, node will run with default device")

    params = {
        "serial_no": serial,
        "camera_name": cfg["name"],
        "device_type": "d455" if cam_key == "d455" else "d405",
        "enable_color": True, # 彩色图
        "enable_depth": False, # 深度图
        "enable_gyro": False, # 陀螺仪
        "enable_accel": False, # 加速度计
        "enable_infra1": False, # 红外1
        "enable_infra2": False, # 红外2
        "align_depth.enable": False, # 深度对齐
        "pointcloud.enable": False, # 点云
        "clip_distance": 2.0 if cam_key == "d455" else 1.5,
        "linear_accel_cov": 0.01, # 线性加速度协方差
        "angular_velocity_cov": 0.01, # 角速度协方差
    }
    profile_key = "rgb_camera.color_profile" if cam_key == "d455" else "depth_module.color_profile"
    params[profile_key] = "640x480x30"

    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name=cfg["name"],
        namespace=cfg["namespace"],
        parameters=[params],
        output="screen",
        condition=condition,
    )


def generate_launch_description():
    cam_config = load_camera_config()

    enable_d455_arg = DeclareLaunchArgument(
        "enable_d455",
        default_value="true" if cam_config["d455"]["enabled"] else "false",
        description="Enable D455 top camera",
    )
    enable_d405_left_arg = DeclareLaunchArgument(
        "enable_d405_left",
        default_value="true" if cam_config["d405_left"]["enabled"] else "false",
        description="Enable D405 left wrist camera",
    )
    enable_d405_right_arg = DeclareLaunchArgument(
        "enable_d405_right",
        default_value="true" if cam_config["d405_right"]["enabled"] else "false",
        description="Enable D405 right wrist camera",
    )

    enable_d455 = LaunchConfiguration("enable_d455")
    enable_d405_left = LaunchConfiguration("enable_d405_left")
    enable_d405_right = LaunchConfiguration("enable_d405_right")

    nodes = [
        enable_d455_arg,
        enable_d405_left_arg,
        enable_d405_right_arg,
        make_node("d455", cam_config["d455"], IfCondition(enable_d455)),
        make_node("d405_left", cam_config["d405_left"], IfCondition(enable_d405_left)),
        make_node("d405_right", cam_config["d405_right"], IfCondition(enable_d405_right)),
    ]

    return LaunchDescription(nodes)