
#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from operator import is_
import rclpy
from rclpy.node import Node

from dataclasses import dataclass
from typing import List, Dict
from std_msgs.msg import String,Header
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import std_msgs.msg

import os
import cv2
import sys
import time
import json
import signal
import threading
import numpy as np
import pandas as pd

from utils.mapping import *
# # 导入piper_sdk模块
# from piper_sdk import *
from utils.piper_recorder_adapter import *


# 配置和常量定义
@dataclass
class HandConfig:
    """手部配置数据类"""
    joint_names: List[str] = None
    joint_names_en: List[str] = None
    init_pos: List[int] = None
    preset_actions: Dict[str, List[int]] = None

    @classmethod
    def from_hand_type(cls, hand_type: str) -> 'HandConfig':
        """根据手部类型创建配置"""
        hand_configs = {
            "L25": cls(
                joint_names=["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部",
                            "大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆",
                            "大拇指横滚","预留","预留","预留","预留","大拇指中部","食指中部",
                            "中指中部","无名指中部","小拇指中部","大拇指指尖","食指指尖",
                            "中指指尖","无名指指尖","小拇指指尖"],
                init_pos=[255] * 25,
                preset_actions={
                    "握拳": [0]*25,
                    "张开": [255]*25,
                    "OK": [255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 
                           255, 255, 255, 255, 255, 255, 0, 0, 255, 255, 
                           0, 0, 0, 255, 255]
                }
            ),
            "L21": cls(
                joint_names=["大拇指根部","食指根部","中指根部","无名指根部","小拇指根部",
                            "大拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小拇指侧摆",
                            "大拇指横滚","预留","预留","预留","预留","大拇指中部","预留",
                            "预留","预留","预留","大拇指指尖","食指指尖","中指指尖",
                            "无名指指尖","小拇指指尖"],
                init_pos=[255] * 25
            ),
            "L20": cls(
                joint_names=["拇指根部", "食指根部", "中指根部", "无名指根部","小指根部",
                            "拇指侧摆","食指侧摆","中指侧摆","无名指侧摆","小指侧摆",
                            "拇指横摆","预留","预留","预留","预留","拇指尖部","食指末端",
                            "中指末端","无名指末端","小指末端"],
                init_pos=[255,255,255,255,255,255,10,100,180,240,245,255,255,255,255,255,255,255,255,255],
                preset_actions = {
                    "握拳": [40, 0, 0, 0, 0, 131, 10, 100, 180, 240, 19, 255, 255, 255, 255, 135, 0, 0, 0, 0],
                    "张开": [255, 255, 255, 255, 255, 255, 10, 100, 180, 240, 245, 255, 255, 255, 255, 255, 255, 255, 255, 255],
                    "OK": [191, 95, 255, 255, 255, 136, 107, 100, 180, 240, 72, 255, 255, 255, 255, 116, 99, 255, 255, 255],
                    "点赞": [255, 0, 0, 0, 0, 127, 10, 100, 180, 240, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0]
                }
            ),
            "L10": cls(
                joint_names_en =["thumb_cmc_pitch", "thumb_cmc_roll","index_mcp_pitch","middle_mcp_pitch","ring_mcp_pitch","pinky_mcp_pitch","index_mcp_roll","ring_mcp_roll","pinky_mcp_roll","thumb_cmc_yaw"],
                joint_names=["拇指根部", "拇指侧摆","食指根部", "中指根部", "无名指根部", 
                            "小指根部","食指侧摆","无名指侧摆","小指侧摆","拇指旋转"],
                init_pos=[255] * 10,
                preset_actions={
                    "握拳": [75, 128, 0, 0, 0, 0, 128, 128, 128, 57],
                    "张开": [255, 128, 255, 255, 255, 255, 128, 128, 128, 128],
                    "OK": [110, 128, 75, 255, 255, 255, 128, 128, 128, 68],
                    "点赞": [255, 145, 0, 0, 0, 0, 0, 255, 255, 65]
                }
            ),
            "L7": cls(
                joint_names=["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲", 
                            "小拇指弯曲","拇指旋转"],
                init_pos=[250] * 7,
                preset_actions={
                    "握拳": [71, 79, 0, 0, 0, 0, 64],
                    "张开": [255, 111, 250, 250, 250, 250, 55],
                    "OK": [141, 111, 168, 250, 250, 250, 86],
                    "点赞": [255, 111, 0, 0, 0, 0, 86]
                }
            ),
            "O6": cls(
                # [103, 102, 111, 250, 250, 250] 对指
                joint_names_en = ["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch", "middle_mcp_pitch", "pinky_mcp_pitch", "ring_mcp_pitch"],
                joint_names=["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"],
                init_pos=[250] * 6,
                preset_actions={
                    
                    "张开": [250, 250, 250, 250, 250, 250],
                    "壹": [125, 18, 255, 0, 0, 0],
                    "贰": [92, 87, 255, 255, 0, 0],
                    "叁": [92, 87, 255, 255, 255, 0],
                    "肆": [92, 87, 255, 255, 255, 255],
                    "伍": [255, 255, 255, 255, 255, 255],
                    "OK": [96, 100, 118, 250, 250, 250],
                    "点赞": [250, 79, 0, 0, 0, 0],
                    "握拳": [102, 18, 0, 0, 0, 0],
                    
                }
            ),
            "L6": cls(
                joint_names_en = ["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch", "middle_mcp_pitch", "pinky_mcp_pitch", "ring_mcp_pitch"],
                joint_names=["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"],
                init_pos=[250] * 6,
                preset_actions={
                    
                    "张开": [250, 250, 250, 250, 250, 250],
                    "壹": [125, 18, 255, 0, 0, 0],
                    "贰": [92, 87, 255, 255, 0, 0],
                    "叁": [92, 87, 255, 255, 255, 0],
                    "肆": [92, 87, 255, 255, 255, 255],
                    "伍": [255, 255, 255, 255, 255, 255],
                    "OK": [96, 100, 118, 250, 250, 250],
                    "点赞": [250, 79, 0, 0, 0, 0],
                    "握拳": [102, 18, 0, 0, 0, 0],
                    
                }
            ),
            "L6P": cls(
                joint_names_en = ["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch", "middle_mcp_pitch", "pinky_mcp_pitch", "ring_mcp_pitch"],
                joint_names=["大拇指弯曲", "大拇指横摆","食指弯曲", "中指弯曲", "无名指弯曲", "小拇指弯曲"],
                init_pos=[250] * 6,
                preset_actions={
                    "张开": [250, 250, 250, 250, 250, 250],
                    "壹": [0, 31, 255, 0, 0, 0],
                    "贰": [0, 31, 255, 255, 0, 0],
                    "叁": [0, 30, 255, 255, 255, 0],
                    "肆": [0, 30, 255, 255, 255, 255],
                    "伍": [250, 250, 250, 250, 250, 250],
                    "OK": [54, 41, 164, 250, 250, 250],
                    "点赞": [255, 31, 0, 0, 0, 0],
                    "握拳": [49, 61, 0, 0, 0, 0],
                }
            ),
        }
        return hand_configs.get(hand_type.upper(), hand_configs["L10"])



class JointCommandPublisher_piper():
    def __init__(self, hand_type):
        super().__init__()
    
        # 获取松灵piper机械臂接口
        self.piper_controller = PiperRecorderAdapter(side=hand_type)

    def get_qpos(self):
        # 使用如下方法获取角度值（角度）
        print(self.piper_controller.get_qpos())

    def set_position(self, positions_list):
        # 注意！官方代码输入的是弧度值，我将其改为角度输入
        # positions_list = [0,0,0,0,0,0,0]
        # positions_list = [0.2,0.2,-0.2,0.3,-0.2,0.5,0.08]
        # 需要把夹爪数据也放进去
        # positions_list = np.append(positions_list, [0])
        self.piper_controller.set_joints(positions_list)


class JointCommandPublisher_doublelinker():

    def __init__(self, hand_type):
        super().__init__()

        if not rclpy.ok():
            rclpy.init(args=None)
        node_name = f"replay_publisher_doublelinker_{hand_type}"  # "joint_command_publisher"
        self.node = Node(node_name)

        
        """创建关节状态消息"""
        self.msg = JointState()
        
        # 创建发布者 - 左臂/右臂
        self.double_publisher = self.node.create_publisher(
            JointState, 
            f'/{hand_type}_arm_joint_control', 
            10
        )
        
        # 关节名称（假设7个关节）
        self.joint_names = [
            'shoulder_pan_joint',    # 肩部平移
            'shoulder_lift_joint',   # 肩部抬升
            'elbow_joint',          # 肘部
            'wrist_1_joint',        # 腕部1
            'wrist_2_joint',        # 腕部2
            'wrist_3_joint',        # 腕部3
            'gripper_joint'         # 夹爪
        ]

        # 设置关节名称
        self.msg.name = self.joint_names
        
        # 关节数量
        self.num_joints = len(self.joint_names)
        
    
    def set_position(self, positions_list, velocities_list=None):
        """设置右臂目标位置和速度"""
        if len(positions_list) != self.num_joints:
            self.get_logger().error(f'位置数据长度应为{self.num_joints}，实际为{len(positions_list)}')
            return False
        
        self.target_positions = list(positions_list)
        
        if velocities_list is not None:
            if len(velocities_list) != self.num_joints:
                self.get_logger().error(f'速度数据长度应为{self.num_joints}，实际为{len(velocities_list)}')
                return False
            self.target_velocities = list(velocities_list)
        else:
            self.target_velocities = [0.0] * self.num_joints
        
        # 设置位置、速度、力矩
        # 当前目标位置和速度
        self.msg.position = self.target_positions
        self.msg.velocity = self.target_velocities
        self.msg.effort = [0.0] * self.num_joints
        # 发布右臂命令
        self.double_publisher.publish(self.msg)
        
        # self.get_logger().info(f'设置右臂目标位置: {positions_list}')
        return True

    def destroy(self):
        self.node.destroy_node()



class JointCommandPublisher_linkerhand():
    def __init__(self, hand_type, hand_joint, is_arc):
        super().__init__()
    
        self.hand_type = hand_type
        self.hand_joint = hand_joint
        self.is_arc = is_arc

        # 【步骤2】建立ROS节点
        self.publisher = None
        self.publisher_arc = None
        self.joint_state = JointState()
        self.joint_state.header = Header()

        if not rclpy.ok():
            rclpy.init(args=None)
        node_name = f"replay_publisher_linkerhand_{hand_type}"  # "hand_control_node"
        self.node = Node(node_name)
        
        # # 声明参数
        # node.declare_parameter('hand_type', hand_type)
        # node.declare_parameter('hand_joint', hand_joint)
        # node.declare_parameter('topic_hz', hz)
        # node.declare_parameter('is_arc', is_arc)
        
        # # 获取参数（保留，用于学习）
        # hand_type = node.get_parameter('hand_type').value
        # hand_joint = node.get_parameter('hand_joint').value
        # hz = node.get_parameter('topic_hz').value
        # is_arc = node.get_parameter('is_arc').value
        

        if is_arc == True:
            # 创建发布者
            self.publisher_arc = self.node.create_publisher(
                JointState, f'/cb_{hand_type}_hand_control_cmd_arc', 10
            )
        # 创建发布者
        self.publisher = self.node.create_publisher(
            JointState, f'/cb_{hand_type}_hand_control_cmd', 10
        )


    def set_position(self, positions_list):
        # 发布关节状态
        self.joint_state.header.stamp = self.node.get_clock().now().to_msg()
        self.joint_state.position = [float(pos) for pos in positions_list]
        # self.joint_state.velocity = [0.1] * len(positions_list)
        # self.joint_state.effort = [0.01] * len(positions_list)
        # 如果有关节名称，添加到消息中
        hand_config = HandConfig.from_hand_type(self.hand_joint)
        if len(hand_config.joint_names) == len(positions_list):
            if hand_config.joint_names_en != None:
                self.joint_state.name = hand_config.joint_names_en
            else:
                self.joint_state.name = hand_config.joint_names
            
        self.publisher.publish(self.joint_state)
        # 这里需要做一下手指的限位
        if self.is_arc == True:
            if self.hand_joint == "O6":
                if self.hand_type == "left":
                    pose = range_to_arc_left(positions_list, self.hand_joint)
                elif self.hand_type == "right":
                    pose = range_to_arc_right(positions_list, self.hand_joint)
            elif self.hand_joint == "L7" or self.hand_joint == "L21" or self.hand_joint == "L25":
                if self.hand_type == "left":
                    pose = range_to_arc_left(positions_list, self.hand_joint)
                elif self.hand_type == "right":
                    pose = range_to_arc_right(positions_list, self.hand_joint)
            elif self.hand_joint == "L10":
                if self.hand_type == "left":
                    pose = range_to_arc_left_10(positions_list)
                elif self.hand_type == "right":
                    pose = range_to_arc_right_10(positions_list)
            elif self.hand_joint == "L20":
                if self.hand_type == "left":
                    pose = range_to_arc_left_l20(positions_list)
                elif self.hand_type == "right":
                    pose = range_to_arc_right_l20(positions_list)
            else:
                print(f"当前{self.hand_joint} {self.hand_type}不支持弧度转换", flush=True)
            self.joint_state.position = [float(pos) for pos in pose]
            self.publisher_arc.publish(self.joint_state)
        # print("info", "关节状态已发布")


        # ############################################
    # #【可保留】下列代码可以保留，用于学习
    # joint_state = JointState() 
    # rclpy.init(args=None)
    # node = Node("dong_test_sender")
    # rate = 1.0 / 30  # 60 FPS
    # pub = node.create_publisher(JointState, '/cb_right_hand_control_cmd', 10)
    # now = node.get_clock().now()
    
    # joint_state.header = Header()

    # joint_state.header.stamp = Time(sec=int(now.nanoseconds // 1e9), 
    #                         nanosec=int(now.nanoseconds % 1e9))

    # hand_config = HandConfig.from_hand_type(hand_joint)
    # joint_state.name = hand_config.joint_names
    # joint_state.velocity = [0] * len(hand_config.joint_names)  # 与position数组长度相同，全部填充为0
    # joint_state.effort = [0] * len(hand_config.joint_names)  # 为每个关节设置努力为零
    # pub.publish(joint_state)
    # count = 0
    # while rclpy.ok():  # 持续1秒

    #     hand = {"joint1":255,   #拇指根部弯曲
    #     "joint2":128,   #拇指侧摆
    #     "joint3":255,   #食指根部弯曲  
    #     "joint4":255,   #中指根部弯曲
    #     "joint5":255,   #无名指根部弯曲
    #     "joint6":255,   #小指根部弯曲
    #     "joint7":128,   #食指侧摆
    #     "joint8":128,   #中指侧摆
    #     "joint9":128,   #无名指侧摆
    #     "joint10":255,  #拇指旋转
    #     }

    #     position = hand.values()
    #     if(position is not None):
    #         joint_state.position = position
    #     # rospy.loginfo(f"Publishing joint states {joint_state.__str__}")
    #     pub.publish(joint_state)
    #     time.sleep(rate)
    #     count = count + 1
    #     print(count)
    # #################################


    def destroy(self):
        self.node.destroy_node()


################################################

# 读取CSV文件
def read_csv_file(file_path):
    df = pd.read_csv(file_path)
    data = df.to_numpy()
    return data


def read_json_file(file_path):
    # 读取文件内容
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    # 解析 JSON
    data = json.loads(content)
    return data


def read_npz_file(file_path):

    data = np.load(file_path)
    print("=== NPZ 文件内容分析 ===")
    print(f"文件路径: {file_path}")
    print(f"包含的数组数量: {len(data.files)}")
    print("数组列表:", data.files)
    return data


# 寻找所有的相机文件
def find_cameras(data_json_camera):
    have_video_list = []
    # 根据json文件，找到文件夹中的视频
    for cam_name in data_json_camera:
        print(cam_name)
        cam_data = data_json_camera[cam_name]
        if cam_data["left"] is not None:
            have_video_list.append(cam_name)
    return have_video_list


# 打开相机文件
class Open_Video_Files():
    def __init__(self, video_path_list):

        # 先对窗口进行命名
        self.window_titles = [os.path.split(path)[-1] for path in video_path_list]
        
        # 创建视频捕获对象
        self.caps = [cv2.VideoCapture(path) for path in video_path_list]
        
        # 检查视频是否成功打开
        for i, cap in enumerate(self.caps):
            if not cap.isOpened():
                print(f"错误：无法打开视频 {video_path_list[i]}")
                return
    
    def read(self, layout="horizontal"):
        frames = []
        # 读取所有视频的帧
        for i, cap in enumerate(self.caps):
            ret, frame = cap.read()
            
            if ret:
                all_closed = False
                frames.append(frame)
                
        combined_frame = combine_frames(frames, layout)
        # # 显示合成视频窗口
        # cv2.imshow("combined_frame", combined_frame)

        # cv2.waitKey(1)
        return frames, combined_frame
    
    def read_by_idx(self, idx, layout="horizontal"):
        frames = []
        # 读取所有视频的帧
        for i, cap in enumerate(self.caps):
            # 获取视频总帧数
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            # 如果索引超出范围，使用最后一帧
            actual_idx = min(idx, total_frames - 1) if total_frames > 0 else 0
            
            cap.set(cv2.CAP_PROP_POS_FRAMES, actual_idx)
            ret, frame = cap.read()
            
            if ret and frame is not None:
                frames.append(frame)
            else:
                # 如果读取失败，尝试读取最后一帧
                if total_frames > 0:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, total_frames - 1)
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        frames.append(frame)
                                
        combined_frame = combine_frames(frames, layout)
        # # 显示合成视频窗口
        # cv2.imshow("combined_frame", combined_frame)

        # cv2.waitKey(1)
        return frames, combined_frame


# 将多相机图像合并为一个
def combine_frames(frames, layout='horizontal'):
    """合并多个帧为一个图像"""
    valid_frames = [f for f in frames if f is not None]
    if not valid_frames:
        return None
    
    # 调整所有帧到相同高度
    heights = [f.shape[0] for f in valid_frames]
    min_height = min(heights)
    
    resized_frames = []
    for frame in valid_frames:
        ratio = min_height / frame.shape[0]
        new_width = int(frame.shape[1] * ratio)
        resized = cv2.resize(frame, (new_width, min_height))
        resized_frames.append(resized)
    
    if layout == 'horizontal':
        # 水平拼接
        return np.hstack(resized_frames)
    elif layout == 'vertical':
        # 垂直拼接
        return np.vstack(resized_frames)
    elif layout == 'grid':
        # 网格布局（2x2等）
        rows = []
        for i in range(0, len(resized_frames), 2):
            row_frames = resized_frames[i:i+2]
            # 确保每行有相同数量的帧
            while len(row_frames) < 2 and len(row_frames) > 0:
                # 添加空白帧
                blank = np.zeros_like(row_frames[0])
                row_frames.append(blank)
            if row_frames:
                rows.append(np.hstack(row_frames))
        return np.vstack(rows)



def main():

    # 输入数据路径
    # 修改为你要回放的episode路径
    dataset_folder_path = r"/home/zmh/zmh/code/linkerhand_data_collection_ws/collection_data/linkerhand_piper_grasp/session_2025-11-10_11-14-26/episode_000003"

    
    FPS_value = 15      # 回放帧率（建议10-30，根据原始采集帧率调整）
    speed_up = 1.0      # 加速倍数（1.0=正常速度, 2.0=2倍速）
    is_arc = True       # 是否使用弧度模式（True=弧度, False=范围值0-255）
        

    # 【步骤1】解析数据文件
  
    # 解析视频文件
    folder_path_camera = os.path.join(dataset_folder_path, "extracted", "cameras_csv")
    # 如果有视频文件存在就生成列表
    if os.path.exists(folder_path_camera):
        have_video_list = os.listdir(folder_path_camera)
        have_video_list = [name.split(".")[0] for name in have_video_list]
        video_path_list = [os.path.join(dataset_folder_path, "cameras", name+".mp4") for name in have_video_list]
        Video_File = Open_Video_Files(video_path_list)
    else:
        Video_File = None

    # 解析机器人基本信息
    json_file_path_robot = os.path.join(dataset_folder_path, "manifest.json")
    data_json_robot = read_json_file(json_file_path_robot)
    frame_count = data_json_robot["frame_count"]
    active_sides = data_json_robot["metadata"]["hardware"]["active_sides"]
    arm_joints = data_json_robot["metadata"]["hardware"]["arm_joints"]
    hand_joints = data_json_robot["metadata"]["hardware"]["hand_joints"]
    delay_time = data_json_robot["metadata"]["dt"]
    task_name = data_json_robot["metadata"]["task_name"]

    # 获取机器人的左右臂和左右手是否存在
    left_arm_exist = active_sides["left_arm"]
    right_arm_exist = active_sides["right_arm"]
    left_hand_exist = active_sides["left_hand"]
    right_hand_exist = active_sides["right_hand"]
    
    # 获取机器人左右臂和左右手的自由度
    left_arm_dof = arm_joints["left_arm_dof"]
    right_arm_dof = arm_joints["right_arm_dof"]
    left_hand_dof = hand_joints["left_hand_dof"]
    right_hand_dof = hand_joints["right_hand_dof"]

    # 根据自由度数，生成相应的机械手型号
    # 创建关节数到手部型号的映射字典
    hand_dof_to_name_dict = {
        6: "O6",
        7: "L7", 
        10: "L10",
        20: "L20",
        21: "L21",
        25: "L25"
    }
    left_hand_name = hand_dof_to_name_dict.get(left_hand_dof)
    right_hand_name = hand_dof_to_name_dict.get(right_hand_dof)

    # 解析机器人动作文件
    csv_file_path_actions = os.path.join(dataset_folder_path, "extracted", "telemetry_csv", "actions.csv")
    data_robot_action = read_csv_file(csv_file_path_actions)[:, 1:]

    # 获取机器人动作序列
    left_arm_action = data_robot_action[:, 0:left_arm_dof]
    right_arm_action = data_robot_action[:, left_arm_dof:left_arm_dof+right_arm_dof]
    left_hand_action = data_robot_action[:, left_arm_dof+right_arm_dof:left_arm_dof+right_arm_dof+left_hand_dof]
    right_hand_action = data_robot_action[:, left_arm_dof+right_arm_dof+left_hand_dof:left_arm_dof+right_arm_dof+left_hand_dof+right_hand_dof]


    # 用户可以自定义FPS
    # 计算数据采集时使用的FPS
    current_FPS = 1.0/delay_time
    step = current_FPS/FPS_value
    delay_time_vale = 1/FPS_value

    # 均匀抽帧
    sampled_idx_list = []
    sampled_idx = 0
    while(1):
        # 修复：使用 < 而不是 <=，因为索引从0开始，最大索引是 frame_count - 1
        if sampled_idx < frame_count:
            # 确保索引不会超出范围（round可能产生超出范围的索引）
            idx = min(int(round(sampled_idx)), frame_count - 1)
            sampled_idx_list.append(idx)
        else:
            break
        sampled_idx = sampled_idx + step
   

    if task_name == "linkerhand_piper_grasp":
        # 建立ROS节点
        # 初始化变量，避免清理时未定义错误
        JCP_linkerhand_left = None
        JCP_linkerhand_right = None
        
        if left_hand_exist == True:
            JCP_piper = JointCommandPublisher_piper("left")
            JCP_linkerhand_left = JointCommandPublisher_linkerhand("left", left_hand_name, is_arc)
        if right_hand_exist == True:
            JCP_piper = JointCommandPublisher_piper("right")
            JCP_linkerhand_right = JointCommandPublisher_linkerhand("right", right_hand_name, is_arc)


        for i in sampled_idx_list:
            time_start = time.time()

            # 显示视频
            if Video_File is not None:
                # 根据索引进行显示
                frames, combined_frame = Video_File.read_by_idx(i)
                # 按照顺序进行显示
                # frames, combined_frame = Video_File.read()

                # 显示合成视频窗口（检查combined_frame是否有效）
                if combined_frame is not None and combined_frame.size > 0:
                    cv2.imshow("combined_frame", combined_frame)
                    cv2.waitKey(1)
                # 如果视频读取失败，跳过显示但不影响机器人控制

            # 控制机器人

            # 控制机械臂
            if left_arm_exist == True:
                positions = np.append(left_arm_action[i], [0])
                JCP_piper.set_position(positions)

            if right_arm_exist == True:
                positions = np.append(right_arm_action[i], [0])
                JCP_piper.set_position(positions)

            
            # 控制灵巧手
            if left_hand_exist == True:
                positions = left_hand_action[i]
                JCP_linkerhand_left.set_position(positions)
            if right_hand_exist == True:
                positions = right_hand_action[i]
                JCP_linkerhand_right.set_position(positions)
            time.sleep(delay_time_vale/speed_up)
            print("正在运行索引:", i, "| 当前FPS:", 1/(time.time()-time_start))

    elif task_name == "double_linkerhand_grasp":
        # 建立ROS节点
        # 初始化变量，避免清理时未定义错误
        JCP_linkerhand_left = None
        JCP_linkerhand_right = None
        
        if left_hand_exist == True:
            # 创建发布器
            JCP_double_left = JointCommandPublisher_doublelinker("left")
            JCP_linkerhand_left = JointCommandPublisher_linkerhand("left", left_hand_name, is_arc)
        if right_hand_exist == True:
            JCP_double_right = JointCommandPublisher_doublelinker("right")
            JCP_linkerhand_right = JointCommandPublisher_linkerhand("right", right_hand_name, is_arc)


        for i in sampled_idx_list:
            time_start = time.time()

            # 显示视频
            if Video_File is not None:
                # 根据索引进行显示
                frames, combined_frame = Video_File.read_by_idx(i)
                # 按照顺序进行显示
                # frames, combined_frame = Video_File.read()

                # 显示合成视频窗口（检查combined_frame是否有效）
                if combined_frame is not None and combined_frame.size > 0:
                    cv2.imshow("combined_frame", combined_frame)
                    cv2.waitKey(1)
                # 如果视频读取失败，跳过显示但不影响机器人控制

            # 控制机器人

            # 控制机械臂
            if left_arm_exist == True:
                positions = left_arm_action[i]
                JCP_double_left.set_position(positions)
                print("left_arm_exist:", positions)

            if right_arm_exist == True:
                positions = right_arm_action[i]
                JCP_double_right.set_position(positions)
                print("right_arm_exist:", positions)

            
            # 控制灵巧手
            if left_hand_exist == True:
                positions = left_hand_action[i]
                JCP_linkerhand_left.set_position(positions)
                print("left_hand_exist:", positions)
            if right_hand_exist == True:
                positions = right_hand_action[i]
                JCP_linkerhand_right.set_position(positions)
                print("right_hand_exist:", positions)
            time.sleep(delay_time_vale/speed_up)
            print("正在运行索引:", i, "| 当前FPS:", 1/(time.time()-time_start))

    # 清理ROS2
    if rclpy.ok():
        # 只销毁已初始化的节点，避免未定义变量错误
        if JCP_linkerhand_left is not None:
            JCP_linkerhand_left.destroy()
        if JCP_linkerhand_right is not None:
            JCP_linkerhand_right.destroy()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
