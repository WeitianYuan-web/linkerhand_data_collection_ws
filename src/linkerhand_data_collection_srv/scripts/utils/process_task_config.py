#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务配置处理器 - 从 JSON 配置生成/更新 YAML 任务配置
支持从外部程序传入的 JSON 配置参数
"""

import json
import yaml
from pathlib import Path
from typing import Dict, Any, Optional
import os
import sys

# 添加当前目录到 Python 路径以导入 hardware_detector
sys.path.insert(0, str(Path(__file__).parent))

# 获取工作空间根目录
current_file = Path(__file__).resolve()
workspace_root = None

for parent in current_file.parents:
    if (parent / 'src').is_dir() and (parent / 'src' / 'linkerhand_data_collection_srv').is_dir():
        workspace_root = parent
        break

if workspace_root is None:
    workspace_root = Path(os.environ.get('ROS_WORKSPACE', Path.cwd()))

# 配置文件路径
CONFIG_DIR = workspace_root / 'src' / 'linkerhand_data_collection_srv' / 'configs'
TASKS_DIR = CONFIG_DIR / 'tasks'
TASK_CONFIG_JSON = workspace_root / 'task_config.json'
# LinkerHand SDK setting.yaml 路径（已弃用，linkerhand_cl 使用 task_config.json）
# 保留此路径仅用于向后兼容，如果文件不存在则跳过
LINKERHAND_SDK_CONFIG = workspace_root / 'src' / 'linker_hand_ros2_sdk' / 'linker_hand_ros2_sdk' / 'linker_hand_ros2_sdk' / 'LinkerHand' / 'config' / 'setting.yaml'


class TaskConfigProcessor:
    """任务配置处理器 - 将 JSON 配置转换为 YAML 任务配置"""
    
    def __init__(self):
        self._hardware_specs_cache = None
    
    def _get_device_config(self) -> Dict[str, Any]:
        """
        获取设备配置（检测硬件信息）
        
        Returns:
            设备配置字典
        """
        # 如果已缓存，直接返回
        if self._hardware_specs_cache is not None:
            return self._hardware_specs_cache
        
        # 尝试检测硬件信息
        try:
            from hardware_detector import detect_hardware
            hardware_specs = detect_hardware()
            print(f"✅ 检测到硬件信息: GPU={hardware_specs['gpu']}, CPU={hardware_specs['cpu']}, Memory={hardware_specs['memory']}, Storage={hardware_specs['storage']}")
        except Exception as e:
            print(f"⚠️  无法检测硬件信息: {e}")
            print("⚠️  使用默认硬件信息")
            hardware_specs = {
                'gpu': 'Unknown',
                'cpu': 'Unknown',
                'memory': 'Unknown',
                'storage': 'Unknown',
            }
        
        self._hardware_specs_cache = {
            'device_number': '123456',
            'device_type': 'gather',
            'hardware_specs': hardware_specs
        }
        
        return self._hardware_specs_cache
    
    def load_json_config(self, json_path: Optional[Path] = None) -> Optional[Dict[str, Any]]:
        """
        加载 JSON 配置文件
        
        Args:
            json_path: JSON 文件路径，如果为 None 则使用默认路径
        
        Returns:
            JSON 配置字典，如果失败返回 None
        """
        if json_path is None:
            json_path = TASK_CONFIG_JSON
        
        if not json_path.exists():
            print(f"Error: JSON config file not found: {json_path}")
            return None
        
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            return config
        except Exception as e:
            print(f"Error loading JSON config: {e}")
            return None
    
    def determine_task_name(self, json_config: Dict[str, Any]) -> str:
        """
        根据 JSON 配置确定任务名称
        
        Args:
            json_config: JSON 配置字典
        
        Returns:
            任务名称
        """
        arm_type = json_config.get('armType', 'piper')
        hand_side = json_config.get('handSide', 'left')
        
        if arm_type == 'linker' and hand_side == 'both':
            return 'double_linkerhand_grasp'
        elif arm_type == 'piper':
            return 'linkerhand_piper_grasp'
        else:
            # 自定义任务名称
            return f'{arm_type}_{hand_side}_grasp'
    
    def json_to_yaml_config(self, json_config: Dict[str, Any]) -> Dict[str, Any]:
        """
        将 JSON 配置转换为 YAML 任务配置
        
        Args:
            json_config: JSON 配置字典
            格式：
            {
              "armType": "piper" | "linker",
              "handSide": "left" | "right" | "both",
              "handModel": "O6" | "L10" | "L20",
              "collectTactile": true | false,
              "cameraD455Count": 1,
              "cameraD405Count": 0 | 1 | 2
            }
        
        Returns:
            YAML 配置字典
        """
        # 提取 JSON 参数
        arm_type = json_config.get('armType', 'piper')
        hand_side = json_config.get('handSide', 'left')
        hand_model = json_config.get('handModel', 'O6')
        collect_tactile = json_config.get('collectTactile', False)
        camera_d455_count = json_config.get('cameraD455Count', 1)
        camera_d405_count = json_config.get('cameraD405Count', 0)
        
        # 确定任务名称
        task_name = self.determine_task_name(json_config)
        
        # 确定臂部 DoF
        if arm_type == 'piper':
            arm_dof = 6
            arm_description = "Piper"
        elif arm_type == 'linker':
            arm_dof = 7
            arm_description = "Robstride"
        else:
            arm_dof = 6
            arm_description = arm_type
        
        # 确定手部配置
        hand_model_upper = hand_model.upper()
        hand_joint_map = {
            'O6': 6,
            'L6': 6,
            'L6P': 6,
            'L7': 7,
            'L10': 10,
            'L10V6': 10,
            'L10V7': 10,
            'L20': 20,
            'L21': 21,
            'L25': 25,
        }
        hand_dof = hand_joint_map.get(hand_model_upper, 10)  # Default to 10 if unknown
        
        # 确定硬件启用配置
        if hand_side == 'both':
            setup_left_arm = True
            setup_right_arm = True
            setup_left_hand = True
            setup_right_hand = True
        elif hand_side == 'left':
            setup_left_arm = True
            setup_right_arm = False
            setup_left_hand = True
            setup_right_hand = False
        elif hand_side == 'right':
            setup_left_arm = False
            setup_right_arm = True
            setup_left_hand = False
            setup_right_hand = True
        else:
            # 默认右手
            setup_left_arm = False
            setup_right_arm = True
            setup_left_hand = False
            setup_right_hand = True
        
        # 确定相机 preset（简化版）
        # linker任务: A. d455+2个d405  B. 单d455
        # piper任务: A. d455+d405     B. 单d455
        if arm_type == 'linker' and hand_side == 'both':
            # LinkerHand双臂任务
            if camera_d455_count != 1:
                raise ValueError("double_linkerhand_grasp 任务必须包含一个 D455 顶视相机")
            if camera_d405_count == 0:
                # Config B: 单个D455
                camera_preset = 'intel_d455_single_top'
            elif camera_d405_count == 2:
                # Config A: D455 + 双D405
                camera_preset = 'intel_d455_with_dual_wrist'
            else:
                raise ValueError("double_linkerhand_grasp 任务仅支持 0 或 2 个 D405 相机")
        elif arm_type == 'piper':
            # Piper单臂任务
            if camera_d455_count != 1:
                raise ValueError("linkerhand_piper_grasp 任务必须包含一个 D455 顶视相机")
            if camera_d405_count == 0:
                # Config B: 单个D455
                camera_preset = 'intel_d455_single_top'
            elif camera_d405_count == 1:
                # Config A: D455 + 单D405，根据手侧决定左右
                if hand_side == 'left':
                    camera_preset = 'intel_d455_with_left_wrist'
                elif hand_side == 'right':
                    camera_preset = 'intel_d455_with_right_wrist'
                else:
                    raise ValueError("Piper 任务 handSide 必须为 left 或 right 才能匹配单个 D405 相机")
            else:
                raise ValueError("linkerhand_piper_grasp 任务仅支持 0 或 1 个 D405 相机")
        else:
            # 其他情况使用默认配置
            if camera_d455_count == 1 and camera_d405_count == 0:
                camera_preset = 'intel_d455_single_top'
            else:
                camera_preset = 'intel_d455_single_top'
        
        # 确定硬件 preset（对齐 hardware_presets.yaml）
        if arm_type == 'piper':
            hardware_preset = 'piper_linkerhand_single'
        elif arm_type == 'linker' and hand_side == 'both':
            hardware_preset = 'linkerarm_linkerhand_dual'
        else:
            hardware_preset = 'custom'
        
        # 生成任务描述
        if hand_side == 'both':
            description = f"Dual arm bimanual task with {arm_description} arms and LinkerHand {hand_model}"
        else:
            description = f"Single {hand_side} arm task with {arm_description} arm and LinkerHand {hand_model}"
        
        # 构建 YAML 配置
        yaml_config = {
            'task_name': task_name,
            'description': description,
            'hardware': {
                'preset': hardware_preset,
                'active_sides': {
                    'left_arm': setup_left_arm,
                    'right_arm': setup_right_arm,
                    'left_hand': setup_left_hand,
                    'right_hand': setup_right_hand,
                },
                'arm_joints': {
                    'left_arm_dof': arm_dof if setup_left_arm else 0,
                    'right_arm_dof': arm_dof if setup_right_arm else 0,
                },
                'hand_joints': {
                    'left_hand_dof': hand_dof if setup_left_hand else 0,
                    'right_hand_dof': hand_dof if setup_right_hand else 0,
                },
                'topics': {
                    'left_hand_state': '/cb_left_hand_state_arc',
                    'right_hand_state': '/cb_right_hand_state_arc',
                    'left_hand_cmd': '/cb_left_hand_control_cmd',
                    'right_hand_cmd': '/cb_right_hand_control_cmd',
                    'left_hand_tactile': '/cb_left_hand_matrix_touch',
                    'right_hand_tactile': '/cb_right_hand_matrix_touch',
                    'left_arm_state': '/left_arm_joint_state',
                    'right_arm_state': '/right_arm_joint_state',
                    'left_arm_cmd': '/left_arm_joint_control',
                    'right_arm_cmd': '/right_arm_joint_control',
                }
            },
            'cameras': {
                'preset': camera_preset,
                'stereo_mode': False,
                'enable_top_camera': True,
                'enable_left_wrist_camera': camera_preset in [
                    'intel_d455_with_left_wrist',
                    'intel_d455_with_dual_wrist'
                ],
                'enable_right_wrist_camera': camera_preset in [
                    'intel_d455_with_right_wrist',
                    'intel_d455_with_dual_wrist'
                ],
                'video_settings': {
                    'quality': 'high',
                    'fps': 30,
                    'use_compression': True,
                }
            },
            'collection': {
                'dataset_dir': f'collection_data/{task_name}',
                'num_episodes': 1000,
                'episode_len': 1000,
                'dt': 0.04,
            },
            'device': self._get_device_config()
        }
        
        # 添加触觉传感器配置
        yaml_config['hardware']['collect_tactile'] = collect_tactile
        
        return yaml_config
    
    def update_linkerhand_sdk_config(self, hand_model: str, hand_side: str):
        """
        更新 LinkerHand SDK 的 setting.yaml
        
        Args:
            hand_model: 手部型号 (O6, L10, L20)
            hand_side: 手部位置 (left, right, both)
        """
        if not LINKERHAND_SDK_CONFIG.exists():
            print(f"Warning: LinkerHand SDK config not found: {LINKERHAND_SDK_CONFIG}")
            return
        
        try:
            with open(LINKERHAND_SDK_CONFIG, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            if 'LINKER_HAND' not in config:
                config['LINKER_HAND'] = {}
            
            hand_model_upper = hand_model.upper()
            
            if hand_side in ['left', 'both']:
                if 'LEFT_HAND' not in config['LINKER_HAND']:
                    config['LINKER_HAND']['LEFT_HAND'] = {}
                config['LINKER_HAND']['LEFT_HAND']['JOINT'] = hand_model_upper
            
            if hand_side in ['right', 'both']:
                if 'RIGHT_HAND' not in config['LINKER_HAND']:
                    config['LINKER_HAND']['RIGHT_HAND'] = {}
                config['LINKER_HAND']['RIGHT_HAND']['JOINT'] = hand_model_upper
            
            # 保存更新后的配置
            with open(LINKERHAND_SDK_CONFIG, 'w', encoding='utf-8') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)
            
            print(f"✅ Updated LinkerHand SDK config: {hand_model_upper} ({hand_side})")
        except Exception as e:
            print(f"Error updating LinkerHand SDK config: {e}")
    
    def save_yaml_config(self, yaml_config: Dict[str, Any], output_path: Optional[Path] = None):
        """
        保存 YAML 配置到文件
        
        Args:
            yaml_config: YAML 配置字典
            output_path: 输出文件路径，如果为 None 则根据 task_name 自动生成
        """
        if output_path is None:
            task_name = yaml_config.get('task_name', 'custom_task')
            output_path = TASKS_DIR / f'{task_name}.yaml'
        
        # 确保目录存在
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        try:
            with open(output_path, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            print(f"✅ Saved task config: {output_path}")
        except Exception as e:
            print(f"Error saving YAML config: {e}")
    
    def process_json_config(self, json_path: Optional[Path] = None, save_yaml: bool = True, update_sdk: bool = True) -> Optional[Dict[str, Any]]:
        """
        处理 JSON 配置并生成/更新 YAML 任务配置
        
        Args:
            json_path: JSON 文件路径
            save_yaml: 是否保存 YAML 配置文件
            update_sdk: 是否更新 LinkerHand SDK 配置
        
        Returns:
            生成的 YAML 配置字典，如果失败返回 None
        """
        # 1. 加载 JSON 配置
        json_config = self.load_json_config(json_path)
        if json_config is None:
            return None
        
        print(f"📋 Processing JSON config:")
        print(f"   armType: {json_config.get('armType')}")
        print(f"   handSide: {json_config.get('handSide')}")
        print(f"   handModel: {json_config.get('handModel')}")
        print(f"   collectTactile: {json_config.get('collectTactile')}")
        print(f"   cameraD455Count: {json_config.get('cameraD455Count')}")
        print(f"   cameraD405Count: {json_config.get('cameraD405Count')}")
        
        # 2. 转换为 YAML 配置
        yaml_config = self.json_to_yaml_config(json_config)
        
        # 3. 更新 LinkerHand SDK 配置
        if update_sdk:
            hand_model = json_config.get('handModel', 'O6')
            hand_side = json_config.get('handSide', 'left')
            self.update_linkerhand_sdk_config(hand_model, hand_side)
        
        # 4. 保存 YAML 配置
        if save_yaml:
            self.save_yaml_config(yaml_config)
        
        print(f"✅ Task configuration processed: {yaml_config['task_name']}")
        return yaml_config


def main():
    """主函数 - 命令行入口"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Process task configuration from JSON to YAML')
    parser.add_argument('--json', type=str, help='Path to JSON config file (default: task_config.json)')
    parser.add_argument('--no-save', action='store_true', help='Do not save YAML config')
    parser.add_argument('--no-update-sdk', action='store_true', help='Do not update LinkerHand SDK config')
    
    args = parser.parse_args()
    
    processor = TaskConfigProcessor()
    json_path = Path(args.json) if args.json else None
    
    yaml_config = processor.process_json_config(
        json_path=json_path,
        save_yaml=not args.no_save,
        update_sdk=not args.no_update_sdk
    )
    
    if yaml_config:
        print("\n✅ Configuration processing complete!")
        print(f"   Task name: {yaml_config['task_name']}")
        print(f"   Dataset dir: {yaml_config['collection']['dataset_dir']}")
    else:
        print("\n❌ Configuration processing failed!")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())

