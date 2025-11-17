#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配置加载器 - 从 YAML 文件加载任务配置
统一管理所有配置文件的加载逻辑
"""

from pathlib import Path
import os
import json
import yaml
from typing import Dict, Optional, Any
from copy import deepcopy

# 获取工作空间根目录
current_file = Path(__file__).resolve()
workspace_root = None

for parent in current_file.parents:
    if (parent / 'src').is_dir() and (parent / 'src' / 'linkerhand_data_collection_srv').is_dir():
        workspace_root = parent
        break

if workspace_root is None:
    workspace_root = Path(os.environ.get('ROS_WORKSPACE', Path.cwd()))

# 配置文件路径（优先使用可写项目根目录，若缺失则回退到安装路径）
CONFIG_DIR = workspace_root / 'src' / 'linkerhand_data_collection_srv' / 'configs'
if not CONFIG_DIR.exists():
    CONFIG_DIR = Path(__file__).resolve().parents[2] / 'share' / 'linkerhand_data_collection_srv' / 'configs'
TASKS_DIR = CONFIG_DIR / 'tasks'
HARDWARE_PRESETS_FILE = CONFIG_DIR / 'hardware_presets.yaml'
CAMERA_SERIAL_FILE = CONFIG_DIR / 'camera_serial_numbers.yaml'

# LinkerHand SDK setting.yaml 路径（已弃用，linkerhand_cl 使用 task_config.json）
# 保留此路径仅用于向后兼容，如果文件不存在则跳过
LINKERHAND_SDK_CONFIG = workspace_root / 'src' / 'linker_hand_ros2_sdk' / 'linker_hand_ros2_sdk' / 'linker_hand_ros2_sdk' / 'LinkerHand' / 'config' / 'setting.yaml'
# 新的配置路径（linkerhand_cl 使用 task_config.json）
TASK_CONFIG_JSON = workspace_root / 'task_config.json'

# 数据收集根目录
DATA_DIR = str(workspace_root / 'collection_data')


class ConfigLoader:
    """配置加载器类 - 统一管理所有配置加载"""
    
    def __init__(self):
        self._hardware_presets_cache = None
        self._camera_serial_cache = None
        self._hand_config_cache = None
    
    def load_task_config(self, task_name: str) -> Optional[Dict[str, Any]]:
        """
        加载任务配置文件
        
        Args:
            task_name: 任务名称
        
        Returns:
            任务配置字典，如果文件不存在返回 None
        """
        task_yaml_path = TASKS_DIR / f'{task_name}.yaml'
        
        if not task_yaml_path.exists():
            print(f"Warning: Task config file not found: {task_yaml_path}")
            return None
        
        try:
            with open(task_yaml_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            # 添加数据集目录（如果没有指定）
            if 'collection' in config and 'dataset_dir' in config['collection']:
                # 使用配置中的路径
                dataset_dir = config['collection']['dataset_dir']
                if not dataset_dir.startswith('/'):
                    # 相对路径，转换为绝对路径
                    # 如果路径已经包含 collection_data，则直接使用工作空间根目录
                    if dataset_dir.startswith('collection_data/'):
                        config['collection']['dataset_dir'] = str(workspace_root / dataset_dir)
                    else:
                        # 否则拼接到 DATA_DIR
                        config['collection']['dataset_dir'] = str(Path(DATA_DIR) / dataset_dir)
            else:
                # 默认：使用任务名创建数据集目录
                if 'collection' not in config:
                    config['collection'] = {}
                config['collection']['dataset_dir'] = f"{DATA_DIR}/{task_name}/all_episodes"
            
            return config
        except Exception as e:
            print(f"Error loading task config {task_name}: {e}")
            return None
    
    def load_hardware_presets(self) -> Dict[str, Any]:
        """加载硬件预设配置"""
        if self._hardware_presets_cache is not None:
            return self._hardware_presets_cache
        
        if not HARDWARE_PRESETS_FILE.exists():
            print(f"Warning: Hardware presets file not found: {HARDWARE_PRESETS_FILE}")
            return {}
        
        try:
            with open(HARDWARE_PRESETS_FILE, 'r', encoding='utf-8') as f:
                self._hardware_presets_cache = yaml.safe_load(f)
            return self._hardware_presets_cache
        except Exception as e:
            print(f"Error loading hardware presets: {e}")
            return {}
    
    def load_camera_serial_numbers(self) -> Dict[str, str]:
        """加载相机序列号映射"""
        if self._camera_serial_cache is not None:
            return self._camera_serial_cache
        
        if not CAMERA_SERIAL_FILE.exists():
            print(f"Warning: Camera serial file not found: {CAMERA_SERIAL_FILE}")
            return {}
        
        try:
            with open(CAMERA_SERIAL_FILE, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            self._camera_serial_cache = config.get('camera_mapping', {})
            return self._camera_serial_cache
        except Exception as e:
            print(f"Error loading camera serial numbers: {e}")
            return {}
    
    def load_hand_config(self) -> Dict[str, Any]:
        """
        从 LinkerHand SDK 的 setting.yaml 加载手部配置
        
        Returns:
            手部配置字典，包含型号和关节数
        """
        if self._hand_config_cache is not None:
            return self._hand_config_cache
        
        json_config: Dict[str, Any] = {}
        try:
            json_config_path = workspace_root / 'task_config.json'
            if json_config_path.exists():
                with open(json_config_path, 'r', encoding='utf-8') as f:
                    if json_config_path.suffix.lower() in ['.yaml', '.yml']:
                        json_config = yaml.safe_load(f) or {}
                    else:
                        json_config = json.load(f) or {}
        except Exception as e:
            print(f"Warning: Could not load task_config.json: {e}")

        preferred_hand_model = json_config.get('handModel') if isinstance(json_config, dict) else None
        preferred_hand_side = json_config.get('handSide', 'both').lower() if isinstance(json_config, dict) else 'both'

        try:
            if LINKERHAND_SDK_CONFIG.exists():
                with open(LINKERHAND_SDK_CONFIG, 'r', encoding='utf-8') as f:
                    config = yaml.safe_load(f)

                left_hand_model = config.get('LINKER_HAND', {}).get('LEFT_HAND', {}).get('JOINT')
                right_hand_model = config.get('LINKER_HAND', {}).get('RIGHT_HAND', {}).get('JOINT')

                if preferred_hand_model:
                    if preferred_hand_side in ('both', 'left'):
                        left_hand_model = preferred_hand_model
                    if preferred_hand_side in ('both', 'right'):
                        right_hand_model = preferred_hand_model

                left_hand_model = left_hand_model or 'L10'
                right_hand_model = right_hand_model or left_hand_model

                self._hand_config_cache = {
                    'left_hand_model': left_hand_model,
                    'right_hand_model': right_hand_model,
                    'left_hand_joints': self._get_hand_joint_count(left_hand_model),
                    'right_hand_joints': self._get_hand_joint_count(right_hand_model),
                }
                return self._hand_config_cache
        except Exception as e:
            print(f"Warning: Could not load hand config from setting.yaml: {e}")

        fallback_model = preferred_hand_model or 'L10'
        self._hand_config_cache = {
            'left_hand_model': fallback_model,
            'right_hand_model': fallback_model,
            'left_hand_joints': self._get_hand_joint_count(fallback_model),
            'right_hand_joints': self._get_hand_joint_count(fallback_model),
        }
        return self._hand_config_cache
    
    def _get_hand_joint_count(self, hand_model: str) -> int:
        """根据手部型号获取关节数"""
        hand_model_upper = hand_model.upper()
        joint_count_map = {
            'O6': 6,
            'L6': 6,
            'L6P': 6,
            'L7': 7,
            'L10': 10,
            'L20': 20,
            'L21': 25,
            'L25': 25,
        }
        return joint_count_map.get(hand_model_upper, 10)
    
    def has_tactile_support(self, hand_model: str) -> bool:
        """检查手部型号是否支持触觉传感器"""
        hand_model_upper = hand_model.upper()
        # O6 和 L6 型号不支持触觉传感器
        no_tactile_models = ['O6', 'L6', 'L6P']
        return hand_model_upper not in no_tactile_models
    
    def get_task_config_legacy_format(self, task_name: str) -> Optional[Dict[str, Any]]:
        """
        获取任务配置（转换为旧格式以兼容现有代码）
        
        Args:
            task_name: 任务名称
        
        Returns:
            任务配置字典（兼容旧格式）
        """
        config = self.load_task_config(task_name)
        if config is None:
            return None
        
        # 转换为旧格式
        legacy_config = {
            'task_name': task_name,
            'dataset_dir': config.get('collection', {}).get('dataset_dir', f"{DATA_DIR}/{task_name}/all_episodes"),
            'num_episodes': config.get('collection', {}).get('num_episodes', 1000),
            'episode_len': config.get('collection', {}).get('episode_len', 1000),
            'use_video_compression': config.get('cameras', {}).get('video_settings', {}).get('use_compression', True),
            'video_quality': config.get('cameras', {}).get('video_settings', {}).get('quality', 'high'),
            'stereo_mode': config.get('cameras', {}).get('stereo_mode', False),
        }
        
        # 提取硬件配置
        hardware = config.get('hardware', {})
        active_sides = hardware.get('active_sides', {})
        
        legacy_config['setup_left_arm'] = active_sides.get('left_arm', False)
        legacy_config['setup_right_arm'] = active_sides.get('right_arm', False)
        legacy_config['setup_left_hand'] = active_sides.get('left_hand', False)
        legacy_config['setup_right_hand'] = active_sides.get('right_hand', False)
        
        # 提取相机配置
        cameras = config.get('cameras', {})
        camera_preset = cameras.get('preset', 'intel_d455_single_top')
        
        # 根据 preset 生成 camera_names
        camera_names = []
        if camera_preset == 'intel_d455_single_top':
            # Config B: 单个D455顶视相机
            camera_names = ['cam_top']
        elif camera_preset == 'intel_d455_with_right_wrist':
            # Config A for Piper: D455 + 右侧D405
            camera_names = ['cam_top', 'cam_right_wrist']
        elif camera_preset == 'intel_d455_with_left_wrist':
            # Piper 左手配置：D455 + 左侧 D405
            camera_names = ['cam_top', 'cam_left_wrist']
        elif camera_preset == 'intel_d455_with_dual_wrist':
            # Config A for LinkerHand: D455 + 双D405
            camera_names = ['cam_top', 'cam_left_wrist', 'cam_right_wrist']

        # Apply enable overrides if provided (for backward compatibility)
        enable_top = cameras.get('enable_top_camera')
        enable_left = cameras.get('enable_left_wrist_camera')
        enable_right = cameras.get('enable_right_wrist_camera')
        if enable_top is False and 'cam_top' in camera_names:
            camera_names.remove('cam_top')
        if enable_left is False and 'cam_left_wrist' in camera_names:
            camera_names.remove('cam_left_wrist')
        if enable_right is False and 'cam_right_wrist' in camera_names:
            camera_names.remove('cam_right_wrist')

        legacy_config['camera_names'] = camera_names
        # Also include enable flags if they exist in the task config
        cameras_section = config.get('cameras', {})
        legacy_config['enable_top_camera'] = cameras_section.get('enable_top_camera')
        legacy_config['enable_left_wrist_camera'] = cameras_section.get('enable_left_wrist_camera')
        legacy_config['enable_right_wrist_camera'] = cameras_section.get('enable_right_wrist_camera')
        
        return legacy_config
    
    def get_arm_joint_config(self, task_name: str) -> Dict[str, int]:
        """
        获取臂部关节配置
        
        Args:
            task_name: 任务名称
        
        Returns:
            臂部 DoF 配置字典
        """
        config = self.load_task_config(task_name)
        if config is None:
            # 默认值
            return {
                'left_arm_dof': 7,
                'right_arm_dof': 7,
            }
        
        arm_joints = config.get('hardware', {}).get('arm_joints', {})
        return {
            'left_arm_dof': arm_joints.get('left_arm_dof', 7),
            'right_arm_dof': arm_joints.get('right_arm_dof', 7),
        }
    
    def get_camera_config(self, task_name: str) -> Dict[str, Any]:
        """
        获取相机配置
        
        Args:
            task_name: 任务名称
        
        Returns:
            相机配置字典
        """
        config = self.load_task_config(task_name)
        if config is None:
            # 默认值：单个顶视相机
            return {
                'preset': 'intel_d455_single_top',
                'enable_d455': True,
                'enable_d405_left': False,
                'enable_d405_right': False
            }
        
        camera_preset = config.get('cameras', {}).get('preset', 'intel_d455_single_top')
        
        # 根据 preset 映射到相机启用配置
        camera_config = {
            'preset': camera_preset,
            'enable_d455': False,
            'enable_d405_left': False,
            'enable_d405_right': False
        }
        
        if camera_preset == 'intel_d455_single_top':
            # Config B: 单个D455顶视相机
            camera_config['enable_d455'] = True
        elif camera_preset == 'intel_d455_with_right_wrist':
            # Config A for Piper: D455 + 右侧D405
            camera_config['enable_d455'] = True
            camera_config['enable_d405_right'] = True
        elif camera_preset == 'intel_d455_with_left_wrist':
            # Piper 左手配置：D455 + 左侧 D405
            camera_config['enable_d455'] = True
            camera_config['enable_d405_left'] = True
        elif camera_preset == 'intel_d455_with_dual_wrist':
            # Config A for LinkerHand: D455 + 双D405
            camera_config['enable_d455'] = True
            camera_config['enable_d405_left'] = True
            camera_config['enable_d405_right'] = True
        
        return camera_config
    
    def get_device_info(self, task_name: Optional[str] = None) -> Dict[str, Any]:
        """
        获取设备信息（硬件规格）
        
        Args:
            task_name: 任务名称，如果为 None 则从第一个可用任务读取
        
        Returns:
            设备信息字典
        """
        # 如果没有指定任务名，尝试从任意任务配置读取
        if task_name is None:
            # 查找第一个可用的任务配置
            if TASKS_DIR.exists():
                yaml_files = list(TASKS_DIR.glob('*.yaml'))
                if yaml_files:
                    task_name = yaml_files[0].stem
        
        if task_name:
            config = self.load_task_config(task_name)
            if config and 'device' in config:
                return config['device']
        
        # 默认值
        return {
            'device_number': '123456',
            'device_type': 'gather',
            'hardware_specs': {
                'gpu': 'Unknown',
                'cpu': 'Unknown',
                'memory': 'Unknown',
                'storage': 'Unknown',
            }
        }
    
    def get_hardware_specs(self, task_name: Optional[str] = None) -> Dict[str, str]:
        """
        获取硬件规格信息
        
        Args:
            task_name: 任务名称，如果为 None 则从第一个可用任务读取
        
        Returns:
            硬件规格字典
        """
        device_info = self.get_device_info(task_name)
        return device_info.get('hardware_specs', {
            'gpu': 'Unknown',
            'cpu': 'Unknown',
            'memory': 'Unknown',
            'storage': 'Unknown',
        })

    # ------------------------------------------------------------------
    # New accessors built for YAML-driven presets
    # ------------------------------------------------------------------

    def get_hardware_preset(self, preset_name: str) -> Dict[str, Any]:
        presets = self.load_hardware_presets()
        hardware_presets = presets.get('hardware_presets', {})
        if preset_name not in hardware_presets:
            raise KeyError(f"Hardware preset not found: {preset_name}")
        return deepcopy(hardware_presets[preset_name])

    def get_camera_preset(self, preset_name: str) -> Dict[str, Any]:
        presets = self.load_hardware_presets()
        camera_presets = presets.get('camera_presets', {})
        if preset_name not in camera_presets:
            raise KeyError(f"Camera preset not found: {preset_name}")
        return deepcopy(camera_presets[preset_name])

    def get_video_encoding_preset(self, preset_name: str) -> Dict[str, Any]:
        presets = self.load_hardware_presets()
        video_presets = presets.get('video_encoding_presets', {})
        if preset_name not in video_presets:
            raise KeyError(f"Video encoding preset not found: {preset_name}")
        return deepcopy(video_presets[preset_name])

    def build_runtime_config(self, task_name: str) -> Dict[str, Any]:
        """Assemble a runtime configuration by merging task YAML with presets."""
        task_config = self.load_task_config(task_name)
        # Debug output removed for cleaner logs
        # print(f"[DEBUG——config_loader] task_config = {task_config}")
        # print(f"[DEBUG——config_loader] task_name = {task_name}")
        if task_config is None:
            raise ValueError(f"Task configuration not found: {task_name}")

        runtime_config: Dict[str, Any] = {
            'task_name': task_name,
            'description': task_config.get('description', ''),
            'collection': deepcopy(task_config.get('collection', {})),
            'device': deepcopy(task_config.get('device', {})),
        }

        # Merge hardware preset details
        hardware_section = task_config.get('hardware', {})
        runtime_hardware = deepcopy(hardware_section)
        preset_name = hardware_section.get('preset')
        if preset_name:
            try:
                preset = self.get_hardware_preset(preset_name)
                runtime_hardware['preset'] = preset_name
                runtime_hardware.update({
                    'arms': deepcopy(preset.get('arms', {})),
                    'hands': deepcopy(preset.get('hands', {})),
                    'max_joints_per_side': preset.get('max_joints_per_side'),
                    'total_joints': preset.get('total_joints'),
                })
                # Use default camera preset from hardware preset if not specified in task config
                if 'cameras' not in task_config or 'preset' not in task_config.get('cameras', {}):
                    default_camera_preset = preset.get('default_camera_preset')
                    if default_camera_preset:
                        print(f"[DEBUG——config_loader] Using default camera preset '{default_camera_preset}' from hardware preset '{preset_name}'")
                        runtime_cameras = task_config.get('cameras', {})
                        runtime_cameras['preset'] = default_camera_preset
                        task_config['cameras'] = runtime_cameras
            except KeyError:
                runtime_hardware['preset_missing'] = preset_name

        runtime_hardware.setdefault('active_sides', deepcopy(hardware_section.get('active_sides', {})))
        runtime_hardware.setdefault('arm_joints', deepcopy(hardware_section.get('arm_joints', {})))
        runtime_hardware.setdefault('topics', deepcopy(hardware_section.get('topics', {})))
        runtime_hardware.setdefault('hand_joints', deepcopy(hardware_section.get('hand_joints', {})))
        runtime_config['hardware'] = runtime_hardware

        # Merge camera preset details
        cameras_section = task_config.get('cameras', {})
        runtime_cameras = deepcopy(cameras_section)
        camera_preset_name = cameras_section.get('preset')
        if camera_preset_name:
            try:
                camera_preset = self.get_camera_preset(camera_preset_name)
                runtime_cameras.update({
                    'preset': camera_preset_name,
                    'cameras': camera_preset.get('cameras', []),
                    'type': camera_preset.get('type'),
                    'sdk': camera_preset.get('sdk'),
                })
                # Build topic lookup tables for quick access
                topic_map = {}
                info_map = {}
                stereo_topic_map = {}
                stereo_info_map = {}
                camera_resolutions = {}
                for cam in camera_preset.get('cameras', []):
                    name = cam.get('name')
                    if not name:
                        continue
                    topic_map[name] = cam.get('topic')
                    info_map[name] = cam.get('topic_info')
                    resolution = cam.get('resolution')
                    if isinstance(resolution, list) and len(resolution) == 2:
                        camera_resolutions[name] = (resolution[0], resolution[1])
                    color_format = cam.get('color_format', 'RGB8')
                    if 'color_format_map' not in runtime_cameras:
                        runtime_cameras['color_format_map'] = {}
                    runtime_cameras['color_format_map'][name] = color_format
                    stereo_topics = cam.get('stereo_topics', {}) or {}
                    if stereo_topics:
                        stereo_topic_map[name] = {
                            'left': stereo_topics.get('left'),
                            'right': stereo_topics.get('right'),
                        }
                        stereo_info_map[name] = {
                            'left': stereo_topics.get('left_info'),
                            'right': stereo_topics.get('right_info'),
                        }
                runtime_cameras['topic_map'] = topic_map
                runtime_cameras['info_topic_map'] = info_map
                runtime_cameras['stereo_topic_map'] = stereo_topic_map
                runtime_cameras['stereo_info_topic_map'] = stereo_info_map
                runtime_cameras['camera_resolutions'] = camera_resolutions
            except KeyError:
                runtime_cameras['preset_missing'] = camera_preset_name

        # Attach video encoding preset
        video_settings = runtime_cameras.get('video_settings', {})
        video_quality = video_settings.get('quality', 'medium_quality')
        presets = self.load_hardware_presets().get('video_encoding_presets', {})
        if isinstance(video_quality, str) and video_quality not in presets:
            candidate = f"{video_quality}_quality"
            if candidate in presets:
                video_quality = candidate
        try:
            video_preset = self.get_video_encoding_preset(video_quality)
            runtime_cameras['video_encoding'] = video_preset
        except KeyError:
            runtime_cameras['video_encoding_missing'] = video_quality

        runtime_cameras['stereo_mode'] = cameras_section.get('stereo_mode', False)
        runtime_config['cameras'] = runtime_cameras

        # Flag whether to use compressed video pipeline
        runtime_config['use_video_compression'] = bool(video_settings.get('use_compression', False))

        # Collection defaults
        collection_section = runtime_config.setdefault('collection', {})
        collection_section.setdefault('dataset_dir', f"{DATA_DIR}/{task_name}/all_episodes")
        collection_section.setdefault('num_episodes', 1000)
        collection_section.setdefault('episode_len', 1000)
        collection_section.setdefault('dt', 0.04)

        # Apply camera enable overrides if provided in task YAML
        camera_overrides = runtime_cameras
        enable_top = camera_overrides.get('enable_top_camera')
        enable_left = camera_overrides.get('enable_left_wrist_camera')
        enable_right = camera_overrides.get('enable_right_wrist_camera')
        runtime_cameras['enable_flags'] = {
            'cam_top': enable_top if enable_top is not None else 'cam_top' in runtime_cameras.get('topic_map', {}),
            'cam_left_wrist': enable_left if enable_left is not None else 'cam_left_wrist' in runtime_cameras.get('topic_map', {}),
            'cam_right_wrist': enable_right if enable_right is not None else 'cam_right_wrist' in runtime_cameras.get('topic_map', {}),
        }

        if any(v is not None for v in (enable_top, enable_left, enable_right)):
            def maybe_disable(cam_key: str, enabled_flag):
                if enabled_flag is True or enabled_flag is None:
                    return
                # Remove this camera from all derived lookup tables
                runtime_cameras.get('camera_resolutions', {}).pop(cam_key, None)
                runtime_cameras.get('topic_map', {}).pop(cam_key, None)
                runtime_cameras.get('info_topic_map', {}).pop(cam_key, None)
                runtime_cameras.get('stereo_topic_map', {}).pop(cam_key, None)
                runtime_cameras.get('stereo_info_topic_map', {}).pop(cam_key, None)
                runtime_cameras.get('color_format_map', {}).pop(cam_key, None)
                runtime_cameras.get('cameras', [])[:] = [c for c in runtime_cameras.get('cameras', []) if c.get('name') != cam_key]

            maybe_disable('cam_top', enable_top)
            maybe_disable('cam_left_wrist', enable_left)
            maybe_disable('cam_right_wrist', enable_right)

        return runtime_config

    def get_camera_enable_overrides(self, task_name: str) -> Dict[str, Optional[bool]]:
        """Return per-camera enable overrides for the task."""
        config = self.load_task_config(task_name)
        if not config:
            return {}
        cameras = config.get('cameras', {})
        return {
            'enable_top_camera': cameras.get('enable_top_camera'),
            'enable_left_wrist_camera': cameras.get('enable_left_wrist_camera'),
            'enable_right_wrist_camera': cameras.get('enable_right_wrist_camera'),
        }

    def get_required_camera_counts(self, task_name: str) -> Dict[str, int]:
        """Return expected number of D455 and D405 cameras for validation."""
        overrides = self.get_camera_enable_overrides(task_name)
        config = self.load_task_config(task_name) or {}
        preset_name = config.get('cameras', {}).get('preset', 'intel_d455_single_top')
        preset = self.get_camera_preset(preset_name)

        def count_from_preset(predicate) -> int:
            return sum(1 for cam in preset.get('cameras', []) if predicate(cam))

        required_d455 = count_from_preset(lambda cam: cam.get('position') == 'top')
        required_left = count_from_preset(lambda cam: cam.get('position') == 'left_wrist')
        required_right = count_from_preset(lambda cam: cam.get('position') == 'right_wrist')
        required_d405 = required_left + required_right

        if overrides.get('enable_top_camera') is False:
            required_d455 = 0
        if overrides.get('enable_left_wrist_camera') is False and required_left > 0:
            required_d405 -= 1
            required_left -= 1
        if overrides.get('enable_right_wrist_camera') is False and required_right > 0:
            required_d405 -= 1
            required_right -= 1

        return {
            'd455': max(required_d455, 0),
            'd405': max(required_d405, 0),
        }


# 全局配置加载器实例（单例）
_config_loader = ConfigLoader()

# 导出常用路径（向后兼容）
__all__ = [
    'workspace_root',
    'DATA_DIR',
    'CONFIG_DIR',
    'TASKS_DIR',
    'load_task_config',
    'get_task_config',
    'get_arm_joint_config_from_yaml',
    'get_camera_config_from_yaml',
    'load_hand_config',
    'get_hand_joint_count',
    'has_tactile_support',
    'get_device_info',
    'get_hardware_specs',
    'get_hardware_preset',
    'get_camera_preset',
    'get_video_encoding_preset',
    'build_runtime_config',
    'LEFT_HAND_JOINT',
    'RIGHT_HAND_JOINT',
    'LEFT_HAND_TACTILE_SUPPORT',
    'RIGHT_HAND_TACTILE_SUPPORT',
]

# 向后兼容的函数接口
def load_task_config(task_name: str) -> Optional[Dict[str, Any]]:
    """加载任务配置（新格式）"""
    return _config_loader.load_task_config(task_name)


def get_task_config(task_name: str) -> Optional[Dict[str, Any]]:
    """获取任务配置（兼容旧格式）"""
    return _config_loader.get_task_config_legacy_format(task_name)


def get_arm_joint_config_from_yaml(task_name: str) -> Dict[str, int]:
    """从任务配置获取臂部 DoF"""
    return _config_loader.get_arm_joint_config(task_name)


def get_camera_config_from_yaml(task_name: str) -> Dict[str, Any]:
    """从任务配置获取相机配置"""
    return _config_loader.get_camera_config(task_name)


def load_hand_config() -> Dict[str, Any]:
    """从 LinkerHand SDK 加载手部配置"""
    return _config_loader.load_hand_config()


def get_hand_joint_count(hand_model: str) -> int:
    """获取手部关节数"""
    return _config_loader._get_hand_joint_count(hand_model)


def has_tactile_support(hand_model: str) -> bool:
    """检查手部是否支持触觉传感器"""
    return _config_loader.has_tactile_support(hand_model)


def get_device_info(task_name: Optional[str] = None) -> Dict[str, Any]:
    """获取设备信息"""
    return _config_loader.get_device_info(task_name)


def get_hardware_specs(task_name: Optional[str] = None) -> Dict[str, str]:
    """获取硬件规格信息"""
    return _config_loader.get_hardware_specs(task_name)


def get_hardware_preset(preset_name: str) -> Dict[str, Any]:
    """获取硬件预设配置"""
    return _config_loader.get_hardware_preset(preset_name)


def get_camera_preset(preset_name: str) -> Dict[str, Any]:
    """获取相机预设配置"""
    return _config_loader.get_camera_preset(preset_name)


def get_video_encoding_preset(preset_name: str) -> Dict[str, Any]:
    """获取视频编码预设配置"""
    return _config_loader.get_video_encoding_preset(preset_name)


def build_runtime_config(task_name: str) -> Dict[str, Any]:
    """获取整合后的运行时配置"""
    return _config_loader.build_runtime_config(task_name)


# 加载手部配置（模块加载时自动执行）
_hand_config = load_hand_config()
LEFT_HAND_JOINT = _hand_config['left_hand_joints']
RIGHT_HAND_JOINT = _hand_config['right_hand_joints']
LEFT_HAND_TACTILE_SUPPORT = has_tactile_support(_hand_config['left_hand_model'])
RIGHT_HAND_TACTILE_SUPPORT = has_tactile_support(_hand_config['right_hand_model'])

