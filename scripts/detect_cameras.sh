#!/usr/bin/env python3
"""
RealSense 摄像头自动检测脚本

功能：
1. 调用 `rs-enumerate-devices` 自动检测连接的 D455 / D405 摄像头。
2. 校验探测到的数量是否与 `task_config.json` 描述一致，不一致则报错退出。
3. 根据任务（piper -> linkerhand_piper_grasp，linker -> double_linkerhand_grasp）的需求，
   分配摄像头到 top / left_wrist / right_wrist，并写入 `camera_serial_numbers.yaml`。
4. 自动更新任务 YAML 中的摄像头启用标志，使采集程序能够自适应订阅对应的话题。
5. 完成后即可通过 `ros2 launch linkerhand_data_collection_srv multi_camera_launch.py`
   启动摄像头，无需额外参数。
"""

from __future__ import annotations

import re
import subprocess
import sys
import json
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Dict, Tuple

import yaml

ROOT_DIR = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT_DIR / 'src' / 'linkerhand_data_collection_srv' / 'configs'
CAMERA_CONFIG_PATH = CONFIG_DIR / 'camera_serial_numbers.yaml'
PIPER_TASK_PATH = CONFIG_DIR / 'tasks' / 'linkerhand_piper_grasp.yaml'
DOUBLE_TASK_PATH = CONFIG_DIR / 'tasks' / 'double_linkerhand_grasp.yaml'
TASK_CONFIG_PATH = ROOT_DIR / 'task_config.json'


@dataclass
class CameraDevice:
    device_type: str  # 'd455' or 'd405'
    serial_number: str
    usb_port: str = ''


def run_rs_enumerate() -> str:
    try:
        output = subprocess.check_output(['rs-enumerate-devices'], text=True)
        return output
    except FileNotFoundError as exc:
        raise SystemExit('❌ 未找到 rs-enumerate-devices，请先安装 Intel RealSense SDK') from exc
    except subprocess.CalledProcessError as exc:
        raise SystemExit(f'❌ 调用 rs-enumerate-devices 失败: {exc}') from exc


def parse_devices(text: str) -> List[CameraDevice]:
    devices: List[CameraDevice] = []
    current: Dict[str, str] = {}

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            if current.get('device_type') and current.get('serial_number'):
                devices.append(CameraDevice(**current))
            current = {}
            continue

        name_match = re.search(r'Name\s*:\s*Intel.*RealSense.*(D\d+)', line)
        if name_match:
            model = name_match.group(1).lower()
            if current.get('device_type') and current.get('serial_number'):
                devices.append(CameraDevice(**current))
            current = {
                'device_type': 'd455' if model == 'd455' else 'd405',
                'serial_number': '',
                'usb_port': ''
            }
            continue

        if 'Serial Number' in line and 'Asic' not in line:
            serial_match = re.search(r'(\d{12})', line)
            if serial_match and current:
                current['serial_number'] = serial_match.group(1)
            continue

        if 'Physical Port' in line:
            port_match = re.search(r'(\d+-\d+(?:\.\d+)*)', line)
            if port_match and current:
                current['usb_port'] = port_match.group(1)
            continue

    if current.get('device_type') and current.get('serial_number'):
        devices.append(CameraDevice(**current))

    return devices


def load_task_config() -> Tuple[str, str, int, int]:
    if not TASK_CONFIG_PATH.exists():
        raise SystemExit('❌ 未找到 task_config.json，无法确定任务期望的摄像头数量')

    data = json.loads(TASK_CONFIG_PATH.read_text(encoding='utf-8'))
    arm_type = data.get('armType', '').strip().lower()
    hand_side = data.get('handSide', '').strip().lower()
    d455_count = int(data.get('cameraD455Count', 0))
    d405_count = int(data.get('cameraD405Count', 0))

    if arm_type not in {'piper', 'linker'}:
        raise SystemExit(f"❌ task_config.json 中的 armType 不支持: {arm_type}")

    return arm_type, hand_side, d455_count, d405_count


def validate_counts(d455_devices: List[CameraDevice], d405_devices: List[CameraDevice],
                    expected_d455: int, expected_d405: int) -> None:
    if len(d455_devices) != expected_d455:
        raise SystemExit(f"❌ D455 数量不匹配: 期望 {expected_d455}，实际 {len(d455_devices)}")
    if len(d405_devices) != expected_d405:
        raise SystemExit(f"❌ D405 数量不匹配: 期望 {expected_d405}，实际 {len(d405_devices)}")


def build_assignment(arm_type: str, hand_side: str,
                     d455_devices: List[CameraDevice],
                     d405_devices: List[CameraDevice]) -> Dict[str, Dict[str, object]]:
    now_str = datetime.now().strftime('%Y-%m-%d')

    assignment = {
        'd455_top': {
            'enabled': False,
            'serial_number': '',
            'device_type': 'd455',
            'position': 'top',
            'namespace': 'camera',
            'camera_name': 'd455_camera',
            'usb_port': '',
            'notes': f'Detected on {now_str}'
        },
        'd405_left_wrist': {
            'enabled': False,
            'serial_number': '',
            'device_type': 'd405',
            'position': 'left_wrist',
            'namespace': 'camera_left_wrist',
            'camera_name': 'd405_left_camera',
            'usb_port': '',
            'notes': f'Detected on {now_str}'
        },
        'd405_right_wrist': {
            'enabled': False,
            'serial_number': '',
            'device_type': 'd405',
            'position': 'right_wrist',
            'namespace': 'camera_right_wrist',
            'camera_name': 'd405_right_camera',
            'usb_port': '',
            'notes': f'Detected on {now_str}'
        }
    }

    if d455_devices:
        assignment['d455_top'].update({
            'enabled': True,
            'serial_number': d455_devices[0].serial_number,
            'usb_port': d455_devices[0].usb_port
        })

    if arm_type == 'piper':
        if len(d405_devices) == 1:
            if hand_side not in {'left', 'right'}:
                raise SystemExit('❌ piper 任务仅支持 handSide 为 left 或 right，当存在 D405 时必须指定')
            target = 'd405_left_wrist' if hand_side == 'left' else 'd405_right_wrist'
            assignment[target].update({
                'enabled': True,
                'serial_number': d405_devices[0].serial_number,
                'usb_port': d405_devices[0].usb_port
            })
        elif len(d405_devices) > 1:
            raise SystemExit('❌ piper 任务最多只支持一个 D405 摄像头')
    else:  # linker / double_linker_hand
        if len(d405_devices) == 0:
            pass
        elif len(d405_devices) == 2:
            left, right = d405_devices
            assignment['d405_left_wrist'].update({
                'enabled': True,
                'serial_number': left.serial_number,
                'usb_port': left.usb_port
            })
            assignment['d405_right_wrist'].update({
                'enabled': True,
                'serial_number': right.serial_number,
                'usb_port': right.usb_port
            })
        else:
            raise SystemExit('❌ linker 任务仅支持 0 或 2 个 D405 摄像头')

    return assignment


def write_camera_config(assignment: Dict[str, Dict[str, object]],
                        d455_count: int, d405_count: int) -> None:
    CAMERA_CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    lines = [
        "# RealSense Camera Serial Number Configuration",
        f"# 自动生成时间: {now}",
        "#",
        "# 检测到的相机：",
        f"#   D455: {d455_count} 个",
        f"#   D405: {d405_count} 个",
        "#",
        "# 如何更新：",
        "#   重新运行 scripts/detect_cameras.sh",
        "",
        "camera_serial_numbers:"
    ]

    for key in ['d455_top', 'd405_left_wrist', 'd405_right_wrist']:
        cam = assignment[key]
        lines.append(f"  {key}:")
        lines.append(f"    enabled: {'true' if cam['enabled'] else 'false'}")
        lines.append(f"    serial_number: \"{cam['serial_number']}\"")
        lines.append(f"    device_type: \"{cam['device_type']}\"")
        lines.append(f"    position: \"{cam['position']}\"")
        lines.append(f"    namespace: \"{cam['namespace']}\"")
        lines.append(f"    camera_name: \"{cam['camera_name']}\"")
        lines.append(f"    usb_port: \"{cam['usb_port']}\"")
        lines.append("    auto_detect: false")
        lines.append(f"    notes: \"{cam['notes']}\"")
    lines.append("")
    lines.extend([
        "# 全局设置",
        "global_settings:",
        "  fallback_to_auto_detect: true",
        "  warn_on_mismatch: true",
        "  unified_resolution:",
        "    width: 640",
        "    height: 480",
        "    fps: 30",
        "    format: \"RGB8\"",
        ""
    ])

    CAMERA_CONFIG_PATH.write_text('\n'.join(lines), encoding='utf-8')


def determine_camera_preset(enable_left: bool, enable_right: bool, arm_type: str) -> str:
    """根据启用的相机确定正确的 preset"""
    if arm_type == 'piper':
        if enable_left:
            return 'intel_d455_with_left_wrist'
        elif enable_right:
            return 'intel_d455_with_right_wrist'
        else:
            return 'intel_d455_single_top'
    else:  # linker
        if enable_left and enable_right:
            return 'intel_d455_with_dual_wrist'
        else:
            return 'intel_d455_single_top'


def update_task_yaml(path: Path, enable_left: bool, enable_right: bool, enable_top: bool = True, arm_type: str = 'linker') -> None:
    text = path.read_text(encoding='utf-8')

    def replace_flag(flag: str, value: bool, body: str) -> str:
        pattern = rf'({flag}:\s*)(true|false)'
        replacement = rf"\1{'true' if value else 'false'}"
        if not re.search(pattern, body):
            raise SystemExit(f'❌ 未在 {path} 中找到 {flag}')
        return re.sub(pattern, replacement, body)
    
    def replace_camera_preset(preset_value: str, body: str) -> str:
        # 只替换 cameras: 块中的 preset，不要影响 hardware: 块
        # 使用更精确的匹配：在 cameras: 之后的第一个 preset
        pattern = r'(cameras:.*?preset:\s*["\']?)([a-z0-9_]+)(["\']?)'
        replacement = rf'\1{preset_value}\3'
        if not re.search(pattern, body, re.DOTALL):
            raise SystemExit(f'❌ 未在 {path} 的 cameras 块中找到 preset 字段')
        return re.sub(pattern, replacement, body, count=1, flags=re.DOTALL)

    text = replace_flag('enable_top_camera', enable_top, text)
    text = replace_flag('enable_left_wrist_camera', enable_left, text)
    text = replace_flag('enable_right_wrist_camera', enable_right, text)
    
    # 更新 cameras 块中的 preset 字段
    preset = determine_camera_preset(enable_left, enable_right, arm_type)
    text = replace_camera_preset(preset, text)

    path.write_text(text, encoding='utf-8')


def main() -> None:
    rs_output = run_rs_enumerate()
    devices = parse_devices(rs_output)

    d455_devices = [d for d in devices if d.device_type == 'd455']
    d405_devices = [d for d in devices if d.device_type == 'd405']

    if not devices:
        raise SystemExit('❌ 未检测到任何 RealSense 摄像头，请检查连接与权限')

    print('=' * 40)
    print('检测到的摄像头:')
    for device in devices:
        print(f"- {device.device_type.upper()} 序列号: {device.serial_number} USB端口: {device.usb_port}")
    print('=' * 40)
    print('')

    arm_type, hand_side, expected_d455, expected_d405 = load_task_config()
    validate_counts(d455_devices, d405_devices, expected_d455, expected_d405)

    assignment = build_assignment(arm_type, hand_side, d455_devices, d405_devices)

    write_camera_config(assignment, len(d455_devices), len(d405_devices))

    if arm_type == 'piper':
        enable_left = assignment['d405_left_wrist']['enabled']
        enable_right = assignment['d405_right_wrist']['enabled']
        update_task_yaml(PIPER_TASK_PATH, enable_left, enable_right, enable_top=True, arm_type='piper')
        update_task_yaml(DOUBLE_TASK_PATH,
                         assignment['d405_left_wrist']['enabled'],
                         assignment['d405_right_wrist']['enabled'],
                         enable_top=assignment['d455_top']['enabled'],
                         arm_type='linker')
    else:
        enable_left = assignment['d405_left_wrist']['enabled']
        enable_right = assignment['d405_right_wrist']['enabled']
        update_task_yaml(DOUBLE_TASK_PATH, enable_left, enable_right, enable_top=True, arm_type='linker')
        # 仍然让 piper 任务保持顶视相机可用
        update_task_yaml(PIPER_TASK_PATH,
                         False,
                         False,
                         enable_top=True,
                         arm_type='piper')

    # 显示当前任务的相机配置
    if arm_type == 'linker':
        current_preset = determine_camera_preset(enable_left, enable_right, 'linker')
        print('')
        print(f'📸 当前任务相机配置:')
        print(f'  - Preset: {current_preset}')
        print(f'  - D455 顶部相机: {"✓ 启用" if assignment["d455_top"]["enabled"] else "✗ 禁用"}')
        print(f'  - D405 左腕相机: {"✓ 启用" if enable_left else "✗ 禁用"}')
        print(f'  - D405 右腕相机: {"✓ 启用" if enable_right else "✗ 禁用"}')
    
    print('')
    print('✅ 摄像头配置已更新：')
    print(f'  - 写入 {CAMERA_CONFIG_PATH.name}')
    print(f'  - 更新 {PIPER_TASK_PATH.name} (preset + 启用标志)')
    print(f'  - 更新 {DOUBLE_TASK_PATH.name} (preset + 启用标志)')
    print('')
    print('现在可以运行以下命令启动摄像头：')
    print('  ros2 launch linkerhand_data_collection_srv multi_camera_launch.py')


if __name__ == '__main__':
    try:
        main()
    except SystemExit as exc:
        print(exc, file=sys.stderr)
        sys.exit(1 if exc.code is None else exc.code)
    except Exception as exc:  # 最外层兜底
        print(f'❌ 脚本执行失败: {exc}', file=sys.stderr)
        sys.exit(1)

