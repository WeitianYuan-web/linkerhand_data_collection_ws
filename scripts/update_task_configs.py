#!/usr/bin/env python3
"""Synchronise task YAMLs with task_config.json settings.

This utility updates the arm/hand configuration in:
  - hardware_presets.yaml
  - linkerhand_piper_grasp.yaml
  - double_linkerhand_grasp.yaml

Based on the workspace-level task_config.json file.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

try:
    from ruamel.yaml import YAML
except ImportError:  # pragma: no cover - dependency check
    print("❌ Missing dependency: ruamel.yaml. Install with `pip install ruamel.yaml`.", file=sys.stderr)
    sys.exit(1)


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = WORKSPACE_ROOT / "src" / "linkerhand_data_collection_srv" / "configs"
TASKS_DIR = CONFIG_DIR / "tasks"

TASK_CONFIG_JSON = WORKSPACE_ROOT / "task_config.json"
HARDWARE_PRESETS_YAML = CONFIG_DIR / "hardware_presets.yaml"
PIPER_TASK_YAML = TASKS_DIR / "linkerhand_piper_grasp.yaml"
DOUBLE_TASK_YAML = TASKS_DIR / "double_linkerhand_grasp.yaml"


@dataclass
class TaskSettings:
    arm_type: str
    hand_side: str
    hand_model: str
    hand_dof: int
    collect_tactile: bool


def load_task_settings() -> TaskSettings:
    if not TASK_CONFIG_JSON.exists():
        raise FileNotFoundError(f"Task config not found: {TASK_CONFIG_JSON}")

    with TASK_CONFIG_JSON.open("r", encoding="utf-8") as f:
        cfg = json.load(f)

    arm_type = cfg.get("armType", "").strip().lower()
    hand_side = cfg.get("handSide", "").strip().lower()
    hand_model = cfg.get("handModel", "").strip().upper()
    collect_tactile = bool(cfg.get("collectTactile", False))

    if arm_type not in {"piper", "linker"}:
        raise ValueError("armType must be 'piper' or 'linker'.")

    if arm_type == "piper" and hand_side not in {"left", "right"}:
        raise ValueError("Piper arm tasks must specify handSide as 'left' or 'right'.")

    if arm_type == "linker" and hand_side != "both":
        raise ValueError("Linker arm tasks require handSide to be 'both'.")

    hand_dof = parse_hand_dof(hand_model)

    return TaskSettings(
        arm_type=arm_type,
        hand_side=hand_side,
        hand_model=hand_model,
        hand_dof=hand_dof,
        collect_tactile=collect_tactile,
    )


def parse_hand_dof(hand_model: str) -> int:
    match = re.search(r"(\d+)", hand_model)
    if not match:
        raise ValueError(f"Unable to infer DoF from hand model: {hand_model}")
    return int(match.group(1))


def load_yaml(path: Path) -> Any:
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.indent(mapping=2, sequence=4, offset=2)
    with path.open("r", encoding="utf-8") as f:
        return yaml.load(f)


def dump_yaml(data: Any, path: Path) -> None:
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.indent(mapping=2, sequence=4, offset=2)
    with path.open("w", encoding="utf-8") as f:
        yaml.dump(data, f)


def update_hardware_presets(data: Dict[str, Any], settings: TaskSettings) -> None:
    presets = data.get("hardware_presets", {})

    piper_preset = presets.get("piper_linkerhand_single")
    if piper_preset:
        hands = piper_preset.setdefault("hands", {})
        hands["type"] = settings.hand_model
        hands["dof_per_hand"] = settings.hand_dof
        hands["tactile_support"] = settings.collect_tactile

        arms = piper_preset.setdefault("arms", {})
        arms["dof_per_arm"] = 6
        arms["type"] = "piper"

        per_side = 6 + settings.hand_dof
        piper_preset["max_joints_per_side"] = per_side
        piper_preset["total_joints"] = per_side

    linker_preset = presets.get("linkerarm_linkerhand_dual")
    if linker_preset:
        hands = linker_preset.setdefault("hands", {})
        hands["type"] = settings.hand_model
        hands["dof_per_hand"] = settings.hand_dof
        hands["tactile_support"] = settings.collect_tactile

        arms = linker_preset.setdefault("arms", {})
        arms["dof_per_arm"] = 7
        arms["type"] = "linker"

        per_side = 7 + settings.hand_dof
        linker_preset["max_joints_per_side"] = per_side
        linker_preset["total_joints"] = per_side * 2


def update_piper_task(data: Dict[str, Any], settings: TaskSettings) -> None:
    hardware = data.setdefault("hardware", {})
    active = hardware.setdefault("active_sides", {})

    left_active = settings.hand_side in {"left", "both"}
    right_active = settings.hand_side in {"right", "both"}

    active["left_arm"] = left_active
    active["left_hand"] = left_active
    active["right_arm"] = right_active
    active["right_hand"] = right_active

    arm_joints = hardware.setdefault("arm_joints", {})
    arm_joints["left_arm_dof"] = 6 if left_active else 0
    arm_joints["right_arm_dof"] = 6 if right_active else 0

    hand_joints = hardware.setdefault("hand_joints", {})
    hand_joints["left_hand_dof"] = settings.hand_dof if left_active else 0
    hand_joints["right_hand_dof"] = settings.hand_dof if right_active else 0

    hardware["collect_tactile"] = settings.collect_tactile

    # Set time synchronization for Piper task (uses system time because Piper SDK)
    collection = data.setdefault("collection", {})
    collection["use_system_time"] = True  # Piper SDK uses system time, not ROS2 time

    update_device_specs(data)


def update_double_task(data: Dict[str, Any], settings: TaskSettings) -> None:
    hardware = data.setdefault("hardware", {})
    active = hardware.setdefault("active_sides", {})

    left_active = settings.hand_side in {"left", "both"}
    right_active = settings.hand_side in {"right", "both"}

    active["left_arm"] = left_active
    active["left_hand"] = left_active
    active["right_arm"] = right_active
    active["right_hand"] = right_active

    arm_joints = hardware.setdefault("arm_joints", {})
    arm_joints["left_arm_dof"] = 7 if left_active else 0
    arm_joints["right_arm_dof"] = 7 if right_active else 0

    hand_joints = hardware.setdefault("hand_joints", {})
    hand_joints["left_hand_dof"] = settings.hand_dof if left_active else 0
    hand_joints["right_hand_dof"] = settings.hand_dof if right_active else 0

    hardware["collect_tactile"] = settings.collect_tactile

    # Set time synchronization for Double LinkerHand task (uses ROS2 time for all sensors)
    collection = data.setdefault("collection", {})
    collection["use_system_time"] = False  # All sensors use ROS2 topics with ROS2 time

    update_device_specs(data)


def update_device_specs(data: Dict[str, Any]) -> None:
    device = data.setdefault("device", {})
    hardware_specs = device.setdefault("hardware_specs", {})
    detected = {
        "gpu": detect_gpu_spec(),
        "cpu": detect_cpu_spec(),
        "memory": detect_memory_spec(),
        "storage": detect_storage_spec(),
    }
    for key, value in detected.items():
        if value:
            hardware_specs[key] = value


def detect_gpu_spec() -> Optional[str]:
    try:
        result = run_command([
            "nvidia-smi",
            "--query-gpu=name",
            "--format=csv,noheader",
        ])
    except FileNotFoundError:
        return None

    if not result:
        return None

    names = [line.strip() for line in result.splitlines() if line.strip()]
    if not names:
        return None

    counts: Dict[str, int] = {}
    for name in names:
        counts[name] = counts.get(name, 0) + 1

    parts = [f"{name} X {count}" for name, count in counts.items()]
    return " + ".join(parts)


def detect_cpu_spec() -> Optional[str]:
    try:
        output = run_command(["lscpu"])
    except FileNotFoundError:
        return None

    if not output:
        return None

    model = None
    sockets = 1
    for line in output.splitlines():
        if "Model name:" in line:
            model = line.split(":", 1)[1].strip()
        elif "Socket(s):" in line:
            try:
                sockets = int(line.split(":", 1)[1].strip())
            except ValueError:
                sockets = 1

    if not model:
        return None

    return f"{model} X {sockets}"


def detect_memory_spec() -> Optional[str]:
    try:
        with open("/proc/meminfo", "r", encoding="utf-8") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    parts = line.split()
                    if len(parts) >= 2:
                        try:
                            mem_kib = int(parts[1])
                            mem_gib = mem_kib / 1024 / 1024
                            mem_rounded = int(round(mem_gib))
                            return f"{mem_rounded}G"
                        except ValueError:
                            return None
    except FileNotFoundError:
        return None

    return None


def detect_storage_spec() -> Optional[str]:
    try:
        output = run_command(["lsblk", "-b", "-d", "-o", "NAME,SIZE"])
    except FileNotFoundError:
        return None

    if not output:
        return None

    total_bytes = 0
    for line in output.splitlines():
        line = line.strip()
        if not line or line.startswith("NAME"):
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        try:
            total_bytes += int(parts[1])
        except ValueError:
            continue

    if total_bytes <= 0:
        return None

    teb = total_bytes / (1024 ** 4)
    if teb >= 1:
        return f"{teb:.1f}TB"
    gib = total_bytes / (1024 ** 3)
    return f"{gib:.1f}GB"


def run_command(cmd: Any) -> str:
    completed = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
        text=True,
    )
    if completed.returncode != 0:
        return ""
    return completed.stdout


def main() -> None:
    settings = load_task_settings()

    hardware_presets = load_yaml(HARDWARE_PRESETS_YAML)
    update_hardware_presets(hardware_presets, settings)
    dump_yaml(hardware_presets, HARDWARE_PRESETS_YAML)

    piper_task = load_yaml(PIPER_TASK_YAML)
    update_piper_task(piper_task, settings)
    dump_yaml(piper_task, PIPER_TASK_YAML)

    double_task = load_yaml(DOUBLE_TASK_YAML)
    update_double_task(double_task, settings)
    dump_yaml(double_task, DOUBLE_TASK_YAML)

    # Determine which task is being used and its time source
    if settings.arm_type == "piper":
        active_task = "linkerhand_piper_grasp"
        time_source = "System Time"
    else:
        active_task = "double_linkerhand_grasp"
        time_source = "ROS2 Time"
    
    summary = [
        f"armType={settings.arm_type}",
        f"handSide={settings.hand_side}",
        f"handModel={settings.hand_model}",
        f"handDoF={settings.hand_dof}",
        f"collectTactile={settings.collect_tactile}",
    ]
    print("✅ Updated task configurations (" + ", ".join(summary) + ")")
    print(f"📋 Active task: {active_task}")
    print(f"🕒 Time source: {time_source} ({'use_system_time=true' if settings.arm_type == 'piper' else 'use_system_time=false'})")


if __name__ == "__main__":
    main()

