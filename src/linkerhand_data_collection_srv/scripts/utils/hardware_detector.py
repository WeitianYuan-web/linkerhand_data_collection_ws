#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
硬件信息检测器 - 自动检测系统硬件信息
"""

import math
import subprocess
from collections import Counter
from typing import Dict


def run_command(cmd):
    """运行命令并返回输出"""
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.DEVNULL)
    except (FileNotFoundError, subprocess.CalledProcessError):
        return ""


def get_gpu_info() -> str:
    """获取 GPU 信息"""
    # 尝试使用 nvidia-smi
    smi_output = run_command([
        "nvidia-smi",
        "--query-gpu=name",
        "--format=csv,noheader",
    ])
    if smi_output.strip():
        names = [line.strip() for line in smi_output.splitlines() if line.strip()]
        if names:
            counts = Counter(names)
            parts = [f"{name} X {count}" for name, count in counts.items()]
            return ", ".join(parts)

    # Fallback: 使用 lspci
    lspci_output = run_command(["lspci"])
    if lspci_output.strip():
        names = []
        for line in lspci_output.splitlines():
            lower = line.lower()
            if "vga compatible controller" in lower or "3d controller" in lower:
                names.append(line.split(":", 2)[-1].strip())
        if names:
            return ", ".join(names)

    return "Unknown"


def get_cpu_info() -> str:
    """获取 CPU 信息"""
    lscpu_output = run_command(["lscpu"])
    model = None
    sockets = None
    
    if lscpu_output.strip():
        for line in lscpu_output.splitlines():
            if line.startswith("Model name:"):
                model = line.split(":", 1)[1].strip()
            elif line.startswith("Socket(s):"):
                value = line.split(":", 1)[1].strip()
                if value.isdigit():
                    sockets = int(value)

    # Fallback: 读取 /proc/cpuinfo
    if model is None:
        try:
            with open("/proc/cpuinfo", "r", encoding="utf-8", errors="ignore") as handle:
                for line in handle:
                    if line.lower().startswith("model name"):
                        model = line.split(":", 1)[1].strip()
                        break
        except FileNotFoundError:
            model = None

    if sockets is None:
        sockets = 1

    return f"{model or 'Unknown'} X {sockets}"


def get_memory_info() -> str:
    """获取内存信息"""
    try:
        with open("/proc/meminfo", "r", encoding="utf-8") as handle:
            for line in handle:
                if line.startswith("MemTotal:"):
                    kib = int(line.split()[1])
                    gib = kib / (1024 ** 2)
                    rounded = int(math.floor(gib + 0.5))
                    return f"{rounded}G"
    except FileNotFoundError:
        pass
    return "Unknown"


def get_storage_info() -> str:
    """获取存储信息"""
    # 尝试使用 lsblk
    lsblk_output = run_command(["lsblk", "-dn", "-o", "SIZE,TYPE"])
    if lsblk_output.strip():
        total_tb = 0.0
        for line in lsblk_output.splitlines():
            parts = line.split()
            if len(parts) != 2:
                continue
            size, blk_type = parts
            if blk_type != "disk":
                continue
            if not size:
                continue
            unit = size[-1].upper()
            try:
                value = float(size[:-1])
            except ValueError:
                continue
            if unit == 'T':
                total_tb += value
            elif unit == 'G':
                total_tb += value / 1024
            elif unit == 'M':
                total_tb += value / (1024 ** 2)
            elif unit == 'K':
                total_tb += value / (1024 ** 3)
            else:
                # assume bytes
                total_tb += value / (1024 ** 4)
        if total_tb > 0:
            return f"{total_tb:.1f}TB"

    # Fallback: 使用 df
    df_output = run_command(["df", "-h", "--total"])
    if df_output.strip():
        for line in df_output.splitlines():
            if line.startswith("total"):
                parts = line.split()
                if len(parts) >= 2:
                    return parts[1]

    return "Unknown"


def detect_hardware() -> Dict[str, str]:
    """
    检测所有硬件信息
    
    Returns:
        硬件信息字典
    """
    return {
        'gpu': get_gpu_info(),
        'cpu': get_cpu_info(),
        'memory': get_memory_info(),
        'storage': get_storage_info(),
    }


def main():
    """命令行入口 - 显示检测到的硬件信息"""
    hardware = detect_hardware()
    print("检测到的硬件信息:")
    print(f"  GPU: {hardware['gpu']}")
    print(f"  CPU: {hardware['cpu']}")
    print(f"  Memory: {hardware['memory']}")
    print(f"  Storage: {hardware['storage']}")


if __name__ == '__main__':
    main()

