#!/usr/bin/env python3
"""
Episode Data Extraction Script
解压 episode 数据的脚本 - 将 NPZ/NPY 文件转换为 CSV 表格格式

功能：
- 将 telemetry.npz 解压为多个 CSV 文件
- 将相机时间戳 .npy 文件转换为 CSV
- 所有 CSV 文件可以用 Excel 或其他表格软件打开

Usage:
    python3 extract_episode_data.py <episode_path> [--output_dir <output_dir>]
    
Example:
    python3 extract_episode_data.py collection_data/linkerhand_piper_grasp/session_2025-10-10_20-45-22/episode_000002
    
输出：
- telemetry_csv/: 包含所有遥测数据的 CSV 文件
- cameras_csv/: 包含相机时间戳的 CSV 文件
"""

import argparse
import json
import numpy as np
import os
import pandas as pd
from pathlib import Path


def extract_npz(npz_path, output_dir):
    """解压 .npz 文件并保存为 CSV 格式"""
    print(f"\n{'='*60}")
    print(f"解压 NPZ 文件: {npz_path}")
    print(f"{'='*60}")
    
    data = np.load(npz_path, allow_pickle=True)
    
    # 显示包含的数组
    print(f"\n包含 {len(data.files)} 个数组:")
    for key in data.files:
        arr = data[key]
        print(f"  - {key}: shape={arr.shape}, dtype={arr.dtype}")
    
    # 保存每个数组为 CSV
    csv_output_dir = output_dir / "telemetry_csv"
    csv_output_dir.mkdir(exist_ok=True)
    
    summary = {
        "source": str(npz_path),
        "arrays": {}
    }
    
    for key in data.files:
        arr = data[key]
        csv_path = csv_output_dir / f"{key}.csv"
        
        try:
            if arr.ndim == 1:
                # 1D 数组：保存为单列 CSV
                df = pd.DataFrame({key: arr})
                df.to_csv(csv_path, index=True, index_label='frame')
                print(f"    ✓ {key}.csv (1D: {arr.shape[0]} 帧)")
                
            elif arr.ndim == 2:
                # 2D 数组：保存为多列 CSV
                columns = [f"{key}_{i}" for i in range(arr.shape[1])]
                df = pd.DataFrame(arr, columns=columns)
                df.to_csv(csv_path, index=True, index_label='frame')
                print(f"    ✓ {key}.csv (2D: {arr.shape[0]} 帧 × {arr.shape[1]} 维)")
                
            elif arr.ndim == 3:
                # 3D 数组：展平成 2D 保存
                # 例如 (413, 12, 6) -> (413, 72)
                reshaped = arr.reshape(arr.shape[0], -1)
                columns = [f"{key}_{i}_{j}" for i in range(arr.shape[1]) for j in range(arr.shape[2])]
                df = pd.DataFrame(reshaped, columns=columns)
                df.to_csv(csv_path, index=True, index_label='frame')
                print(f"    ✓ {key}.csv (3D展平: {arr.shape[0]} 帧 × {reshaped.shape[1]} 维, 原始 {arr.shape})")
                
            else:
                # 更高维度的数组，完全展平
                reshaped = arr.reshape(arr.shape[0], -1)
                columns = [f"{key}_{i}" for i in range(reshaped.shape[1])]
                df = pd.DataFrame(reshaped, columns=columns)
                df.to_csv(csv_path, index=True, index_label='frame')
                print(f"    ✓ {key}.csv ({arr.ndim}D展平: {arr.shape[0]} 帧 × {reshaped.shape[1]} 维)")
            
            # 收集统计信息
            if np.issubdtype(arr.dtype, np.number):
                summary["arrays"][key] = {
                    "shape": list(arr.shape),
                    "dtype": str(arr.dtype),
                    "min": float(arr.min()),
                    "max": float(arr.max()),
                    "mean": float(arr.mean()),
                    "csv_file": str(csv_path.name)
                }
            else:
                summary["arrays"][key] = {
                    "shape": list(arr.shape),
                    "dtype": str(arr.dtype),
                    "csv_file": str(csv_path.name)
                }
                
        except Exception as e:
            print(f"    ✗ {key}.csv - 错误: {e}")
            summary["arrays"][key] = {
                "shape": list(arr.shape),
                "dtype": str(arr.dtype),
                "error": str(e)
            }
    
    # 保存摘要信息
    summary_path = csv_output_dir / "summary.json"
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"\n    摘要信息已保存: {summary_path}")
    
    return data


def extract_npy(npy_path, output_dir):
    """解压 .npy 文件并保存为 CSV"""
    print(f"\n{'='*60}")
    print(f"加载 NPY 文件: {npy_path}")
    print(f"{'='*60}")
    
    data = np.load(npy_path, allow_pickle=True)
    print(f"\n  shape: {data.shape}")
    print(f"  dtype: {data.dtype}")
    
    if np.issubdtype(data.dtype, np.number):
        print(f"  min: {data.min()}")
        print(f"  max: {data.max()}")
        print(f"  mean: {data.mean()}")
    
    # 显示前几个值
    print(f"\n  前10个值:")
    print(f"  {data.flatten()[:10]}")
    
    # 保存为 CSV 格式
    rel_path = npy_path.relative_to(npy_path.parents[2])  # 相对于 episode 根目录
    csv_path = output_dir / "cameras_csv" / f"{rel_path.stem}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        if data.ndim == 1:
            df = pd.DataFrame({rel_path.stem: data})
            df.to_csv(csv_path, index=True, index_label='index')
            print(f"\n  ✓ 已保存为 CSV: {csv_path}")
        elif data.ndim == 2:
            columns = [f"{rel_path.stem}_{i}" for i in range(data.shape[1])]
            df = pd.DataFrame(data, columns=columns)
            df.to_csv(csv_path, index=True, index_label='index')
            print(f"\n  ✓ 已保存为 CSV: {csv_path}")
        else:
            # 高维数组展平
            reshaped = data.reshape(data.shape[0], -1)
            columns = [f"{rel_path.stem}_{i}" for i in range(reshaped.shape[1])]
            df = pd.DataFrame(reshaped, columns=columns)
            df.to_csv(csv_path, index=True, index_label='index')
            print(f"\n  ✓ 已保存为 CSV (展平): {csv_path}")
    except Exception as e:
        print(f"\n  ✗ CSV 保存失败: {e}")
    
    return data


def display_json(json_path):
    """显示 JSON 文件内容"""
    print(f"\n{'='*60}")
    print(f"JSON 文件: {json_path}")
    print(f"{'='*60}")
    
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    print(json.dumps(data, indent=2, ensure_ascii=False))


def extract_episode(episode_path, output_dir=None):
    """解压整个 episode 的数据"""
    episode_path = Path(episode_path).resolve()
    
    if not episode_path.exists():
        raise FileNotFoundError(f"Episode 路径不存在: {episode_path}")
    
    print(f"\n{'#'*60}")
    print(f"# 解压 Episode: {episode_path.name}")
    print(f"# 路径: {episode_path}")
    print(f"{'#'*60}")
    
    # 设置输出目录
    if output_dir is None:
        output_dir = episode_path / "extracted"
    else:
        output_dir = Path(output_dir)
    
    output_dir.mkdir(exist_ok=True)
    print(f"\n输出目录: {output_dir}")
    
    # 查找并处理所有文件
    files = {
        'npz': list(episode_path.glob("**/*.npz")),
        'npy': list(episode_path.glob("**/*.npy")),
        'json': list(episode_path.glob("**/*.json")),
        'mp4': list(episode_path.glob("**/*.mp4")),
    }
    
    print(f"\n找到的文件:")
    print(f"  - NPZ 文件: {len(files['npz'])} 个")
    print(f"  - NPY 文件: {len(files['npy'])} 个")
    print(f"  - JSON 文件: {len(files['json'])} 个")
    print(f"  - MP4 文件: {len(files['mp4'])} 个")
    
    # 解压 NPZ 文件
    for npz_file in files['npz']:
        extract_npz(npz_file, output_dir)
    
    # 解压 NPY 文件
    for npy_file in files['npy']:
        extract_npy(npy_file, output_dir)
    
    # 显示 JSON 文件
    for json_file in files['json']:
        display_json(json_file)
    
    # MP4 文件只列出
    if files['mp4']:
        print(f"\n{'='*60}")
        print(f"视频文件 (无需解压):")
        print(f"{'='*60}")
        for mp4_file in files['mp4']:
            file_size_mb = mp4_file.stat().st_size / (1024 * 1024)
            print(f"  - {mp4_file.relative_to(episode_path)}: {file_size_mb:.2f} MB")
    
    print(f"\n{'#'*60}")
    print(f"# 解压完成!")
    print(f"# 所有数据已保存到: {output_dir}")
    print(f"{'#'*60}\n")


def main():
    parser = argparse.ArgumentParser(
        description='解压 episode 数据为 CSV 表格格式',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 解压单个 episode (生成 CSV 文件)
  python3 extract_episode_data.py collection_data/linkerhand_piper_grasp/session_2025-10-10_20-45-22/episode_000002
  
  # 指定输出目录
  python3 extract_episode_data.py episode_000002 --output_dir /tmp/extracted_data
  
输出文件:
  - telemetry_csv/: 所有遥测数据的 CSV 文件 (qpos, qvel, actions 等)
  - cameras_csv/: 相机时间戳 CSV 文件
  - summary.json: 数据摘要信息
        """
    )
    
    parser.add_argument('episode_path', type=str, help='Episode 目录路径')
    parser.add_argument('--output_dir', '-o', type=str, default=None,
                        help='输出目录 (默认: episode_path/extracted)')
    
    args = parser.parse_args()
    
    try:
        extract_episode(args.episode_path, args.output_dir)
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())

