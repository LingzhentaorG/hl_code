#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
batch_process.py - DEM 批量处理脚本
=====================================

功能概述：
  遍历指定目录下的所有 PLY 点云文件，为每个文件生成独立的配置，
  逐一调用 dem_cli 可执行程序进行 DEM 生成处理，并汇总输出结果报告。

使用方法：
  python scripts/batch_process.py --cli build/dem_cli.exe --config config/example_config.json --input-dir data/ --output-root output/

依赖：
  - Python 3.8+
  - dem_cli 可执行文件（需预先通过 CMake 构建完成）
"""

import argparse
import csv
import json
import subprocess
import sys
import time
from copy import deepcopy
from pathlib import Path


def main() -> int:
    """批量处理主函数：遍历 PLY 文件并逐个调用 dem_cli 处理"""
    
    # ==================== 命令行参数解析 ====================
    parser = argparse.ArgumentParser(description="DEM 批量处理工具")
    
    # dem_cli 可执行文件的路径（默认为构建目录下的 exe）
    parser.add_argument("--cli", default=str(Path("build") / "dem_cli.exe"),
                        help="dem_cli 可执行文件路径")
    
    # 基础配置模板文件路径
    parser.add_argument("--config", default=str(Path("config") / "example_config.json"),
                        help="基础 JSON 配置模板路径")
    
    # 包含输入 PLY 文件的目录
    parser.add_argument("--input-dir", default="data",
                        help="包含 PLY 点云文件的输入目录")
    
    # 批量输出的根目录（将在此下创建时间戳子目录）
    parser.add_argument("--output-root", default="output",
                        help="批量输出根目录")
    
    # PLY 文件匹配模式（支持通配符）
    parser.add_argument("--glob", default="*.ply",
                        help="PLY 文件名匹配模式")
    args = parser.parse_args()

    # 将相对路径转换为绝对路径以便后续操作
    cli_path = Path(args.cli).resolve()
    config_path = Path(args.config).resolve()
    input_dir = Path(args.input_dir).resolve()
    output_root = Path(args.output-root).resolve()

    # 前置检查：验证关键路径是否存在
    if not cli_path.exists():
        raise SystemExit(f"CLI 未找到: {cli_path}")
    if not config_path.exists():
        raise SystemExit(f"配置文件未找到: {config_path}")
    if not input_dir.exists():
        raise SystemExit(f"输入目录未找到: {input_dir}")

    # ==================== 创建批量输出目录结构 ====================
    # 使用当前时间戳创建唯一的批次目录，避免多次运行结果互相覆盖
    batch_root = output_root / f"batch_{time.strftime('%Y%m%d_%H%M%S')}"
    batch_root.mkdir(parents=True, exist_ok=True)
    
    # configs 子目录用于存放每个数据集的独立配置文件
    config_dir = batch_root / "configs"
    config_dir.mkdir(parents=True, exist_ok=True)

    # 加载基础配置模板作为后续修改的基准
    base_config = json.loads(config_path.read_text(encoding="utf-8"))

    # ==================== 遍历并处理每个 PLY 文件 ====================
    records = []   # 用于收集每个数据集的处理记录
    
    for index, ply_path in enumerate(sorted(input_dir.glob(args.glob)), start=1):
        # 从文件名提取数据集名称（不含扩展名）
        dataset_name = ply_path.stem
        
        # 为该数据集创建独立的输出和配置
        dataset_output = batch_root / dataset_name
        dataset_config = deepcopy(base_config)   # 深拷贝避免污染基础配置
        
        # 覆盖配置中的输入/输出路径
        dataset_config.setdefault("input", {})
        dataset_config.setdefault("output", {})
        dataset_config["input"]["file_path"] = ply_path.as_posix()
        dataset_config["output"]["directory"] = dataset_output.as_posix()
        
        # 批量模式下禁用时间戳子目录（已有批次级时间戳）
        dataset_config["output"]["timestamp_subdir"] = False
        dataset_config["output"]["write_png"] = True
        
        # 将独立配置写入磁盘供 dem_cli 读取
        dataset_config_path = config_dir / f"dataset_{index:02d}.json"
        dataset_config_path.write_text(
            json.dumps(dataset_config, ensure_ascii=False, indent=2), encoding="utf-8")

        # ==================== 调用 dem_cli 处理单个数据集 ====================
        started = time.time()
        command = [
            str(cli_path),
            "--config",
            str(dataset_config_path),
        ]
        result = subprocess.run(command, capture_output=True, text=True)
        elapsed = time.time() - started

        # 记录本次处理的结果信息
        record = {
            "dataset": dataset_name,
            "input_file": str(ply_path),
            "status": "success" if result.returncode == 0 else "failed",
            "return_code": result.returncode,
            "elapsed_seconds": round(elapsed, 3),
            "output_dir": str(dataset_output),
            "tile_mode_used": None,          # 是否使用了分块模式
            "stats_file": str(dataset_output / "log" / "stats.txt"),
            "stdout_tail": result.stdout[-4000:],   # 保留最后 4000 字符的输出
            "stderr_tail": result.stderr[-4000:],
        }
        
        # 尝试从统计文件中提取分块模式信息
        stats_path = dataset_output / "log" / "stats.txt"
        if stats_path.exists():
            for line in stats_path.read_text(encoding="utf-8").splitlines():
                if line.startswith("tile_mode_used="):
                    record["tile_mode_used"] = line.split("=", 1)[1].strip()
                    break
        
        records.append(record)

    # ==================== 生成汇总报告 ====================
    # 输出 JSON 格式的完整汇总报告
    summary_json = batch_root / "summary.json"
    summary_csv = batch_root / "summary.csv"
    summary_json.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")

    # 输出 CSV 格式的表格化汇总报告（便于 Excel 打开分析）
    with summary_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "dataset",           # 数据集名称
                "input_file",        # 输入文件路径
                "status",            # 处理状态 (success/failed)
                "return_code",       # 返回码
                "elapsed_seconds",   # 处理耗时（秒）
                "output_dir",        # 输出目录
                "tile_mode_used",    # 分块模式
                "stats_file",        # 统计文件路径
            ],
        )
        writer.writeheader()
        for record in records:
            writer.writerow({key: record.get(key) for key in writer.fieldnames})

    # 统计失败数量并返回相应退出码
    failures = [record for record in records if record["status"] != "success"]
    print(f"批量输出根目录: {batch_root}")
    print(f"共处理 {len(records)} 个文件，失败数: {len(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
