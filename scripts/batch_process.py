#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
batch_process.py - DEM batch runner
"""

from __future__ import annotations

import argparse
import csv
import os
import json
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from copy import deepcopy
from pathlib import Path


def run_dataset(cli_path: Path, ply_path: Path, dataset_config_path: Path, dataset_output: Path) -> dict:
    started = time.time()
    result = subprocess.run(
        [str(cli_path), "--config", str(dataset_config_path)],
        capture_output=True,
        text=True,
    )
    elapsed = time.time() - started

    stats_path = dataset_output / "log" / "stats.txt"
    tile_mode_used = None
    if stats_path.exists():
        for line in stats_path.read_text(encoding="utf-8").splitlines():
            if line.startswith("tile_mode_used="):
                tile_mode_used = line.split("=", 1)[1].strip()
                break

    return {
        "dataset": ply_path.stem,
        "input_file": str(ply_path),
        "status": "success" if result.returncode == 0 else "failed",
        "return_code": result.returncode,
        "elapsed_seconds": round(elapsed, 3),
        "output_dir": str(dataset_output),
        "tile_mode_used": tile_mode_used,
        "stats_file": str(stats_path),
        "stdout_tail": result.stdout[-4000:],
        "stderr_tail": result.stderr[-4000:],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="DEM 批量处理工具")
    parser.add_argument("--cli", default=str(Path("build") / "dem_cli.exe"), help="dem_cli 可执行文件路径")
    parser.add_argument("--config", default=str(Path("config") / "example_config.json"), help="基础 JSON 配置模板路径")
    parser.add_argument("--input-dir", default="data", help="包含 PLY 点云文件的输入目录")
    parser.add_argument("--output-root", default="output", help="批量输出根目录")
    parser.add_argument("--glob", default="*.ply", help="PLY 文件名匹配模式")
    parser.add_argument("--jobs", type=int, default=1, help="并发处理的数据集数量。1=串行")
    args = parser.parse_args()

    cli_path = Path(args.cli).resolve()
    config_path = Path(args.config).resolve()
    input_dir = Path(args.input_dir).resolve()
    output_root = Path(args.output_root).resolve()

    if not cli_path.exists():
        raise SystemExit(f"CLI 未找到: {cli_path}")
    if not config_path.exists():
        raise SystemExit(f"配置文件未找到: {config_path}")
    if not input_dir.exists():
        raise SystemExit(f"输入目录未找到: {input_dir}")
    if args.jobs < 1:
        raise SystemExit("--jobs 必须 >= 1")

    batch_root = output_root / f"batch_{time.strftime('%Y%m%d_%H%M%S')}"
    batch_root.mkdir(parents=True, exist_ok=True)
    config_dir = batch_root / "configs"
    config_dir.mkdir(parents=True, exist_ok=True)

    base_config = json.loads(config_path.read_text(encoding="utf-8"))
    records = []
    jobs = min(args.jobs, max(1, os.cpu_count() or 1))
    datasets = sorted(input_dir.glob(args.glob), key=lambda path: (-path.stat().st_size, path.name))
    tasks: list[tuple[int, Path, Path, Path]] = []

    for index, ply_path in enumerate(datasets, start=1):
        dataset_name = ply_path.stem
        dataset_output = batch_root / dataset_name
        dataset_config = deepcopy(base_config)
        dataset_config.setdefault("input", {})
        dataset_config.setdefault("output", {})
        dataset_config["input"]["file_path"] = ply_path.as_posix()
        dataset_config["output"]["directory"] = dataset_output.as_posix()
        dataset_config["output"]["timestamp_subdir"] = False
        dataset_config["output"]["write_png"] = True

        dataset_config_path = config_dir / f"dataset_{index:02d}.json"
        dataset_config_path.write_text(
            json.dumps(dataset_config, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        tasks.append((index, ply_path, dataset_config_path, dataset_output))

    print(f"批处理任务数: {len(tasks)}，并发数: {jobs}")
    if jobs == 1:
        for index, ply_path, dataset_config_path, dataset_output in tasks:
            print(f"[{index}/{len(tasks)}] 开始处理: {ply_path.name}")
            record = run_dataset(cli_path, ply_path, dataset_config_path, dataset_output)
            print(f"[{index}/{len(tasks)}] 完成处理: {ply_path.name} -> {record['status']} ({record['elapsed_seconds']}s)")
            records.append(record)
    else:
        order_map = {ply_path.stem: index for index, ply_path, _, _ in tasks}
        with ThreadPoolExecutor(max_workers=jobs) as executor:
            future_map = {
                executor.submit(run_dataset, cli_path, ply_path, dataset_config_path, dataset_output): (index, ply_path.name)
                for index, ply_path, dataset_config_path, dataset_output in tasks
            }
            for future in as_completed(future_map):
                index, dataset_name = future_map[future]
                record = future.result()
                print(f"[{index}/{len(tasks)}] 完成处理: {dataset_name} -> {record['status']} ({record['elapsed_seconds']}s)")
                records.append(record)

        records.sort(key=lambda record: order_map.get(record["dataset"], 10**9))

    summary_json = batch_root / "summary.json"
    summary_csv = batch_root / "summary.csv"
    summary_json.write_text(json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8")

    with summary_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "dataset",
                "input_file",
                "status",
                "return_code",
                "elapsed_seconds",
                "output_dir",
                "tile_mode_used",
                "stats_file",
            ],
        )
        writer.writeheader()
        for record in records:
            writer.writerow({key: record.get(key) for key in writer.fieldnames})

    failures = [record for record in records if record["status"] != "success"]
    print(f"批量输出根目录: {batch_root}")
    print(f"共处理 {len(records)} 个文件，失败数: {len(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
