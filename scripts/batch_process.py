#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
batch_process.py - DEM batch runner
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from compare_dem_results import compare_dem_results


def load_manifest(path_value: str | None, input_dir: Path) -> list[Path] | None:
    if not path_value:
        return None

    manifest_path = Path(path_value).resolve()
    if not manifest_path.exists():
        raise SystemExit(f"数据集清单文件未找到: {manifest_path}")

    raw = json.loads(manifest_path.read_text(encoding="utf-8"))
    if isinstance(raw, list):
        items = raw
    elif isinstance(raw, dict) and isinstance(raw.get("datasets"), list):
        items = raw["datasets"]
    else:
        raise SystemExit('数据集清单必须是 JSON 数组，或 {"datasets": [...]} 对象。')

    datasets: list[Path] = []
    for index, item in enumerate(items, start=1):
        if not isinstance(item, str):
            raise SystemExit(f"数据集清单第 {index} 项必须是字符串。")

        candidate = Path(item)
        if not candidate.is_absolute():
            candidate = (input_dir / candidate).resolve()
        else:
            candidate = candidate.resolve()

        if not candidate.exists():
            raise SystemExit(f"数据集清单中的文件不存在: {candidate}")
        if candidate.suffix.lower() != ".ply":
            raise SystemExit(f"数据集清单中的文件必须是 .ply: {candidate}")
        datasets.append(candidate)

    return datasets


def load_compare_map(path_value: str | None) -> dict[str, Path] | None:
    if not path_value:
        return None

    compare_map_path = Path(path_value).resolve()
    if not compare_map_path.exists():
        raise SystemExit(f"对比映射文件未找到: {compare_map_path}")

    raw = json.loads(compare_map_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise SystemExit('对比映射文件必须是 JSON 对象，格式为 {"<dataset>": "<reference_dem_path>"}')

    mapping: dict[str, Path] = {}
    for dataset_name, reference_path in raw.items():
        if not isinstance(dataset_name, str) or not isinstance(reference_path, str):
            raise SystemExit("对比映射文件中的键和值都必须是字符串。")

        resolved = Path(reference_path)
        if not resolved.is_absolute():
            resolved = (compare_map_path.parent / resolved).resolve()
        else:
            resolved = resolved.resolve()
        mapping[dataset_name] = resolved

    return mapping


def build_comparison_record(dataset: str, reference_dem: Path | None, output_dir: Path, status: str, **kwargs: object) -> dict:
    record = {
        "dataset": dataset,
        "status": status,
        "reference_dem": str(reference_dem) if reference_dem is not None else None,
        "output_dir": str(output_dir),
        "report_path": None,
        "summary_path": None,
        "alignment_mode": None,
        "domain_jaccard": None,
        "domain_only_main_area": None,
        "domain_only_reference_area": None,
        "elevation_rmse": None,
        "elevation_p95_abs_error": None,
        "elevation_pass_ratio_le_0_1m": None,
        "hotspots_gt_10m": None,
        "message": None,
    }
    record.update(kwargs)
    return record


def run_dataset(
    cli_path: Path,
    base_config_path: Path,
    ply_path: Path,
    dataset_output: Path,
    compare_requested: bool,
    reference_dem: Path | None,
) -> tuple[dict, dict | None]:
    started = time.time()
    result = subprocess.run(
        [
            str(cli_path),
            "--config",
            str(base_config_path),
            "--set",
            f"input.file_path={json.dumps(ply_path.as_posix(), ensure_ascii=True)}",
            "--set",
            f"output.directory={json.dumps(dataset_output.as_posix(), ensure_ascii=True)}",
            "--set",
            "output.timestamp_subdir=false",
            "--set",
            "output.write_png=true",
        ],
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

    main_record = {
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

    comparison_record = None
    if compare_requested:
        comparison_output = dataset_output / "comparison"
        if result.returncode != 0:
            comparison_record = build_comparison_record(
                dataset=ply_path.stem,
                reference_dem=reference_dem,
                output_dir=comparison_output,
                status="skipped",
                message="main_processing_failed",
            )
        elif reference_dem is None:
            comparison_record = build_comparison_record(
                dataset=ply_path.stem,
                reference_dem=None,
                output_dir=comparison_output,
                status="skipped",
                message="missing_reference_mapping",
            )
        else:
            try:
                report = compare_dem_results(
                    main_dem_path=dataset_output / "surface" / "dem.tif",
                    reference_dem_path=reference_dem,
                    mask_dir=dataset_output / "mask",
                    output_dir=comparison_output,
                    dataset_name=ply_path.stem,
                )
                comparison_record = build_comparison_record(
                    dataset=ply_path.stem,
                    reference_dem=reference_dem,
                    output_dir=comparison_output,
                    status="success",
                    report_path=str(comparison_output / "report.json"),
                    summary_path=str(comparison_output / "summary.md"),
                    alignment_mode=report["alignment"]["mode"],
                    domain_jaccard=report["domain"]["jaccard"],
                    domain_only_main_area=report["domain"]["only_main_area"],
                    domain_only_reference_area=report["domain"]["only_reference_area"],
                    elevation_rmse=report["elevation"]["rmse"],
                    elevation_p95_abs_error=report["elevation"]["p95_abs_error"],
                    elevation_pass_ratio_le_0_1m=report["elevation"]["pass_ratio_le_0.1m"],
                    hotspots_gt_10m=report["hotspots"]["counts"]["gt_10m"],
                    message="ok",
                )
            except Exception as exc:
                comparison_record = build_comparison_record(
                    dataset=ply_path.stem,
                    reference_dem=reference_dem,
                    output_dir=comparison_output,
                    status="failed",
                    message=str(exc),
                )

    return main_record, comparison_record


def main() -> int:
    parser = argparse.ArgumentParser(description="DEM 批量处理工具")
    parser.add_argument("--cli", default=str(Path("build") / "dem_cli.exe"), help="dem_cli 可执行文件路径")
    parser.add_argument("--config", default=str(Path("config") / "example_config.json"), help="基础 JSON 配置模板路径")
    parser.add_argument("--input-dir", default="data", help="包含 PLY 点云文件的输入目录")
    parser.add_argument("--output-root", default="output", help="批量输出根目录")
    parser.add_argument("--glob", default="*.ply", help="PLY 文件名匹配模式")
    parser.add_argument("--manifest", help='官方数据集清单 JSON 文件路径，格式为 ["a.ply", "b.ply"]')
    parser.add_argument("--batch-name", help="固定批处理输出目录名；不传则自动生成时间戳目录")
    parser.add_argument("--jobs", type=int, default=1, help="并发处理的数据集数量。1=串行")
    parser.add_argument("--compare-map", help='对比 DEM 映射 JSON 文件路径，格式为 {"<dataset>": "<reference_dem_path>"}')
    args = parser.parse_args()

    cli_path = Path(args.cli).resolve()
    config_path = Path(args.config).resolve()
    input_dir = Path(args.input_dir).resolve()
    output_root = Path(args.output_root).resolve()
    manifest_datasets = load_manifest(args.manifest, input_dir)
    compare_map = load_compare_map(args.compare_map)
    compare_requested = args.compare_map is not None

    if not cli_path.exists():
        raise SystemExit(f"CLI 未找到: {cli_path}")
    if not config_path.exists():
        raise SystemExit(f"配置文件未找到: {config_path}")
    if not input_dir.exists():
        raise SystemExit(f"输入目录未找到: {input_dir}")
    if args.jobs < 1:
        raise SystemExit("--jobs 必须 >= 1")

    batch_root = output_root / args.batch_name if args.batch_name else output_root / f"batch_{time.strftime('%Y%m%d_%H%M%S')}"
    if batch_root.exists() and any(batch_root.iterdir()):
        raise SystemExit(f"输出目录已存在且非空，请先清理后再运行: {batch_root}")
    batch_root.mkdir(parents=True, exist_ok=True)
    datasets_root = batch_root / "datasets"
    datasets_root.mkdir(parents=True, exist_ok=True)

    records = []
    comparison_records = []
    jobs = min(args.jobs, max(1, os.cpu_count() or 1))
    datasets = manifest_datasets if manifest_datasets is not None else sorted(
        input_dir.glob(args.glob), key=lambda path: (-path.stat().st_size, path.name)
    )
    if not datasets:
        shutil.rmtree(batch_root, ignore_errors=True)
        raise SystemExit("没有找到可处理的 PLY 数据集。")
    tasks: list[tuple[int, Path, Path, Path | None]] = []

    for index, ply_path in enumerate(datasets, start=1):
        dataset_name = ply_path.stem
        dataset_output = datasets_root / dataset_name
        reference_dem = None if compare_map is None else compare_map.get(dataset_name)
        tasks.append((index, ply_path, dataset_output, reference_dem))

    print(f"批处理任务数: {len(tasks)}，并发数: {jobs}")
    if jobs == 1:
        for index, ply_path, dataset_output, reference_dem in tasks:
            print(f"[{index}/{len(tasks)}] 开始处理: {ply_path.name}")
            record, comparison_record = run_dataset(
                cli_path,
                config_path,
                ply_path,
                dataset_output,
                compare_requested=compare_requested,
                reference_dem=reference_dem,
            )
            print(f"[{index}/{len(tasks)}] 完成处理: {ply_path.name} -> {record['status']} ({record['elapsed_seconds']}s)")
            records.append(record)
            if comparison_record is not None:
                print(f"[{index}/{len(tasks)}] 对比分析: {ply_path.name} -> {comparison_record['status']}")
                comparison_records.append(comparison_record)
    else:
        order_map = {ply_path.stem: index for index, ply_path, _, _ in tasks}
        with ThreadPoolExecutor(max_workers=jobs) as executor:
            future_map = {
                executor.submit(
                    run_dataset,
                    cli_path,
                    config_path,
                    ply_path,
                    dataset_output,
                    compare_requested,
                    reference_dem,
                ): (index, ply_path.name)
                for index, ply_path, dataset_output, reference_dem in tasks
            }
            for future in as_completed(future_map):
                index, dataset_name = future_map[future]
                record, comparison_record = future.result()
                print(f"[{index}/{len(tasks)}] 完成处理: {dataset_name} -> {record['status']} ({record['elapsed_seconds']}s)")
                records.append(record)
                if comparison_record is not None:
                    print(f"[{index}/{len(tasks)}] 对比分析: {dataset_name} -> {comparison_record['status']}")
                    comparison_records.append(comparison_record)

        records.sort(key=lambda record: order_map.get(record["dataset"], 10**9))
        comparison_records.sort(key=lambda record: order_map.get(record["dataset"], 10**9))

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

    if compare_requested:
        comparison_json = batch_root / "comparison_summary.json"
        comparison_csv = batch_root / "comparison_summary.csv"
        comparison_json.write_text(json.dumps(comparison_records, ensure_ascii=False, indent=2), encoding="utf-8")

        with comparison_csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "dataset",
                    "status",
                    "reference_dem",
                    "output_dir",
                    "report_path",
                    "summary_path",
                    "alignment_mode",
                    "domain_jaccard",
                    "domain_only_main_area",
                    "domain_only_reference_area",
                    "elevation_rmse",
                    "elevation_p95_abs_error",
                    "elevation_pass_ratio_le_0_1m",
                    "hotspots_gt_10m",
                    "message",
                ],
            )
            writer.writeheader()
            for record in comparison_records:
                writer.writerow({key: record.get(key) for key in writer.fieldnames})

    failures = [record for record in records if record["status"] != "success"]
    print(f"批量输出根目录: {batch_root}")
    print(f"共处理 {len(records)} 个文件，失败数: {len(failures)}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
