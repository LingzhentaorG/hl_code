#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
release_product.py - Clean, rebuild, test, and generate the official release output.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = ROOT / "build"
OUTPUT_DIR = ROOT / "output"
FINAL_RELEASE_DIR = OUTPUT_DIR / "final_release"
SMOKE_OUTPUT_DIR = ROOT / "config" / "output_tile_smoke"
MANIFEST_PATH = ROOT / "config" / "official_dataset_manifest.json"
REFERENCE_MAP_PATH = ROOT / "config" / "reference_dem_map.json"
CONFIG_PATH = ROOT / "config" / "example_config.json"
PYTHON_TEST_PATH = ROOT / "tests" / "test_compare_dem_results.py"


def echo(message: str) -> None:
    print(f"[release] {message}", flush=True)


def remove_path(path: Path) -> None:
    if path.is_symlink() or path.is_file():
        path.unlink(missing_ok=True)
    elif path.is_dir():
        shutil.rmtree(path, ignore_errors=True)


def clean_workspace() -> None:
    for target in (BUILD_DIR, OUTPUT_DIR, SMOKE_OUTPUT_DIR):
        if target.exists():
            echo(f"Removing {target}")
            remove_path(target)

    for cache_dir in ROOT.rglob("__pycache__"):
        if cache_dir.is_dir():
            shutil.rmtree(cache_dir, ignore_errors=True)

    for cache_dir in ROOT.rglob(".pytest_cache"):
        if cache_dir.is_dir():
            shutil.rmtree(cache_dir, ignore_errors=True)

    for pattern in ("*.pyc", "*.pyo"):
        for file_path in ROOT.rglob(pattern):
            if file_path.is_file():
                file_path.unlink(missing_ok=True)


def run(command: list[str], *, cwd: Path = ROOT) -> None:
    echo("Running: " + " ".join(f'"{part}"' if " " in part else part for part in command))
    subprocess.run(command, cwd=cwd, check=True)


def resolve_executable(stem: str) -> Path:
    suffix = ".exe" if sys.platform.startswith("win") else ""
    candidates = [
        BUILD_DIR / f"{stem}{suffix}",
        BUILD_DIR / "Debug" / f"{stem}{suffix}",
        BUILD_DIR / "Release" / f"{stem}{suffix}",
        BUILD_DIR / "RelWithDebInfo" / f"{stem}{suffix}",
        BUILD_DIR / "MinSizeRel" / f"{stem}{suffix}",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate

    matches = sorted(BUILD_DIR.rglob(f"{stem}{suffix}"))
    if matches:
        return matches[0]
    raise FileNotFoundError(f"Unable to find built executable: {stem}{suffix}")


def load_manifest_dataset_names() -> list[str]:
    raw = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    if not isinstance(raw, list):
        raise RuntimeError("official_dataset_manifest.json must be a JSON array.")
    return [Path(item).stem for item in raw]


def verify_final_release() -> None:
    if not FINAL_RELEASE_DIR.exists():
        raise RuntimeError(f"Final release directory not found: {FINAL_RELEASE_DIR}")

    output_children = sorted(child.name for child in OUTPUT_DIR.iterdir())
    if output_children != ["final_release"]:
        raise RuntimeError(f"output/ should contain only final_release, got: {output_children}")

    required_root_files = [
        FINAL_RELEASE_DIR / "summary.json",
        FINAL_RELEASE_DIR / "summary.csv",
        FINAL_RELEASE_DIR / "comparison_summary.json",
        FINAL_RELEASE_DIR / "comparison_summary.csv",
    ]
    for path in required_root_files:
        if not path.exists():
            raise RuntimeError(f"Missing release artifact: {path}")

    expected_datasets = load_manifest_dataset_names()
    datasets_root = FINAL_RELEASE_DIR / "datasets"
    actual_datasets = sorted(path.name for path in datasets_root.iterdir() if path.is_dir())
    if sorted(expected_datasets) != actual_datasets:
        raise RuntimeError(f"Release datasets mismatch. expected={sorted(expected_datasets)} actual={actual_datasets}")

    for dataset in expected_datasets:
        dataset_dir = datasets_root / dataset
        for required_dir in ("surface", "mask", "log"):
            if not (dataset_dir / required_dir).exists():
                raise RuntimeError(f"Missing {required_dir} directory for dataset: {dataset_dir}")

    resource3_dir = datasets_root / "资源三号_石林_点云示范" / "comparison"
    for path in (
        resource3_dir / "report.json",
        resource3_dir / "summary.md",
        resource3_dir / "domain_overlap.png",
        resource3_dir / "diff_signed.png",
        resource3_dir / "diff_abs.png",
        resource3_dir / "slope_diff.png",
        resource3_dir / "hotspots.csv",
    ):
        if not path.exists():
            raise RuntimeError(f"Missing Resource3 comparison artifact: {path}")

    comparison_records = json.loads((FINAL_RELEASE_DIR / "comparison_summary.json").read_text(encoding="utf-8"))
    comparison_by_dataset = {record["dataset"]: record for record in comparison_records}
    for dataset in expected_datasets:
        if dataset not in comparison_by_dataset:
            raise RuntimeError(f"Dataset missing from comparison summary: {dataset}")
    if comparison_by_dataset["资源三号_石林_点云示范"]["status"] != "success":
        raise RuntimeError("Resource3 comparison did not succeed.")


def main() -> int:
    clean_workspace()

    run(["cmake", "-S", ".", "-B", "build"])
    run(["cmake", "--build", "build", "-j", "4"])

    dem_tests_path = resolve_executable("dem_tests")
    run([str(dem_tests_path)])
    run([sys.executable, str(PYTHON_TEST_PATH)])

    dem_cli_path = resolve_executable("dem_cli")
    run(
        [
            sys.executable,
            str(ROOT / "scripts" / "batch_process.py"),
            "--cli",
            str(dem_cli_path),
            "--config",
            str(CONFIG_PATH),
            "--input-dir",
            str(ROOT / "data"),
            "--output-root",
            str(OUTPUT_DIR),
            "--manifest",
            str(MANIFEST_PATH),
            "--batch-name",
            "final_release",
            "--compare-map",
            str(REFERENCE_MAP_PATH),
            "--jobs",
            "1",
        ]
    )

    run([sys.executable, str(PYTHON_TEST_PATH)])
    verify_final_release()
    echo(f"Official release is ready: {FINAL_RELEASE_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
