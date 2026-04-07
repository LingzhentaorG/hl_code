#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
compare_dem_results.py - Compare project DEM output against a reference DEM.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import numpy as np
import rasterio
from matplotlib import colormaps
from PIL import Image
from rasterio.coords import BoundingBox
from rasterio.enums import Resampling
from rasterio.transform import array_bounds
from rasterio.warp import reproject


EPSILON = 1e-6
ELEVATION_PASS_THRESHOLDS = (0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0)
HOTSPOT_THRESHOLDS = (1.0, 2.0, 5.0, 10.0)


@dataclass
class RasterBundle:
    path: Path
    data: np.ndarray
    valid_mask: np.ndarray
    nodata: float | None
    width: int
    height: int
    transform: Any
    crs: Any
    bounds: BoundingBox
    res_x: float
    res_y: float
    dtype: str


def _float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(numeric):
        return None
    return numeric


def _round_or_none(value: float | None, digits: int = 12) -> float | None:
    if value is None or not math.isfinite(value):
        return None
    return round(value, digits)


def _to_bbox_dict(bounds: BoundingBox | None) -> dict[str, float] | None:
    if bounds is None:
        return None
    return {
        "left": float(bounds.left),
        "bottom": float(bounds.bottom),
        "right": float(bounds.right),
        "top": float(bounds.top),
    }


def _format_value(value: float | None, digits: int = 6) -> str:
    if value is None:
        return "N/A"
    return f"{value:.{digits}f}"


def _format_count(value: int | None) -> str:
    if value is None:
        return "N/A"
    return f"{value:,}"


def _count_to_area(count: int, pixel_area: float) -> float:
    return float(count) * pixel_area


def _fraction(numerator: int, denominator: int) -> float | None:
    if denominator <= 0:
        return None
    return float(numerator) / float(denominator)


def _bounds_intersection(a: BoundingBox, b: BoundingBox) -> BoundingBox | None:
    left = max(a.left, b.left)
    right = min(a.right, b.right)
    bottom = max(a.bottom, b.bottom)
    top = min(a.top, b.top)
    if right <= left or top <= bottom:
        return None
    return BoundingBox(left, bottom, right, top)


def _bounds_area(bounds: BoundingBox | None) -> float:
    if bounds is None:
        return 0.0
    return max(0.0, float(bounds.right - bounds.left)) * max(0.0, float(bounds.top - bounds.bottom))


def _read_single_band_raster(path: Path) -> RasterBundle:
    with rasterio.open(path) as dataset:
        if dataset.count != 1:
            raise ValueError(f"Expected a single-band raster: {path}")
        data = dataset.read(1)
        nodata = _float_or_none(dataset.nodata)
        valid_mask = np.isfinite(data)
        if nodata is not None:
            valid_mask &= data != nodata
        return RasterBundle(
            path=path,
            data=data,
            valid_mask=valid_mask,
            nodata=nodata,
            width=dataset.width,
            height=dataset.height,
            transform=dataset.transform,
            crs=dataset.crs,
            bounds=dataset.bounds,
            res_x=float(dataset.res[0]),
            res_y=float(dataset.res[1]),
            dtype=str(dataset.dtypes[0]),
        )


def _read_optional_mask(mask_dir: Path, filename: str, shape: tuple[int, int]) -> np.ndarray:
    path = mask_dir / filename
    if not path.exists():
        return np.zeros(shape, dtype=bool)
    bundle = _read_single_band_raster(path)
    if bundle.data.shape != shape:
        raise ValueError(f"Mask shape mismatch for {path}: expected {shape}, got {bundle.data.shape}")
    return bundle.valid_mask & (bundle.data > 0.5)


def _is_axis_aligned(transform: Any) -> bool:
    return abs(float(transform.b)) <= EPSILON and abs(float(transform.d)) <= EPSILON


def _detect_integer_alignment(main: RasterBundle, reference: RasterBundle) -> tuple[int, int] | None:
    if not _is_axis_aligned(main.transform) or not _is_axis_aligned(reference.transform):
        return None
    if abs(main.res_x - reference.res_x) > EPSILON or abs(main.res_y - reference.res_y) > EPSILON:
        return None

    col_offset = (float(reference.transform.c) - float(main.transform.c)) / main.res_x
    row_offset = (float(main.transform.f) - float(reference.transform.f)) / main.res_y
    rounded_col = round(col_offset)
    rounded_row = round(row_offset)

    if abs(col_offset - rounded_col) > EPSILON or abs(row_offset - rounded_row) > EPSILON:
        return None
    return int(rounded_row), int(rounded_col)


def _align_reference_to_main(main: RasterBundle, reference: RasterBundle) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    integer_offsets = _detect_integer_alignment(main, reference)
    if integer_offsets is not None:
        row_offset, col_offset = integer_offsets
        aligned_data = np.full(main.data.shape, np.nan, dtype=reference.data.dtype)
        aligned_valid = np.zeros(main.data.shape, dtype=bool)

        main_row_start = max(0, row_offset)
        main_col_start = max(0, col_offset)
        ref_row_start = max(0, -row_offset)
        ref_col_start = max(0, -col_offset)
        copy_rows = min(main.height - main_row_start, reference.height - ref_row_start)
        copy_cols = min(main.width - main_col_start, reference.width - ref_col_start)

        if copy_rows > 0 and copy_cols > 0:
            main_row_slice = slice(main_row_start, main_row_start + copy_rows)
            main_col_slice = slice(main_col_start, main_col_start + copy_cols)
            ref_row_slice = slice(ref_row_start, ref_row_start + copy_rows)
            ref_col_slice = slice(ref_col_start, ref_col_start + copy_cols)

            aligned_data[main_row_slice, main_col_slice] = reference.data[ref_row_slice, ref_col_slice]
            aligned_valid[main_row_slice, main_col_slice] = reference.valid_mask[ref_row_slice, ref_col_slice]
            aligned_data[~aligned_valid] = np.nan

        overlap_bounds = None
        if copy_rows > 0 and copy_cols > 0:
            overlap_bounds = BoundingBox(
                left=float(main.transform.c) + main_col_start * main.res_x,
                bottom=float(main.transform.f) - (main_row_start + copy_rows) * main.res_y,
                right=float(main.transform.c) + (main_col_start + copy_cols) * main.res_x,
                top=float(main.transform.f) - main_row_start * main.res_y,
            )

        meta = {
            "mode": "integer_window",
            "row_offset": row_offset,
            "col_offset": col_offset,
            "copy_rows": int(max(copy_rows, 0)),
            "copy_cols": int(max(copy_cols, 0)),
            "overlap_bounds": _to_bbox_dict(overlap_bounds),
            "reference_clipped_to_main_grid": bool(copy_rows < reference.height or copy_cols < reference.width),
        }
        return aligned_data, aligned_valid, meta

    if main.crs is None or reference.crs is None:
        raise ValueError("Non-integer raster alignment requires both rasters to have a CRS.")

    aligned_dtype = np.result_type(reference.data.dtype, np.float32)
    aligned_data = np.full(main.data.shape, np.nan, dtype=aligned_dtype)
    reproject(
        source=reference.data,
        destination=aligned_data,
        src_transform=reference.transform,
        src_crs=reference.crs,
        src_nodata=reference.nodata,
        dst_transform=main.transform,
        dst_crs=main.crs,
        dst_nodata=np.nan,
        resampling=Resampling.bilinear,
    )

    reference_valid_u8 = reference.valid_mask.astype(np.uint8)
    aligned_valid_u8 = np.zeros(main.data.shape, dtype=np.uint8)
    reproject(
        source=reference_valid_u8,
        destination=aligned_valid_u8,
        src_transform=reference.transform,
        src_crs=reference.crs,
        src_nodata=0,
        dst_transform=main.transform,
        dst_crs=main.crs,
        dst_nodata=0,
        resampling=Resampling.nearest,
    )
    aligned_valid = aligned_valid_u8 > 0
    aligned_data[~aligned_valid] = np.nan

    approx_col_offset = (float(reference.transform.c) - float(main.transform.c)) / main.res_x
    approx_row_offset = (float(main.transform.f) - float(reference.transform.f)) / main.res_y
    overlap_bounds = _bounds_intersection(
        main.bounds,
        BoundingBox(*array_bounds(reference.height, reference.width, reference.transform)),
    )

    meta = {
        "mode": "reprojected",
        "row_offset": _round_or_none(approx_row_offset),
        "col_offset": _round_or_none(approx_col_offset),
        "copy_rows": main.height,
        "copy_cols": main.width,
        "overlap_bounds": _to_bbox_dict(overlap_bounds),
        "reference_clipped_to_main_grid": True,
    }
    return aligned_data, aligned_valid, meta


def _compute_basic_stats(values: np.ndarray) -> dict[str, float | None]:
    if values.size == 0:
        return {
            "count": 0,
            "bias": None,
            "mae": None,
            "rmse": None,
            "median_abs_error": None,
            "p90_abs_error": None,
            "p95_abs_error": None,
            "p99_abs_error": None,
            "max_abs_error": None,
        }

    abs_values = np.abs(values)
    return {
        "count": int(values.size),
        "bias": float(np.mean(values, dtype=np.float64)),
        "mae": float(np.mean(abs_values, dtype=np.float64)),
        "rmse": float(np.sqrt(np.mean(values * values, dtype=np.float64))),
        "median_abs_error": float(np.quantile(abs_values, 0.5)),
        "p90_abs_error": float(np.quantile(abs_values, 0.90)),
        "p95_abs_error": float(np.quantile(abs_values, 0.95)),
        "p99_abs_error": float(np.quantile(abs_values, 0.99)),
        "max_abs_error": float(np.max(abs_values)),
    }


def _compute_slope_degrees(data: np.ndarray, valid_mask: np.ndarray, cell_x: float, cell_y: float) -> np.ndarray:
    work = data.astype(np.float32, copy=True)
    work[~valid_mask] = np.nan
    grad_y, grad_x = np.gradient(work, cell_y, cell_x)
    slope = np.degrees(np.arctan(np.sqrt(grad_x * grad_x + grad_y * grad_y))).astype(np.float32)
    slope[~np.isfinite(slope)] = np.nan
    return slope


def _save_colorized_png(
    data: np.ndarray,
    valid_mask: np.ndarray,
    output_path: Path,
    cmap_name: str,
    vmin: float,
    vmax: float,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rgba = np.zeros((data.shape[0], data.shape[1], 4), dtype=np.uint8)
    rgba[..., 3] = 0
    if not np.any(valid_mask):
        Image.fromarray(rgba).save(output_path)
        return

    if not math.isfinite(vmin) or not math.isfinite(vmax) or vmax <= vmin:
        vmax = vmin + 1.0

    normalized = np.zeros(data.shape, dtype=np.float32)
    normalized[valid_mask] = np.clip((data[valid_mask] - vmin) / (vmax - vmin), 0.0, 1.0)
    cmap = colormaps[cmap_name]
    rgba[valid_mask] = (cmap(normalized[valid_mask]) * 255).astype(np.uint8)
    rgba[..., 3][valid_mask] = 255
    Image.fromarray(rgba).save(output_path)


def _save_domain_overlap_png(
    main_valid: np.ndarray,
    reference_valid: np.ndarray,
    output_path: Path,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    rgba = np.zeros((main_valid.shape[0], main_valid.shape[1], 4), dtype=np.uint8)
    rgba[..., 3] = 255

    both = main_valid & reference_valid
    only_main = main_valid & (~reference_valid)
    only_reference = (~main_valid) & reference_valid
    neither = (~main_valid) & (~reference_valid)

    rgba[both] = np.array([40, 167, 69, 255], dtype=np.uint8)
    rgba[only_main] = np.array([52, 152, 219, 255], dtype=np.uint8)
    rgba[only_reference] = np.array([231, 76, 60, 255], dtype=np.uint8)
    rgba[neither] = np.array([0, 0, 0, 0], dtype=np.uint8)
    Image.fromarray(rgba).save(output_path)


def _top_hotspots(
    diff_full: np.ndarray,
    main: RasterBundle,
    reference_aligned: np.ndarray,
    object_mask: np.ndarray,
    boundary_mask: np.ndarray,
    limit: int = 10,
) -> list[dict[str, Any]]:
    if limit <= 0:
        return []

    scores = np.nan_to_num(np.abs(diff_full), nan=-1.0)
    if scores.size == 0:
        return []

    flat_scores = scores.reshape(-1)
    positive_count = int(np.count_nonzero(flat_scores > 0.0))
    if positive_count == 0:
        return []

    top_n = min(limit, positive_count)
    candidate_indices = np.argpartition(flat_scores, -top_n)[-top_n:]
    candidate_indices = candidate_indices[np.argsort(flat_scores[candidate_indices])[::-1]]

    results: list[dict[str, Any]] = []
    flat_main = main.data.reshape(-1)
    flat_reference = reference_aligned.reshape(-1)
    flat_object = object_mask.reshape(-1)
    flat_boundary = boundary_mask.reshape(-1)

    for rank, flat_index in enumerate(candidate_indices, start=1):
        abs_score = float(flat_scores[flat_index])
        if abs_score <= 0.0:
            continue
        row, col = divmod(int(flat_index), main.width)
        x = float(main.transform.c) + (col + 0.5) * main.res_x
        y = float(main.transform.f) - (row + 0.5) * main.res_y
        results.append(
            {
                "rank": rank,
                "row": row,
                "col": col,
                "x": x,
                "y": y,
                "main_elevation": float(flat_main[flat_index]),
                "reference_elevation": float(flat_reference[flat_index]),
                "diff_reference_minus_main": float(diff_full.reshape(-1)[flat_index]),
                "abs_diff": abs_score,
                "on_object_mask": bool(flat_object[flat_index]),
                "on_boundary_mask": bool(flat_boundary[flat_index]),
            }
        )
    return results


def _write_hotspots_csv(path: Path, hotspots: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "rank",
        "row",
        "col",
        "x",
        "y",
        "main_elevation",
        "reference_elevation",
        "diff_reference_minus_main",
        "abs_diff",
        "on_object_mask",
        "on_boundary_mask",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for hotspot in hotspots:
            writer.writerow(hotspot)


def _domain_commentary(domain: dict[str, Any]) -> str:
    only_reference_area = domain["only_reference_area"]
    only_main_area = domain["only_main_area"]
    jaccard = domain["jaccard"]

    if only_reference_area > only_main_area * 1.5 and only_reference_area > 0.0:
        return "参考结果在官方域外存在更明显的外扩，官方结果的边界更保守。"
    if only_main_area > only_reference_area * 1.5 and only_main_area > 0.0:
        return "官方结果在部分区域覆盖更广，参考结果边界相对更收缩。"
    if jaccard is not None and jaccard >= 0.995:
        return "两者有效域高度一致，范围差异很小。"
    return "两者有效域总体接近，但仍存在可见的边界差异。"


def _elevation_commentary(elevation: dict[str, Any]) -> str:
    rmse = elevation["rmse"]
    pass_0_1 = elevation["pass_ratio_le_0.1m"]
    pass_0_5 = elevation["pass_ratio_le_0.5m"]
    if rmse is None:
        return "共享有效像元为空，无法进行高程差异评价。"
    if rmse <= 0.2 and (pass_0_1 or 0.0) >= 0.95:
        return "共享范围内两者高程结果高度一致。"
    if rmse <= 1.0 and (pass_0_5 or 0.0) >= 0.98:
        return "共享范围内两者高程结果较一致。"
    return "共享范围内存在明显高程差异，需要结合热点位置进一步排查。"


def _hotspot_commentary(hotspots: dict[str, Any]) -> str:
    count_gt_10 = hotspots["counts"]["gt_10m"]
    count_gt_5 = hotspots["counts"]["gt_5m"]
    if count_gt_10 > 0:
        return "整体一致，但存在局部 10m 以上异常热点。"
    if count_gt_5 > 0:
        return "整体一致，但存在局部 5m 以上异常热点。"
    return "未发现显著的大幅度异常热点。"


def _write_summary_markdown(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    alignment = report["alignment"]
    domain = report["domain"]
    elevation = report["elevation"]
    slope = report["slope"]
    hotspots = report["hotspots"]
    interpretation = report["interpretation"]

    lines = [
        "# DEM 对比分析",
        "",
        f"- 数据集: {report['metadata']['dataset_name']}",
        f"- 主结果: `{report['metadata']['main_dem']}`",
        f"- 参考结果: `{report['metadata']['reference_dem']}`",
        f"- 对齐模式: `{alignment['mode']}`",
        f"- 行列偏移: row={alignment['row_offset']}, col={alignment['col_offset']}",
        "",
        "## 范围一致性",
        f"- 共享有效像元: {_format_count(domain['intersection_valid_count'])}",
        f"- Jaccard: {_format_value(domain['jaccard'])}",
        f"- Dice: {_format_value(domain['dice'])}",
        f"- 仅主结果有效面积: {_format_value(domain['only_main_area'], 2)}",
        f"- 仅参考结果有效面积: {_format_value(domain['only_reference_area'], 2)}",
        f"- 结论: {interpretation['domain']}",
        "",
        "## 高程差异",
        f"- Bias (参考-主结果): {_format_value(elevation['bias'])}",
        f"- MAE: {_format_value(elevation['mae'])}",
        f"- RMSE: {_format_value(elevation['rmse'])}",
        f"- P95 绝对误差: {_format_value(elevation['p95_abs_error'])}",
        f"- |dz|<=0.1m 占比: {_format_value(elevation['pass_ratio_le_0.1m'])}",
        f"- |dz|<=0.5m 占比: {_format_value(elevation['pass_ratio_le_0.5m'])}",
        f"- 结论: {interpretation['elevation']}",
        "",
        "## 地形形态",
        f"- 坡度 Bias: {_format_value(slope['bias'])}",
        f"- 坡度 MAE: {_format_value(slope['mae'])}",
        f"- 坡度 RMSE: {_format_value(slope['rmse'])}",
        f"- 坡度 P95 绝对误差: {_format_value(slope['p95_abs_error'])}",
        "",
        "## 异常热点",
        f"- |dz|>1m 数量: {_format_count(hotspots['counts']['gt_1m'])}",
        f"- |dz|>5m 数量: {_format_count(hotspots['counts']['gt_5m'])}",
        f"- |dz|>10m 数量: {_format_count(hotspots['counts']['gt_10m'])}",
        f"- 结论: {interpretation['hotspots']}",
    ]

    if hotspots["top_records"]:
        lines.extend(["", "## Top 10 热点", ""])
        for record in hotspots["top_records"]:
            lines.append(
                "- "
                f"#{record['rank']} row={record['row']} col={record['col']} "
                f"x={_format_value(record['x'], 3)} y={_format_value(record['y'], 3)} "
                f"dz={_format_value(record['diff_reference_minus_main'])} "
                f"|dz|={_format_value(record['abs_diff'])} "
                f"object={record['on_object_mask']} boundary={record['on_boundary_mask']}"
            )

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def compare_dem_results(
    main_dem_path: str | Path,
    reference_dem_path: str | Path,
    mask_dir: str | Path,
    output_dir: str | Path,
    dataset_name: str | None = None,
) -> dict[str, Any]:
    main_path = Path(main_dem_path).resolve()
    reference_path = Path(reference_dem_path).resolve()
    mask_root = Path(mask_dir).resolve()
    output_root = Path(output_dir).resolve()
    output_root.mkdir(parents=True, exist_ok=True)

    main = _read_single_band_raster(main_path)
    reference = _read_single_band_raster(reference_path)

    object_mask = _read_optional_mask(mask_root, "dem_object_mask.tif", main.data.shape)
    boundary_mask = _read_optional_mask(mask_root, "dem_boundary_mask.tif", main.data.shape)

    aligned_reference, aligned_reference_valid, alignment_meta = _align_reference_to_main(main, reference)

    main_valid = main.valid_mask
    reference_valid = aligned_reference_valid
    both_valid = main_valid & reference_valid
    only_main = main_valid & (~reference_valid)
    only_reference = (~main_valid) & reference_valid
    union_valid = main_valid | reference_valid

    pixel_area = abs(main.res_x * main.res_y)
    reference_pixel_area = abs(reference.res_x * reference.res_y)
    raw_reference_valid_count = int(np.count_nonzero(reference.valid_mask))
    aligned_reference_valid_count = int(np.count_nonzero(reference_valid))
    main_valid_count = int(np.count_nonzero(main_valid))
    intersection_valid_count = int(np.count_nonzero(both_valid))
    only_main_count = int(np.count_nonzero(only_main))
    only_reference_count = int(np.count_nonzero(only_reference))
    union_valid_count = int(np.count_nonzero(union_valid))

    bbox_intersection = _bounds_intersection(main.bounds, reference.bounds)
    main_bounds_area = _bounds_area(main.bounds)
    reference_bounds_area = _bounds_area(reference.bounds)
    intersection_area = _bounds_area(bbox_intersection)

    domain = {
        "pixel_area": pixel_area,
        "main_valid_count": main_valid_count,
        "reference_valid_count_raw": raw_reference_valid_count,
        "reference_valid_count_aligned": aligned_reference_valid_count,
        "intersection_valid_count": intersection_valid_count,
        "union_valid_count": union_valid_count,
        "only_main_count": only_main_count,
        "only_reference_count": only_reference_count,
        "main_valid_area": _count_to_area(main_valid_count, pixel_area),
        "reference_valid_area_raw": _count_to_area(raw_reference_valid_count, reference_pixel_area),
        "reference_valid_area_aligned": _count_to_area(aligned_reference_valid_count, pixel_area),
        "intersection_area": _count_to_area(intersection_valid_count, pixel_area),
        "only_main_area": _count_to_area(only_main_count, pixel_area),
        "only_reference_area": _count_to_area(only_reference_count, pixel_area),
        "jaccard": _fraction(intersection_valid_count, union_valid_count),
        "dice": _fraction(2 * intersection_valid_count, main_valid_count + aligned_reference_valid_count),
        "shared_ratio_vs_main": _fraction(intersection_valid_count, main_valid_count),
        "shared_ratio_vs_reference": _fraction(intersection_valid_count, aligned_reference_valid_count),
        "bbox_overlap_area": intersection_area,
        "bbox_overlap_ratio_vs_main": None if main_bounds_area <= 0.0 else intersection_area / main_bounds_area,
        "bbox_overlap_ratio_vs_reference": None if reference_bounds_area <= 0.0 else intersection_area / reference_bounds_area,
        "reference_valid_outside_main_bbox_count": max(0, raw_reference_valid_count - aligned_reference_valid_count),
    }

    diff_full = np.full(main.data.shape, np.nan, dtype=np.float32)
    valid_diff = aligned_reference[both_valid].astype(np.float64, copy=False) - main.data[both_valid].astype(np.float64, copy=False)
    diff_full[both_valid] = valid_diff.astype(np.float32, copy=False)
    elevation = _compute_basic_stats(valid_diff)
    for threshold in ELEVATION_PASS_THRESHOLDS:
        count = int(np.count_nonzero(np.abs(valid_diff) <= threshold)) if valid_diff.size else 0
        elevation[f"pass_count_le_{threshold}m"] = count
        elevation[f"pass_ratio_le_{threshold}m"] = _fraction(count, int(valid_diff.size))

    slope_main = _compute_slope_degrees(main.data, main_valid, main.res_x, main.res_y)
    slope_reference = _compute_slope_degrees(aligned_reference, reference_valid, main.res_x, main.res_y)
    slope_valid = both_valid & np.isfinite(slope_main) & np.isfinite(slope_reference)
    slope_diff = np.full(main.data.shape, np.nan, dtype=np.float32)
    slope_diff[slope_valid] = slope_reference[slope_valid] - slope_main[slope_valid]
    slope_metrics = _compute_basic_stats(slope_diff[slope_valid])

    hotspot_counts = {}
    for threshold in HOTSPOT_THRESHOLDS:
        count = int(np.count_nonzero(np.abs(valid_diff) > threshold)) if valid_diff.size else 0
        hotspot_counts[f"gt_{int(threshold)}m"] = count
        hotspot_counts[f"ratio_gt_{int(threshold)}m"] = _fraction(count, int(valid_diff.size))

    top_records = _top_hotspots(diff_full, main, aligned_reference, object_mask, boundary_mask, limit=10)
    hotspots = {
        "counts": hotspot_counts,
        "top_records": top_records,
    }

    diff_limit = 1.0
    if valid_diff.size:
        diff_limit = max(0.25, float(np.quantile(np.abs(valid_diff), 0.99)))
    abs_diff_full = np.abs(diff_full)
    abs_diff_limit = 1.0
    if valid_diff.size:
        abs_diff_limit = max(0.25, float(np.quantile(np.abs(valid_diff), 0.99)))

    valid_slope_diff = slope_diff[slope_valid]
    slope_limit = 1.0
    if valid_slope_diff.size:
        slope_limit = max(1.0, float(np.quantile(np.abs(valid_slope_diff), 0.99)))

    domain_overlap_png = output_root / "domain_overlap.png"
    diff_signed_png = output_root / "diff_signed.png"
    diff_abs_png = output_root / "diff_abs.png"
    slope_diff_png = output_root / "slope_diff.png"
    hotspots_csv = output_root / "hotspots.csv"
    report_json = output_root / "report.json"
    summary_md = output_root / "summary.md"

    _save_domain_overlap_png(main_valid, reference_valid, domain_overlap_png)
    _save_colorized_png(diff_full, both_valid, diff_signed_png, "coolwarm", -diff_limit, diff_limit)
    _save_colorized_png(abs_diff_full, both_valid, diff_abs_png, "inferno", 0.0, abs_diff_limit)
    _save_colorized_png(slope_diff, slope_valid, slope_diff_png, "coolwarm", -slope_limit, slope_limit)
    _write_hotspots_csv(hotspots_csv, top_records)

    interpretation = {
        "domain": _domain_commentary(domain),
        "elevation": _elevation_commentary(elevation),
        "hotspots": _hotspot_commentary(hotspots),
    }

    report = {
        "metadata": {
            "dataset_name": dataset_name or main_path.parent.parent.name,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "main_dem": str(main_path),
            "reference_dem": str(reference_path),
            "mask_dir": str(mask_root),
            "output_dir": str(output_root),
        },
        "alignment": {
            "mode": alignment_meta["mode"],
            "row_offset": alignment_meta["row_offset"],
            "col_offset": alignment_meta["col_offset"],
            "copy_rows": alignment_meta["copy_rows"],
            "copy_cols": alignment_meta["copy_cols"],
            "main_shape": [main.height, main.width],
            "reference_shape": [reference.height, reference.width],
            "main_resolution": [main.res_x, main.res_y],
            "reference_resolution": [reference.res_x, reference.res_y],
            "main_bounds": _to_bbox_dict(main.bounds),
            "reference_bounds": _to_bbox_dict(reference.bounds),
            "overlap_bounds": alignment_meta["overlap_bounds"],
            "reference_clipped_to_main_grid": alignment_meta["reference_clipped_to_main_grid"],
            "main_crs": None if main.crs is None else main.crs.to_string(),
            "reference_crs": None if reference.crs is None else reference.crs.to_string(),
            "main_nodata": main.nodata,
            "reference_nodata": reference.nodata,
            "main_dtype": main.dtype,
            "reference_dtype": reference.dtype,
        },
        "domain": domain,
        "elevation": elevation,
        "slope": slope_metrics,
        "hotspots": hotspots,
        "interpretation": interpretation,
        "artifacts": {
            "domain_overlap_png": str(domain_overlap_png),
            "diff_signed_png": str(diff_signed_png),
            "diff_abs_png": str(diff_abs_png),
            "slope_diff_png": str(slope_diff_png),
            "hotspots_csv": str(hotspots_csv),
            "summary_md": str(summary_md),
        },
    }

    report_json.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_summary_markdown(summary_md, report)
    return report


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compare project DEM output against a reference DEM")
    parser.add_argument("--main-dem", required=True, help="Path to the main/project DEM")
    parser.add_argument("--reference-dem", required=True, help="Path to the reference DEM")
    parser.add_argument("--mask-dir", required=True, help="Mask directory for the main DEM result")
    parser.add_argument("--output-dir", required=True, help="Comparison output directory")
    parser.add_argument("--dataset-name", help="Dataset name used in the report")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    report = compare_dem_results(
        main_dem_path=args.main_dem,
        reference_dem_path=args.reference_dem,
        mask_dir=args.mask_dir,
        output_dir=args.output_dir,
        dataset_name=args.dataset_name,
    )
    print(json.dumps({"report": str(Path(report["artifacts"]["summary_md"]).with_name("report.json"))}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
