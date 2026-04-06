#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
write_geotiff.py - GeoTIFF and PNG writer
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import rasterio
from matplotlib import colormaps
from PIL import Image
from rasterio.transform import from_origin


def write_png(meta: dict, data: np.ndarray) -> None:
    png_path = meta.get("png_path")
    if not png_path:
        return

    output_path = Path(png_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    mode = meta.get("png_mode", "dem")
    nodata = meta["nodata"]

    if mode == "dem":
        rgba = np.zeros((data.shape[0], data.shape[1], 4), dtype=np.uint8)
        rgba[..., 3] = 255
        valid_mask = np.isfinite(data) & (data != nodata)
        if np.any(valid_mask):
            valid_values = data[valid_mask]
            lo, hi = np.percentile(valid_values, [2, 98])
            if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
                hi = lo + 1.0
            normalized = np.clip((data - lo) / (hi - lo), 0.0, 1.0)
            cmap = colormaps["terrain"]
            rgba[valid_mask] = (cmap(normalized[valid_mask]) * 255).astype(np.uint8)
        Image.fromarray(rgba).save(output_path)
        return

    if mode == "mask":
        active_mask = np.isfinite(data) & (data > 0.5)
        rgba = np.zeros((data.shape[0], data.shape[1], 4), dtype=np.uint8)
        rgba[..., 3] = 255
        rgba[active_mask] = np.array([255, 255, 255, 255], dtype=np.uint8)
        Image.fromarray(rgba).save(output_path)
        return

    active_mask = np.isfinite(data) & (data > 0.5)
    rgba = np.zeros((data.shape[0], data.shape[1], 4), dtype=np.uint8)
    rgba[..., 3] = 255
    if mode == "support_mask":
        rgba[active_mask] = np.array([0, 200, 0, 255], dtype=np.uint8)
    elif mode == "object_mask":
        rgba[active_mask] = np.array([255, 160, 0, 255], dtype=np.uint8)
    else:
        rgba[active_mask] = np.array([220, 40, 40, 255], dtype=np.uint8)
    Image.fromarray(rgba).save(output_path)


def main() -> int:
    parser = argparse.ArgumentParser(description="GeoTIFF and PNG writer")
    parser.add_argument("--meta", required=True, help="Meta JSON path")
    args = parser.parse_args()

    meta_path = Path(args.meta)
    meta = json.loads(meta_path.read_text(encoding="utf-8"))

    data = np.fromfile(meta["bin_path"], dtype=np.float64).reshape((meta["rows"], meta["cols"]))

    profile = {
        "driver": "GTiff",
        "height": meta["rows"],
        "width": meta["cols"],
        "count": 1,
        "dtype": "float64",
        "nodata": meta["nodata"],
        "transform": from_origin(meta["origin_x"], meta["origin_y"], meta["cell_size"], meta["cell_size"]),
        "compress": "deflate",
    }

    crs = None
    if meta.get("crs_wkt"):
        crs = meta["crs_wkt"]
    elif meta.get("epsg_code", 0) > 0:
        crs = f"EPSG:{meta['epsg_code']}"
    if crs:
        profile["crs"] = crs

    output_path = Path(meta["output_path"])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(output_path, "w", **profile) as dataset:
        dataset.write(data, 1)

    write_png(meta, data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
