#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
write_geotiff.py - GeoTIFF 与 PNG 可视化图像写入脚本
================================================

功能概述：
  接收 C++ 程序生成的二进制 DEM 栅格数据和元信息 JSON，
  使用 rasterio 库将其写入标准 GeoTIFF 文件（含地理坐标参考），
  同时生成配套的 PNG 彩色可视化图像。

使用方法：
  python scripts/write_geotiff.py --meta <meta_json_path>

依赖：
  - Python 3.8+
  - numpy: 数组运算
  - rasterio: 地理空间栅格读写（基于 GDAL）
  - matplotlib: terrain 配色方案
  - Pillow (PNG): 图像编码

输入格式：
  meta.json 包含以下关键字段：
    - bin_path: 二进制栅格数据文件路径（float64, 行优先）
    - rows/cols: 栅格行列数
    - nodata: NoData 填充值
    - origin_x/origin_y: 左上角地理坐标
    - cell_size: 像素分辨率（米）
    - crs_wkt 或 epsg_code: 坐标参考系定义
    - output_path: GeoTIFF 输出路径
    - png_path: PNG 可选输出路径
    - png_mode: PNG 模式 ("dem"=地形渲染, "mask"/"edge"=掩码图)
"""

import argparse
import json
from pathlib import Path

import numpy as np
import rasterio
from matplotlib import colormaps
from PIL import Image
from rasterio.transform import from_origin


def write_png(meta: dict, data: np.ndarray) -> None:
    """
    根据 meta 配置将 DEM 栅格数据渲染为 PNG 图像
    
    渲染模式说明：
    - dem 模式：使用 matplotlib 的 terrain 配色方案进行地形渲染，
      高程值按 2%~98% 分位数线性拉伸到 [0,255]，NoData 区域透明
    - mask 模式：有效区域显示为半透明绿色（用于地面点覆盖掩码）
    - edge 模式：有效区域显示为半透明红色（用于边缘掩码）
    
    Args:
        meta: 元信息字典，需包含 png_path、nodata、png_mode 等字段
        data: 二维 float64 栅格数组
    """
    png_path = meta.get("png_path")
    if not png_path:
        return   /* 未指定 PNG 路径时跳过 */

    nodata = meta["nodata"]
    mode = meta.get("png_mode", "dem")   /* 默认使用 DEM 地形模式 */
    output_path = Path(png_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if mode == "dem":
        /* DEM 地形渲染模式 */
        valid_mask = np.isfinite(data) & (data != nodata)   /* 有效像素掩码 */
        rgba = np.zeros((data.shape[0], data.shape[1], 4), dtype=np.uint8)
        
        if np.any(valid_mask):
            /* 计算 2%~98% 分位数范围进行拉伸（剔除极端值影响） */
            valid_values = data[valid_mask]
            lo, hi = np.percentile(valid_values, [2, 98])
            if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
                hi = lo + 1.0   /* 异常情况下的回退策略 */
            
            /* 线性归一化到 [0, 1] 并应用 terrain 配色映射 */
            normalized = np.clip((data - lo) / (hi - lo), 0.0, 1.0)
            cmap = colormaps["terrain"]
            rgba = (cmap(normalized) * 255).astype(np.uint8)
            
            /* NoData 区域设为全透明黑色 */
            rgba[~valid_mask] = np.array([0, 0, 0, 0], dtype=np.uint8)
        
        Image.fromarray(rgba).save(output_path)
        return

    /* 掩码渲染模式（mask / edge） */
    active_mask = np.isfinite(data) & (data > 0.5)   /* 有效阈值判断 */
    rgba = np.zeros((data.shape[0], data.shape[1], 4), dtype=np.uint8)
    if mode == "mask":
        rgba[active_mask] = np.array([0, 200, 0, 220], dtype=np.uint8)   /* 半透明绿色 */
    else:
        rgba[active_mask] = np.array([220, 40, 40, 220], dtype=np.uint8)   /* 半透明红色 */
    Image.fromarray(rgba).save(output_path)


def main() -> int:
    """
    主函数：读取元信息和二进制数据，输出 GeoTIFF 和 PNG 文件
    
    处理流程：
      1. 解析命令行参数获取元信息 JSON 路径
      2. 读取元信息并加载二进制栅格数据
      3. 构建 rasterio 写入 profile（含地理变换和 CRS）
      4. 调用 rasterio 写入 GeoTIFF 文件
      5. 调用 write_png 生成配套可视化图像
    """
    parser = argparse.ArgumentParser(description="GeoTIFF 与 PNG 写入工具")
    parser.add_argument("--meta", required=True, help="元信息 JSON 文件路径")
    args = parser.parse_args()

    # 读取元信息 JSON
    meta_path = Path(args.meta)
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    
    # 从二进制文件加载 float64 栅格数据（行优先存储）
    data = np.fromfile(meta["bin_path"], dtype=np.float64).reshape((meta["rows"], meta["cols"]))

    # 构建 rasterio 写入配置（profile）
    profile = {
        "driver": "GTiff",              /* GeoTIFF 格式驱动 */
        "height": meta["rows"],         /* 栅格行数 */
        "width": meta["cols"],          /* 栅格列数 */
        "count": 1,                     /* 单波段（高程） */
        "dtype": "float64",             /* 64 位浮点精度 */
        "nodata": meta["nodata"],       /* NoData 值声明 */
        "transform": from_origin(       /* 仿射变换矩阵（像素→地理坐标） */
            meta["origin_x"],
            meta["origin_y"],
            meta["cell_size"],
            meta["cell_size"]
        ),
        "compress": "deflate",          /* deflate 压缩以减小文件体积 */
    }

    # 设置坐标参考系（CRS）：优先使用 WKT，其次使用 EPSG 编码
    crs = None
    if meta.get("crs_wkt"):
        crs = meta["crs_wkt"]
    elif meta.get("epsg_code", 0) > 0:
        crs = f"EPSG:{meta['epsg_code']}"

    if crs:
        profile["crs"] = crs

    # 写入 GeoTIFF 文件
    output_path = Path(meta["output_path"])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with rasterio.open(output_path, "w", **profile) as dataset:
        dataset.write(data, 1)   /* 写入第一个（唯一）波段 */

    # 生成配套的 PNG 可视化图像
    write_png(meta, data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
