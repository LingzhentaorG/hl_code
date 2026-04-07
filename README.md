# DEM 官方产线工具

面向光学测绘卫星摄影测量点云的 DEM 生成与结果对比工具。当前仓库已经收敛为一套可复跑的正式产品：单一官方 DEM 产线、稳定的批处理目录合同、资源三号对比分析模块，以及一键清理重跑的发布脚本。

## 功能范围

- 从 ASCII PLY 点云生成唯一官方 `DEM`
- 输出正式产物：`surface/`、`mask/`、`log/`
- 支持大数据集 Tile 分块处理
- 支持批处理汇总 `summary.json/csv`
- 支持主结果 vs ArcGIS DEM 对比，输出 `comparison/`、`comparison_summary.json/csv`
- 支持按官方清单重跑并固化到 `output/final_release/`

## 环境依赖

- C++20
- CMake 3.24+
- Python 3.8+
- Python 包：`numpy`、`rasterio`、`matplotlib`、`Pillow`

## 快速开始

构建：

```powershell
cmake -S . -B build
cmake --build build -j 4
```

单数据集运行：

```powershell
.\build\dem_cli.exe --config .\config\example_config.json
```

官方批处理：

```powershell
python .\scripts\batch_process.py `
  --cli .\build\dem_cli.exe `
  --config .\config\example_config.json `
  --input-dir .\data `
  --output-root .\output `
  --manifest .\config\official_dataset_manifest.json `
  --batch-name final_release `
  --compare-map .\config\reference_dem_map.json `
  --jobs 1
```

正式重跑：

```powershell
python .\scripts\release_product.py
```

## 正式输出合同

```text
output/final_release/
├── summary.json
├── summary.csv
├── comparison_summary.json
├── comparison_summary.csv
└── datasets/
    └── <dataset>/
        ├── surface/
        │   ├── dem.tif / dem.png
        │   └── dem_raw.tif / dem_raw.png
        ├── mask/
        │   ├── dem_domain_mask.tif / .png
        │   ├── dem_boundary_mask.tif / .png
        │   └── dem_object_mask.tif / .png
        ├── log/
        │   ├── run_log.txt
        │   ├── config_used.json
        │   ├── stats.txt
        │   └── report.json
        └── comparison/                  # 仅存在于启用 compare-map 的数据集
            ├── report.json
            ├── summary.md
            ├── domain_overlap.png
            ├── diff_signed.png
            ├── diff_abs.png
            ├── slope_diff.png
            └── hotspots.csv
```

## 官方配置文件

- [example_config.json](./config/example_config.json)
- [official_dataset_manifest.json](./config/official_dataset_manifest.json)
- [reference_dem_map.json](./config/reference_dem_map.json)

## 文档

- [使用手册](./docs/使用手册.md)
- [架构说明](./docs/架构说明.md)
