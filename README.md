# DEM CLI

基于 `C++20 + CMake + MinGW g++` 的命令行 DEM 生成程序，面向光学测绘卫星摄影测量点云。

## 当前实现范围

- 输入：仅支持 `ASCII PLY`
- 处理链路：输入检查、预处理、3D 邻域离群点剔除、粗格网种子、迭代地面点扩张、最近邻/IDW DEM、空洞填补、Tile 分块模式
- 输出：GeoTIFF、PNG、PLY、中间统计、配置回写、日志
- GeoTIFF 后端：默认通过仓库内 Python 脚本调用 `rasterio` 写出

## 构建

```powershell
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build -j 4
```

## 运行

```powershell
.\build\dem_cli.exe --config .\config\example_config.json
```

支持运行时覆盖配置：

```powershell
.\build\dem_cli.exe --config .\config\example_config.json --set dem.cell_size=2.0 --set tile.mode=forced
```

批处理 `data` 目录全部 PLY：

```powershell
python .\scripts\batch_process.py --cli .\build\dem_cli.exe --config .\config\example_config.json --input-dir .\data --output-root .\output
```

## 输出结构

- `output/<timestamp>/dem/`
- `output/<timestamp>/pointcloud/`
- `output/<timestamp>/log/`
- `output/<timestamp>/tile/`

`pointcloud/` 下会包含：

- `filtered_points.ply`
- `seed_points.ply`
- `ground_points.ply`
- `nonground_points.ply`

`dem/` 下每个栅格结果同时产出 `.tif` 和 `.png`。

## 配置说明

顶层分组固定为：

- `input`
- `crs`
- `preprocess`
- `outlier`
- `ground`
- `dem`
- `tile`
- `output`
  - `write_png`
  - `timestamp_subdir`

参考示例见 [config/example_config.json](/D:/Desktop/hl_code/config/example_config.json)。

## 已知限制

- 二进制 PLY 尚未实现
- 原生 `libtiff + libgeotiff + PROJ` GeoTIFF 后端尚未接入，当前默认后端依赖 Python 环境中的 `rasterio`
- Tile 模式当前通过 `output/tile/aggregate/*.xyz` 暂存汇总点云，避免在最终汇总阶段一次性保留全部结果点于内存
