# DEM CLI

基于 **C++20 + CMake + MinGW g++** 的命令行 DEM（数字高程模型）生成程序，专门面向**光学测绘卫星摄影测量点云**数据处理。从输入的 ASCII PLY 格式点云中提取地面点，通过插值算法生成规则格网的高程栅格数据（DEM/DTM）。

## 功能概览

| 阶段 | 功能 |
|------|------|
| **输入** | ASCII PLY 点云读取与验证（支持 XYZ + 可选强度/颜色/分类属性） |
| **预处理** | NaN/Inf 清洗 → 去重 → 极端离群点剔除（分位数统计） |
| **离群点过滤** | 基于 KNN 统计的 3D 邻域离群点检测（SOR） |
| **地面滤波** | 粗格网种子选取 → 迭代式种子扩张（多参考模式：局部平面 / IDW / 局部最低点）→ 8 扇区邻域完整性校验 |
| **DEM 插值** | 最近邻 / IDW（反距离加权）双模式，支持直落格、支撑栅格构建、有效域掩膜生成 |
| **空洞填充** | 受限邻域均值填充 NoData 区域 |
| **DTM 产物** | 分析型裸地 DTM + 展示型全域补全平滑 DTM + 建筑/树木对象掩膜 |
| **边缘处理** | 结构性边缘掩码（外边界/孔洞边界/对象边界/Tile 接缝）+ 全局边缘收缩 |
| **分块处理** | Tile 模式自动切分大数据集（含缓冲区，内存感知） |
| **输出** | GeoTIFF（Python rasterio 桥接）、PNG 可视化、PLY 中间点云、统计报告 |

## 技术栈

- **语言**: C++20（强制要求）
- **构建**: CMake 3.24+
- **编译器**: MinGW g++（Windows）/ GCC / Clang（Linux）
- **空间索引**: [nanoflann](https://github.com/jlblancoc/nanoflann) v1.5.5（KD-Tree）
- **JSON 解析**: [nlohmann/json](https://github.com/nlohmann/json) v3.11.3
- **GeoTIFF 后端**: [rasterio](https://rasterio.readthedocs.io/)（Python 脚本桥接调用）
- **测试框架**: [doctest](https://github.com/doctest/doctest) v2.4.12
- **并行加速**: OpenMP（可选）

## 快速开始

### 环境准备

```powershell
# 1. 安装 MinGW-w64 编译工具链（推荐 MSYS2）
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake

# 2. 确保 g++ 支持 C++20
g++ --version   # 应显示版本 ≥ 11.0

# 3. 安装 Python 3.8+ 及依赖（GeoTIFF 输出需要）
pip install numpy rasterio matplotlib Pillow
```

### 构建与运行

```powershell
# 构建
cmake -S . -B build -G "MinGW Makefiles"
cmake --build build -j 4

# 使用默认配置运行
.\build\dem_cli.exe --config .\config\example_config.json

# 运行时覆盖参数（无需修改配置文件）
.\build\dem_cli.exe --config config.json --set dem.cell_size=2.0 --set ground.max_height_diff=0.5
```

### 批量处理

```powershell
python .\scripts\batch_process.py --cli .\build\dem_cli.exe --config .\config\example_config.json --input-dir .\data --output-root .\output --jobs 4
```

## 项目结构

```
hl_code/
├── CMakeLists.txt                 # 构建配置
├── config/
│   └── example_config.json        # 示例配置文件
├── data/                          # 输入 PLY 点云数据
├── include/dem/                   # 头文件
│   ├── Types.hpp                  # 核心类型定义（Point3D, RasterGrid, Bounds, DEMConfig 等）
│   ├── MainController.hpp         # 主控制器（流水线编排）
│   ├── InputManager.hpp           # PLY 文件读取与验证
│   ├── PreprocessEngine.hpp       # 预处理引擎
│   ├── GroundFilterEngine.hpp     # 地面滤波引擎（离群点剔除 + 地面提取）
│   ├── DEMEngine.hpp              # DEM 插值引擎（栅格创建/插值/填洞/掩膜）
│   ├── SpatialIndexManager.hpp    # KD-Tree 空间索引管理
│   ├── TileManager.hpp            # 分块处理管理器
│   ├── CRSManager.hpp             # 坐标参考系管理
│   ├── Config.hpp                 # 配置加载与校验
│   ├── OutputManager.hpp          # 结果输出管理
│   ├── Logger.hpp                 # 日志记录器
│   └── Utils.hpp                  # 工具函数
├── src/                           # 实现文件（与头文件一一对应）
├── scripts/
│   ├── batch_process.py           # 批量处理脚本
│   └── write_geotiff.py           # GeoTIFF/PNG 写入脚本（rasterio 桥接）
└── tests/                         # 单元测试
```

## 处理流水线

```
ASCII PLY 输入
    │
    ▼
┌──────────────┐
│  输入验证     │  InputManager：格式检查、属性解析、扫描摘要
└──────────────┘
    │
    ▼
┌──────────────┐
│  CRS 解析     │  CRSManager：EPSG/WKT 坐标系确认
└──────────────┘
    │
    ▼
┌──────────────┐
│  预处理       │  PreprocessEngine：NaN清洗 → 去重 → 极端值剔除 → 密度估算
└──────────────┘
    │
    ▼
┌──────────────┐
│  KNN 离群点   │  GroundFilterEngine::removeOutliers：3D SOR 统计过滤
└──────────────┘
    │
    ▼
┌──────────────┐
│  地面滤波     │  GroundFilterEngine::extractGround：
│              │  ① 粗格网种子选取（每格最低Z）
│              │  ② 迭代式地面点扩张（多参考模式）
│              │  ③ 8扇区邻域完整性判定
└──────────────┘
    │
    ├─── 地面点 ──→ 直落格 (ground_direct)
    ├─── 非地面点 ──→ nonground_points.ply
    │
    ▼
┌──────────────┐
│  支撑栅格构建  │  DEMEngine：
│              │  refineAcceptedSupport → buildDomainMask → buildObjectMask
└──────────────┘
    │
    ▼
┌──────────────┐
│  DEM 插值     │  interpolateNearest / interpolateIdw
└──────────────┘
    │
    ▼
┌──────────────┐
│  空洞填充     │  fillHoles：受限邻域均值填充
└──────────────┘
    │
    ▼
┌──────────────┐
│  DTM 产物     │  buildAnalysisDtm + buildDisplayDtm
└──────────────┘
    │
    ▼
┌──────────────┐
│  边缘掩码     │  buildEdgeMask / buildStructuralEdgeMask
└──────────────┘
    │
    ▼
┌──────────────┐
│  结果输出     │  OutputManager：PLY + GeoTIFF(.tif+.png) + 统计报告
└──────────────┘
```

## 输出目录结构

```
output/<timestamp>/
├── dem/
│   ├── dem_raw_direct.tif (+ .png)       # 预处理后直接落格 DEM
│   ├── dem_ground_direct.tif (+ .png)    # 地面点直接落格 DEM
│   ├── dem_nearest.tif (+ .png)          # 最近邻插值 DEM（填洞前）
│   ├── dem_idw.tif (+ .png)              # IDW 插值 DEM（填洞前）
│   ├── dem_nearest_filled.tif (+ .png)   # 最近邻插值 DEM（已填洞）
│   ├── dem_idw_filled.tif (+ .png)       # IDW 插值 DEM（已填洞）
│   ├── dem_support_mask.tif (+ .png)     # 直接支撑掩膜
│   ├── dem_mask.tif (+ .png)             # 最终 DEM 有效域掩膜
│   ├── dem_edge_mask.tif (+ .png)        # 结构性边缘掩码
│   ├── dtm_analysis.tif (+ .png)         # 分析型裸地 DTM
│   ├── dtm_display.tif (+ .png)          # 展示型全域补全平滑 DTM
│   └── dtm_object_mask.tif (+ .png)      # 建筑/树木对象掩膜
├── pointcloud/
│   ├── filtered_points.ply               # 离群点过滤后的点云
│   ├── seed_points.ply                   # 种子点云
│   ├── ground_points.ply                 # 最终地面点云
│   └── nonground_points.ply              # 非地面点云
├── log/
│   ├── run_log.txt                       # 运行日志
│   ├── config_used.txt                   # 实际使用的完整配置
│   ├── stats.txt                         # 详细统计信息
│   └── report.json                       # JSON 格式处理报告
└── tile/                                 # （Tile 模式时）
    ├── tile_0.xyz ~ tile_N.xyz           # 各 Tile 的暂存点云
    └── aggregate/*.xyz                   # 汇总点云
```

## 配置说明

配置文件采用 JSON 格式，顶层分为以下分组：

| 分组 | 说明 |
|------|------|
| `input` | 输入 PLY 文件路径 |
| `crs` | 坐标参考系（EPSG 编码等） |
| `preprocess` | 预处理参数（采样数、极端值采样上限） |
| `outlier` | KNN 离群点过滤参数（K 值、标准差倍数、鲁棒统计开关） |
| `ground` | ⭐ 地面滤波核心参数（种子格网、迭代次数、高度差阈值、坡度阈值、参考模式等） |
| `dem` | DEM 生成参数（分辨率、插值方法、搜索半径、空洞填充、边缘收缩） |
| `tile` | 分块处理参数（模式、Tile 尺寸、缓冲区宽度、内存阈值） |
| `output` | 输出控制（目录、PNG 开关、时间戳子目录） |

完整配置项说明及调参建议参见 [使用说明文档](./使用说明文档.md)，示例配置见 [config/example_config.json](./config/example_config.json)。

## 构建选项

```powershell
# 启用/禁用 OpenMP 并行加速
cmake -S . -B build -DDEM_ENABLE_OPENMP=ON/OFF

# 启用/禁用单元测试
cmake -S . -B build -DDEM_ENABLE_TESTS=ON/OFF

# 选择 GeoTIFF 后端（AUTO/PYTHON/NATIVE）
cmake -S . -B build -DDEM_GTIFF_BACKEND=AUTO
```

## 已知限制

- 仅支持 **ASCII PLY** 格式输入（二进制 PLY 尚未实现）
- GeoTIFF 写入依赖 Python 环境中的 `rasterio`（原生 libtiff + PROJ 后端尚未接入）
- Tile 模式通过 `output/tile/*.xyz` 暂存各分块点云，避免最终汇总阶段全量驻留内存

## 许可证

本项目仅供学习与研究使用。
