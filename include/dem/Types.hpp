#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace dem {

/**
 * @brief 三维空间范围（边界框）结构体
 *
 * 表示点云或栅格在三维空间中的最小/最大坐标边界。
 * 用于空间查询、Tile 划分、栅格创建等场景。
 */
struct Bounds {
  double xmin = std::numeric_limits<double>::infinity();   /**< X 轴最小值 */
  double xmax = -std::numeric_limits<double>::infinity();  /**< X 轴最大值 */
  double ymin = std::numeric_limits<double>::infinity();   /**< Y 轴最小值 */
  double ymax = -std::numeric_limits<double>::infinity();  /**< Y 轴最大值 */
  double zmin = std::numeric_limits<double>::infinity();   /**< Z 轴最小值 */
  double zmax = -std::numeric_limits<double>::infinity();  /**< Z 轴最大值 */

  /**
   * @brief 检查边界是否有效（所有坐标均为有限值且 min <= max）
   * @return true 表示边界有效
   */
  [[nodiscard]] bool valid() const noexcept {
    return std::isfinite(xmin) && std::isfinite(xmax) && std::isfinite(ymin) &&
           std::isfinite(ymax) && std::isfinite(zmin) && std::isfinite(zmax) &&
           xmin <= xmax && ymin <= ymax && zmin <= zmax;
  }

  /**
   * @brief 将一个点坐标扩展到当前边界中
   * @param x X 坐标
   * @param y Y 坐标
   * @param z Z 坐标
   */
  void expand(double x, double y, double z) noexcept;

  /**
   * @brief 合并另一个边界到当前边界中
   * @param another 另一个边界对象
   */
  void merge(const Bounds& other) noexcept;

  /** @return 边界的 X 方向宽度 */
  [[nodiscard]] double width() const noexcept { return xmax - xmin; }
  /** @return 边界的 Y 方向高度 */
  [[nodiscard]] double height() const noexcept { return ymax - ymin; }
  /** @return 边界的 Z 方向深度 */
  [[nodiscard]] double depth() const noexcept { return zmax - zmin; }

  /**
   * @brief 判断指定 XY 坐标是否在边界范围内（含边界）
   * @param x X 坐标
   * @param y Y 坐标
   * @return true 表示在范围内
   */
  [[nodiscard]] bool containsXY(double x, double y) const noexcept;

  /**
   * @brief 判断两个边界在 XY 平面上是否相交
   * @param another 另一个边界
   * @return true 表示存在交集
   */
  [[nodiscard]] bool intersectsXY(const Bounds& other) const noexcept;

  /**
   * @brief 返回各方向扩展指定边距后的新边界副本
   * @param margin 扩展边距
   * @return 新的边界对象
   */
  [[nodiscard]] Bounds expanded(double margin) const noexcept;
};

/**
 * @brief 三维点结构体，是点云处理的基本数据单元
 *
 * 除基本坐标外，还携带滤波过程中产生的各种属性标记和中间计算结果。
 */
struct Point3D {
  double x = 0.0;                          /**< X 坐标 */
  double y = 0.0;                          /**< Y 坐标 */
  double z = 0.0;                          /**< Z 坐标（高程值） */
  std::uint32_t index = 0;                 /**< 点的全局索引号 */
  bool valid = true;                       /**< 是否为有效点（非 NaN/Inf） */
  bool outlier = false;                    /**< 是否被判定为离群点 */
  bool ground = false;                     /**< 是否被分类为地面点 */
  bool edge_point = false;                 /**< 是否位于数据边缘区域 */
  double local_mean_dist_3d = 0.0;         /**< 局部 3D 平均邻域距离（用于离群点检测） */
  double local_height_diff = 0.0;          /**< 相对参考高度的高程差 */
  double local_slope = 0.0;                /**< 局部坡度（度） */
  double reference_z = 0.0;               /**< 地面参考高程 */
  double neighborhood_completeness = 0.0; /**< 邻域完整度（8 扇区占用率） */
  std::optional<std::uint8_t> red;         /**< 红色通道（可选） */
  std::optional<std::uint8_t> green;       /**< 绿色通道（可选） */
  std::optional<std::uint8_t> blue;        /**< 蓝色通道（可选） */
  std::optional<int> classification;      /**< 分类标签（可选，来自 PLY） */
};

/**
 * @brief 点云数据结构体
 *
 * 包含完整的点集合、统计信息和元数据。
 * 在 Tile 模式下支持通过暂存文件（staging_xyz_path）避免全量内存驻留。
 */
struct PointCloud {
  std::vector<Point3D> points;             /**< 点集合向量 */
  std::size_t raw_point_count = 0;        /**< 原始总点数（来自文件头） */
  std::size_t stored_point_count = 0;     /**< 当前存储的点数（可能小于原始数） */
  Bounds bounds;                           /**< 当前有效点的空间边界 */
  bool has_2d_index = false;              /**< 是否已构建 2D 空间索引 */
  bool has_3d_index = false;              /**< 是否已构建 3D 空间索引 */
  double estimated_avg_spacing = 0.0;     /**< 估算的平均点间距 */
  double estimated_density = 0.0;         /**< 估算的点密度（点/单位面积） */
  std::vector<std::string> property_names; /**< PLY 文件中的属性名称列表 */
  std::filesystem::path staging_xyz_path; /**< Tile 模式下的暂存 XYZ 文件路径 */
};

/**
 * @brief 栅格网格结构体，表示 DEM 的二维规则格网数据
 *
 * 采用左上角原点、行优先存储方式，
 * 配套 valid_mask 和 edge_mask 用于标识每个单元格的状态。
 */
struct RasterGrid {
  std::size_t rows = 0;                   /**< 行数 */
  std::size_t cols = 0;                   /**< 列数 */
  double cell_size = 0.0;                 /**< 单元格大小（分辨率） */
  double origin_x = 0.0;                  /**< 左上角原点 X 坐标 */
  double origin_y = 0.0;                  /**< 左上角原点 Y 坐标 */
  double nodata = -9999.0;                /**< 无数据标记值 */
  std::vector<double> values;             /**< 高程值数组（row-major 顺序） */
  std::vector<std::uint8_t> valid_mask;   /**< 有效值掩码（1=有有效高程值） */
  std::vector<std::uint8_t> edge_mask;    /**< 边缘掩码（1=边缘区域） */

  /** @return 栅格总单元数（rows * cols） */
  [[nodiscard]] std::size_t size() const noexcept { return rows * cols; }
  /** @return 栅格是否为空（无行列） */
  [[nodiscard]] bool empty() const noexcept { return rows == 0 || cols == 0; }
  /**
   * @brief 计算指定行列的一维数组偏移量
   * @param row 行号
   * @param col 列号
   * @return 一维偏移索引
   */
  [[nodiscard]] std::size_t offset(std::size_t row, std::size_t col) const noexcept {
    return row * cols + col;
  }
  /**
   * @brief 计算指定列号的单元中心 X 坐标
   * @param col 列号
   * @return 中心 X 坐标
   */
  [[nodiscard]] double cellCenterX(std::size_t col) const noexcept {
    return origin_x + (static_cast<double>(col) + 0.5) * cell_size;
  }
  /**
   * @brief 计算指定行号的单元中心 Y 坐标
   * @param row 行号
   * @return 中心 Y 坐标（Y 轴向下递减）
   */
  [[nodiscard]] double cellCenterY(std::size_t row) const noexcept {
    return origin_y - (static_cast<double>(row) + 0.5) * cell_size;
  }
};

/**
 * @brief 坐标参考系（CRS）定义结构体
 *
 * 支持多种 CRS 定义方式的统一表达，
 * 解析后生成 resolved_wkt 用于 GeoTIFF 写入。
 */
struct CRSDefinition {
  int epsg_code = 0;                       /**< EPSG 编码（如 4326=WGS84 经纬度） */
  std::string crs_wkt;                    /**< WKT 格式的 CRS 字符串 */
  std::filesystem::path crs_wkt_file;      /**< WKT 文件路径（从文件加载 WKT） */
  bool allow_unknown_crs = false;         /**< 是否允许未知 CRS 继续处理 */
  bool known = false;                      /**< 解析后是否确认为已知 CRS */
  std::string resolved_wkt;               /**< 解析后的完整 WKT 字符串 */
  std::string authority_name;              /**< CRS 权威机构名称（如 "EPSG:4326"） */
};

/**
 * @brief Tile 分块定义结构体
 *
 * 每个 Tile 由主区域（main_bounds）和缓冲区域（buffered_bounds）组成，
 * 缓冲区确保 Tile 边界处的插值质量不受截断影响。
 */
struct TileDefinition {
  std::size_t id = 0;                      /**< Tile 全局唯一 ID */
  std::size_t row = 0;                     /**< Tile 在网格中的行号 */
  std::size_t col = 0;                     /**< Tile 在网格中的列号 */
  Bounds main_bounds;                      /**< 主区域边界（不含缓冲区） */
  Bounds buffered_bounds;                  /**< 缓冲区域边界（含缓冲区） */
  std::filesystem::path temp_points_file;  /**< 该 Tile 的临时点云文件路径 */
  std::size_t point_count = 0;             /**< 该 Tile 内的点数量 */
};

/**
 * @brief 处理过程统计信息收集器
 *
 * 收集整个 DEM 生成流程中的各类计数、数值和耗时指标，
 * 最终写入 stats.txt 文件供用户分析处理效果。
 */
struct ProcessStats {
  std::unordered_map<std::string, double> durations_seconds;           /**< 各阶段耗时（秒） */
  std::unordered_map<std::string, double> values;                      /**< 各类数值型指标 */
  std::unordered_map<std::string, std::size_t> counts;                 /**< 各类计数型指标 */
  std::vector<std::size_t> ground_added_per_iteration;                /**< 每次迭代新增地面点数 */
  std::unordered_map<std::string, std::size_t> reference_mode_counts;  /**< 各参考模式使用次数 */
  bool tile_mode_used = false;                                         /**< 是否使用了 Tile 模式 */

  /** 累加计数 */
  void addCount(const std::string& key, std::size_t value);
  /** 设置计数（覆盖已有值） */
  void setCount(const std::string& key, std::size_t value);
  /** 获取计数 */
  [[nodiscard]] std::size_t getCount(const std::string& key) const;
  /** 设置数值 */
  void setValue(const std::string& key, double value);
  /** 获取数值 */
  [[nodiscard]] double getValue(const std::string& key) const;
  /** 累加耗时 */
  void addDuration(const std::string& key, double seconds);
};

/**
 * @brief 空间查询结果结构体
 *
 * KD-Tree 查询返回的结果集，包含匹配点的原始索引和欧氏距离。
 */
struct QueryResult {
  std::vector<std::size_t> indices;     /**< 匹配点在原始点集中的索引列表 */
  std::vector<double> distances;        /**< 对应的欧氏距离列表 */
};

/** 地面点分类时的参考模式枚举 */
enum class ReferenceMode {
  LocalPlane,                /**< 局部平面拟合模式：用邻域地面点拟合平面作为参考面 */
  InverseDistanceSquared,    /**< 反距离平方加权模式：以 IDW 加权平均作为参考高程 */
  LocalMin                   /**< 局部最低点模式：取邻域内最低 Z 值作为参考 */
};

/** Tile 分块模式枚举 */
enum class TileMode {
  Auto,                      /**< 自动模式：根据预估内存自动决定是否分块 */
  Disabled,                  /**< 禁用模式：始终不分块，强制单次处理 */
  Forced                     /**< 强制模式：始终启用分块，无论数据量大小 */
};

/** ========== 以下为各模块配置子结构体 ========== */

/** 输入配置 */
struct InputConfig {
  std::filesystem::path file_path;        /**< 输入 PLY 文件路径 */
};

/** 预处理配置 */
struct PreprocessConfig {
  std::size_t sample_spacing_count = 1024;    /**< 估算点间距时的采样数量 */
  std::size_t max_extreme_sample = 50000;     /**< 极端离群点检测的最大采样数 */
};

/** 离群点剔除配置 */
struct OutlierConfig {
  std::size_t knn = 16;                        /**< KNN 统计中的近邻数 K */
  double stddev_multiplier = 1.5;              /**< 标准差倍数阈值系数 */
  bool use_robust_stat = false;                /**< 是否使用鲁棒统计（MAD 替代标准差） */
};

/** 地面滤波配置 */
struct GroundConfig {
  double seed_grid_size = 0.0;                 /**< 种子选取的粗格网尺寸（0=自动按 cell_size*3 计算） */
  std::size_t knn = 8;                         /**< 迭代扩张时使用的近邻数 K */
  double search_radius = 0.0;                  /**< 邻域搜索半径（0=自动按 cell_size*3 计算） */
  double max_height_diff = 1.0;                /**< 最大允许高度差阈值（米） */
  double max_slope_deg = 20.0;                 /**< 最大允许坡度阈值（度） */
  std::size_t max_iterations = 6;              /**< 最大迭代次数 */
  std::size_t min_ground_neighbors = 3;        /**< 判定为地面的最少地面近邻数 */
  ReferenceMode reference_mode = ReferenceMode::LocalPlane;          /**< 首选参考模式 */
  ReferenceMode reference_fallback_mode = ReferenceMode::InverseDistanceSquared;  /**< 回退参考模式 */
  double distance_weight_power = 2.0;          /**< IDW 距离权重幂指数 */
  double min_neighbor_completeness = 0.5;      /**< 最小邻域完整度阈值 */
};

/** DEM 生成配置 */
struct DemGenerationConfig {
  double cell_size = 1.0;                       /**< 栅格单元大小 / 分辨率（米） */
  double nearest_max_distance = 0.0;            /**< 最近邻插值的最大搜索距离（0=自动） */
  double idw_radius = 0.0;                      /**< IDW 插值的搜索半径（0=自动） */
  std::size_t idw_max_points = 12;              /**< IDW 插值最大使用点数 */
  std::size_t idw_min_points = 3;               /**< IDW 插值最少需要点数 */
  double idw_power = 2.0;                       /**< IDW 距离权重幂指数 */
  double nodata = -9999.0;                      /**< 无数据标记值 */
  double edge_shrink_cells = 1.0;               /**< 边缘收缩的单元格数 */
  bool enable_edge_mask = true;                 /**< 是否启用边缘掩码 */
  bool fill_holes = true;                       /**< 是否填充空洞 */
  int fill_max_radius = 2;                      /**< 空洞填充最大搜索半径（单元格） */
  std::size_t fill_min_neighbors = 4;           /**< 空洞填充最少邻居数 */
};

/** Tile 分块配置 */
struct TileConfig {
  TileMode mode = TileMode::Auto;               /**< 分块模式选择 */
  std::size_t tile_size_cells = 1000;           /**< 每个 Tile 的单元格数（约 1000x1000） */
  double tile_buffer = 0.0;                     /**< Tile 缓冲区宽度（0=自动计算） */
  std::size_t memory_limit_mb = 1024;           /**< 触发自动分块的内存阈值（MB） */
};

/** 输出配置 */
struct OutputConfig {
  std::filesystem::path directory;              /**< 输出根目录路径 */
  bool overwrite = true;                        /**< 是否覆盖已存在的输出目录 */
  bool write_png = true;                        /**< 是否同时生成 PNG 可视化图像 */
  bool timestamp_subdir = true;                 /**< 是否在输出目录下创建时间戳子目录 */
};

/**
 * @brief 完整的 DEM 处理配置结构体
 *
 * 所有子模块配置的聚合根对象，由 ConfigManager 从 JSON 文件解析并校验。
 */
struct DEMConfig {
  InputConfig input;                            /**< 输入配置 */
  CRSDefinition crs;                            /**< 坐标参考系配置 */
  PreprocessConfig preprocess;                  /**< 预处理配置 */
  OutlierConfig outlier;                        /**< 离群点剔除配置 */
  GroundConfig ground;                          /**< 地面滤波配置 */
  DemGenerationConfig dem;                      /**< DEM 生成配置 */
  TileConfig tile;                              /**< Tile 分块配置 */
  OutputConfig output;                          /**< 输出配置 */
};

/**
 * @brief 平面拟合结果结构体
 *
 * 最小二乘法平面拟合（z = a*x + b*y + c）的计算结果，
 * 用于地面点分类中的局部平面参考模式。
 */
struct PlaneFitResult {
  bool success = false;                        /**< 拟合是否成功 */
  double a = 0.0;                              /**< 平面方程 X 方向系数 */
  double b = 0.0;                              /**< 平面方程 Y 方向系数 */
  double c = 0.0;                              /**< 平面方程常数项 */
  double slope_deg = 0.0;                      /**< 平面坡角（度） */
  double condition_indicator = 0.0;            /**< 条件数指示器（越大越不稳定） */
};

/**
 * @brief 处理产物集合结构体
 *
 * 包含 DEM 生成流水线产生的所有中间和最终产物：
 * - 分类后的四类点云（过滤点、种子点、地面点、非地面点）
 * - 多种方法的 DEM 栅格（最近邻、IDW、及其空洞填补版本）
 * - 有效值掩码和边缘掩码
 */
struct ProcessingArtifacts {
  PointCloud filtered_points;                   /**< 离群点过滤后的点云 */
  PointCloud seed_points;                      /**< 种子点云 */
  PointCloud ground_points;                    /**< 最终地面点云 */
  PointCloud nonground_points;                 /**< 非地面点云 */
  RasterGrid dem_nearest;                      /**< 最近邻插值 DEM */
  RasterGrid dem_idw;                           /**< IDW 插值 DEM */
  RasterGrid dem_nearest_filled;               /**< 最近邻插值 DEM（空洞已填充） */
  RasterGrid dem_idw_filled;                   /**< IDW 插值 DEM（空洞已填充） */
  RasterGrid dem_mask;                          /**< 有效值掩码栅格 */
  RasterGrid dem_edge_mask;                     /**< 边缘掩码栅格 */
};

}  // namespace dem
