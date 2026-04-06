#pragma once

#include "dem/InputManager.hpp"
#include "dem/Logger.hpp"
#include "dem/Types.hpp"

namespace dem {

/**
 * @brief Tile 分块构建结果结构体
 *
 * 包含物化后的 Tile 定义列表和临时文件存储目录。
 */
struct TileBuildResult {
  std::vector<TileDefinition> tiles;        /**< 所有 Tile 的定义列表 */
  std::filesystem::path temp_directory;     /**< 临时文件存储目录路径 */
};

/**
 * @brief Tile 分块管理器，负责大范围点云的空间切分与物化
 *
 * 当点云数据量超过内存阈值时，将区域划分为规则网格状的 Tile，
 * 每个 Tile 包含主区域和缓冲区，分别处理后合并为全局结果。
 */
class TileManager {
 public:
  /**
   * @brief 判断是否应该使用 Tile 分块模式
   * 根据配置模式和预估内存占用自动决策
   * @param summary 点云扫描摘要
   * @param config DEM 配置参数
   * @return true 表示应启用 Tile 模式
   */
  bool shouldUseTiles(const PointCloudScanSummary& summary, const DEMConfig& config) const;

  /**
   * @brief 根据空间范围创建 Tile 定义列表（不实际读取点云数据）
   * @param bounds 输入点云的全局空间范围
   * @param config DEM 配置参数
   * @return Tile 定义向量
   */
  std::vector<TileDefinition> createTiles(const Bounds& bounds, const DEMConfig& config) const;

  /**
   * @brief 物化 Tile：流式读取输入点云并按空间位置分配到各 Tile 文件中
   * 将原始 PLY 文件中的点按所属 Tile 写入独立的 .xyz 临时文件
   * @param input_path 输入 PLY 文件路径
   * @param bounds 输入点云的全局空间范围
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 包含已填充点数的 Tile 列表和临时目录
   */
  TileBuildResult materializeTiles(const std::filesystem::path& input_path,
                                   const Bounds& bounds,
                                   const DEMConfig& config,
                                   Logger& logger,
                                   ProcessStats& stats) const;
};

}  // namespace dem
