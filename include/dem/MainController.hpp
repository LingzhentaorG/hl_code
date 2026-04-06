#pragma once

#include "dem/CRSManager.hpp"
#include "dem/Config.hpp"
#include "dem/DEMEngine.hpp"
#include "dem/GroundFilterEngine.hpp"
#include "dem/InputManager.hpp"
#include "dem/Logger.hpp"
#include "dem/OutputManager.hpp"
#include "dem/PreprocessEngine.hpp"
#include "dem/TileManager.hpp"

namespace dem {

/**
 * @brief 主控制器，协调整个 DEM 处理流水线
 *
 * 作为整个系统的入口编排器，负责：
 * 1. 加载和解析配置
 * 2. 初始化各子模块
 * 3. 执行完整的处理流程：输入 → 预处理 → 离群点剔除 → 地面滤波 → DEM 插值 → 输出
 * 4. 支持 Tile 分块模式下的并行处理与结果合并
 */
class MainController {
 public:
  /**
   * @brief 运行完整的 DEM 生成流程
   * @param config_path JSON 配置文件路径
   * @param overrides 运行时配置覆盖参数列表
   * @return 程序退出码（0=成功，非0=失败）
   */
  int run(const std::filesystem::path& config_path, const std::vector<std::string>& overrides);

 private:
  /**
   * @brief 处理单个点云的完整 DEM 生成流程
   * @param cloud 输入点云
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @param forced_grid_bounds 强制指定的栅格边界（Tile 模式下使用）
   * @param apply_edge_shrink 是否应用边缘收缩
   * @return 所有处理产物（栅格、分类点云等）
   */
  ProcessingArtifacts processSingleCloud(PointCloud cloud,
                                         const DEMConfig& config,
                                         Logger& logger,
                                         ProcessStats& stats,
                                         const Bounds* forced_grid_bounds,
                                         bool apply_edge_shrink) const;

  /**
   * @brief 在 Tile 分块模式下处理整个区域
   * 将大范围点云切分为多个 Tile 分别处理，再合并结果
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @param summary 输入点云的扫描摘要
   * @return 合并后的全局处理产物
   */
  ProcessingArtifacts processTiles(const DEMConfig& config,
                                   Logger& logger,
                                   ProcessStats& stats,
                                   const PointCloudScanSummary& summary) const;

  ConfigManager config_manager_;        /**< 配置管理器实例 */
  InputManager input_manager_;          /**< 输入管理器实例 */
  PreprocessEngine preprocess_engine_;  /**< 预处理引擎实例 */
  CRSManager crs_manager_;              /**< CRS 管理器实例 */
  GroundFilterEngine ground_filter_engine_;  /**< 地面滤波引擎实例 */
  DEMEngine dem_engine_;                /**< DEM 生成引擎实例 */
  TileManager tile_manager_;            /**< Tile 分块管理器实例 */
  OutputManager output_manager_;        /**< 输出管理器实例 */
};

}  // namespace dem
