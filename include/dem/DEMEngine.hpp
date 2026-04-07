#pragma once

#include "dem/Logger.hpp"
#include "dem/SpatialIndexManager.hpp"
#include "dem/Types.hpp"

namespace dem {

/**
 * @brief 数字高程模型（DEM）生成引擎
 *
 * 提供栅格创建、官方 IDW 重建、对象掩膜、域掩膜和边界掩膜等核心功能。
 */
class DEMEngine {
 public:
  /**
   * @brief 直接按给定分辨率创建空栅格模板
   * @param bounds 点云边界
   * @param cell_size 目标分辨率
   * @param nodata nodata 值
   * @return 初始化后的栅格模板
   */
  RasterGrid createGrid(const Bounds& bounds, double cell_size, double nodata) const;

  /**
   * @brief 根据点云边界范围和配置参数创建空栅格网格
   * @param bounds 点云的空间范围
   * @param config DEM 配置参数
   * @return 初始化后的栅格网格（所有值为 nodata）
   */
  RasterGrid createGrid(const Bounds& bounds, const DEMConfig& config) const;

  /**
   * @brief 将点云按栅格单元直接落格，记录每个单元的最低高程
   * @param cloud 输入点云
   * @param grid_template 栅格模板（决定输出尺寸和范围）
   * @return 直接落格后的栅格（valid_mask=直接支持像元）
   */
  RasterGrid rasterizeMinimum(const PointCloud& cloud, const RasterGrid& grid_template) const;

  /**
   * @brief 从直接地面支持栅格和原始直落格中生成补支撑后的 accepted support
   * @param raw_direct 原始直落格 DEM
   * @param ground_direct 地面点直落格 DEM
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 可用于插值的 accepted support 栅格
   */
  RasterGrid refineAcceptedSupport(const RasterGrid& raw_direct,
                                   const RasterGrid& ground_direct,
                                   const DEMConfig& config,
                                   Logger& logger,
                                   ProcessStats& stats,
                                   const RasterGrid* object_mask = nullptr) const;

  /**
   * @brief 从支撑栅格构建直接支持掩膜（1=直接支持，0=无直接支持）
   * @param source 支撑栅格
   * @return 二值支持掩膜
   */
  RasterGrid buildSupportMask(const RasterGrid& source) const;

  /**
   * @brief 根据 raw-ground 残差构建建筑/树木等高起伏对象掩膜
   * @param raw_direct 原始直落格 DEM
   * @param ground_direct 地面点直落格 DEM
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 对象掩膜（1=对象候选）
   */
  RasterGrid buildObjectMask(const RasterGrid& raw_direct,
                             const RasterGrid& ground_direct,
                             const DEMConfig& config,
                             Logger& logger,
                             ProcessStats& stats) const;

  /**
   * @brief 从 filtered points 的二维凸包构建最终 DEM 有效域掩膜
   * @param boundary_cloud 作为边界来源的过滤后点云
   * @param grid_template 目标 DEM 栅格模板（决定输出尺寸和范围）
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 最终共享 DEM 插值域掩膜
   */
  RasterGrid buildDomainMask(const PointCloud& boundary_cloud,
                             const RasterGrid& grid_template,
                             const DEMConfig& config,
                             Logger& logger,
                             ProcessStats& stats) const;

  /**
   * @brief 使用反距离权重（IDW）方法对 accepted support 在有效域内进行 DEM 插值
   * @param support_grid accepted support 栅格
   * @param domain_mask 最终 DEM 有效域掩膜
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return IDW 插值后的 DEM 栅格
   */
  RasterGrid interpolateIdw(const RasterGrid& support_grid,
                            const RasterGrid& domain_mask,
                            const DEMConfig& config,
                            Logger& logger,
                            ProcessStats& stats) const;

  /**
   * @brief 从 filtered points 的二维凸包构建边界轮廓掩码
   * @param boundary_cloud 作为边界来源的过滤后点云
   * @param grid_template 目标 DEM 栅格模板（决定输出尺寸和范围）
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 凸包边界轮廓掩码栅格
   */
  RasterGrid buildEdgeMask(const PointCloud& boundary_cloud,
                           const RasterGrid& grid_template,
                           const DEMConfig& config,
                           Logger& logger,
                           ProcessStats& stats) const;
};

}  // namespace dem
