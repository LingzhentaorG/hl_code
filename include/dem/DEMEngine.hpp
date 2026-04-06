#pragma once

#include "dem/Logger.hpp"
#include "dem/SpatialIndexManager.hpp"
#include "dem/Types.hpp"

namespace dem {

/**
 * @brief 数字高程模型（DEM）生成引擎
 *
 * 提供栅格创建、最近邻插值、反距离权重（IDW）插值、空洞填补、
 * 有效值掩码构建和边缘收缩等核心功能。
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
   * @param apply_edge_shrink 是否应用边缘收缩
   * @return 初始化后的栅格网格（所有值为 nodata）
   */
  RasterGrid createGrid(const Bounds& bounds, const DEMConfig& config, bool apply_edge_shrink) const;

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
   * @brief 从 accepted support 构建最终 DEM 有效域掩膜
   * @param accepted_support accepted support 栅格
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @param object_mask 对象掩膜（可选）
   * @return 最终共享 DEM 有效域掩膜
   */
  RasterGrid buildDomainMask(const RasterGrid& accepted_support,
                             const DEMConfig& config,
                             Logger& logger,
                             ProcessStats& stats,
                             const RasterGrid* object_mask = nullptr) const;

  /**
   * @brief 使用最近邻方法对 accepted support 在有效域内进行 DEM 插值
   * @param support_grid accepted support 栅格
   * @param domain_mask 最终 DEM 有效域掩膜
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 插值后的 DEM 栅格
   */
  RasterGrid interpolateNearest(const RasterGrid& support_grid,
                                const RasterGrid& domain_mask,
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
   * @brief 对 DEM 栅格中的空洞（无效区域）进行受限填充
   * @param source 源 DEM 栅格
   * @param domain_mask 最终 DEM 有效域掩膜
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 填充后的 DEM 栅格
   */
  RasterGrid fillHoles(const RasterGrid& source,
                       const RasterGrid& domain_mask,
                       const DEMConfig& config,
                       Logger& logger,
                       ProcessStats& stats,
                       const RasterGrid* object_mask = nullptr) const;

  /**
   * @brief 在全局裸地域上构建分析型 DTM
   * @param support_grid accepted support 栅格
   * @param domain_mask 最终裸地域
   * @param object_mask 对象掩膜
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 分析型 DTM
   */
  RasterGrid buildAnalysisDtm(const RasterGrid& support_grid,
                              const RasterGrid& domain_mask,
                              const RasterGrid& object_mask,
                              const DEMConfig& config,
                              Logger& logger,
                              ProcessStats& stats) const;

  /**
   * @brief 在分析型 DTM 基础上生成展示型全域补全和平滑 DTM
   * @param analysis_dtm 分析型 DTM
   * @param domain_mask 最终裸地域
   * @param object_mask 对象掩膜
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 展示型 DTM（内部空洞会被全局重建后再平滑）
   */
  RasterGrid buildDisplayDtm(const RasterGrid& analysis_dtm,
                             const RasterGrid& domain_mask,
                             const RasterGrid& object_mask,
                             const DEMConfig& config,
                             Logger& logger,
                             ProcessStats& stats) const;

  /**
   * @brief 构建结构性边缘掩码（外边界、大孔洞边界、填洞前沿）
   * @param domain_mask 最终 DEM 有效域掩膜
   * @param dem_nearest 最近邻插值 DEM（填洞前）
   * @param dem_nearest_filled 最近邻插值 DEM（填洞后）
   * @param dem_idw IDW 插值 DEM（填洞前）
   * @param dem_idw_filled IDW 插值 DEM（填洞后）
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 边缘掩码栅格
   */
  RasterGrid buildEdgeMask(const RasterGrid& domain_mask,
                           const RasterGrid& dem_nearest,
                           const RasterGrid& dem_nearest_filled,
                           const RasterGrid& dem_idw,
                           const RasterGrid& dem_idw_filled,
                           const DEMConfig& config,
                           Logger& logger,
                           ProcessStats& stats) const;

  /**
   * @brief 构建结构性边缘掩码（外边界、大孔洞边界、对象边界、tile seam、填洞前沿）
   * @param domain_mask 最终裸地域掩膜
   * @param object_mask 对象掩膜
   * @param before_fill 填洞前栅格
   * @param after_fill 填洞后栅格
   * @param seam_mask tile seam 风险掩膜
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 边缘掩码栅格
   */
  RasterGrid buildStructuralEdgeMask(const RasterGrid& domain_mask,
                                     const RasterGrid& object_mask,
                                     const RasterGrid& before_fill,
                                     const RasterGrid& after_fill,
                                     const RasterGrid& seam_mask,
                                     const DEMConfig& config,
                                     Logger& logger,
                                     ProcessStats& stats) const;

  /**
   * @brief 对栅格应用全局边缘收缩，将边缘区域的值设为 nodata
   * @param grid 待处理的栅格（将被修改）
   * @param edge_shrink_cells 收缩的单元格数
   */
  void applyGlobalEdgeShrink(RasterGrid& grid, double edge_shrink_cells) const;

};

}  // namespace dem
