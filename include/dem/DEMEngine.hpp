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
   * @brief 根据点云边界范围和配置参数创建空栅格网格
   * @param bounds 点云的空间范围
   * @param config DEM 配置参数
   * @param apply_edge_shrink 是否应用边缘收缩
   * @return 初始化后的栅格网格（所有值为 nodata）
   */
  RasterGrid createGrid(const Bounds& bounds, const DEMConfig& config, bool apply_edge_shrink) const;

  /**
   * @brief 使用最近邻方法对地面点云进行 DEM 插值
   * @param ground_cloud 地面点云数据
   * @param grid_template 栅格模板（决定输出尺寸和范围）
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 插值后的 DEM 栅格
   */
  RasterGrid interpolateNearest(const PointCloud& ground_cloud,
                                const RasterGrid& grid_template,
                                const DEMConfig& config,
                                Logger& logger,
                                ProcessStats& stats) const;

  /**
   * @brief 使用反距离权重（IDW）方法对地面点云进行 DEM 插值
   * @param ground_cloud 地面点云数据
   * @param grid_template 栅格模板
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return IDW 插值后的 DEM 栅格
   */
  RasterGrid interpolateIdw(const PointCloud& ground_cloud,
                            const RasterGrid& grid_template,
                            const DEMConfig& config,
                            Logger& logger,
                            ProcessStats& stats) const;

  /**
   * @brief 对 DEM 栅格中的空洞（无效区域）进行填充
   * 使用邻域有效值的均值进行插值填充
   * @param source 源 DEM 栅格
   * @param edge_mask 边缘掩码栅格
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 填充后的 DEM 栅格
   */
  RasterGrid fillHoles(const RasterGrid& source,
                       const RasterGrid& edge_mask,
                       const DEMConfig& config,
                       Logger& logger,
                       ProcessStats& stats) const;

  /**
   * @brief 从源栅格构建有效值掩码（1=有效，0=无效）
   * @param source 源 DEM 栅格
   * @return 有效值掩码栅格
   */
  RasterGrid buildValidMask(const RasterGrid& source) const;

  /**
   * @brief 从源栅格构建边缘掩码（1=边缘，0=非边缘）
   * @param source 源 DEM 栅格
   * @return 边缘掩码栅格
   */
  RasterGrid buildEdgeMask(const RasterGrid& source) const;

  /**
   * @brief 对栅格应用全局边缘收缩，将边缘区域的值设为 nodata
   * @param grid 待处理的栅格（将被修改）
   * @param edge_shrink_cells 收缩的单元格数
   */
  void applyGlobalEdgeShrink(RasterGrid& grid, double edge_shrink_cells) const;

 private:
  /**
   * @brief 计算指定栅格单元位置的邻域完整度（8 方向扇区占用率）
   * @param index 空间索引管理器
   * @param ground_cloud 地面点云
   * @param x 栅格单元中心 X 坐标
   * @param y 栅格单元中心 Y 坐标
   * @param radius 搜索半径
   * @param config DEM 配置参数
   * @return 完整度比值 [0.0, 1.0]
   */
  double computeCompletenessForCell(const SpatialIndexManager& index,
                                    const PointCloud& ground_cloud,
                                    double x,
                                    double y,
                                    double radius,
                                    const DEMConfig& config) const;
};

}  // namespace dem
