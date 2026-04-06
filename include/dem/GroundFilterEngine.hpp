#pragma once

#include "dem/Logger.hpp"
#include "dem/Types.hpp"

namespace dem {

/**
 * @brief 地面滤波引擎，负责离群点剔除和地面点提取
 *
 * 实现基于统计的 3D 邻域离群点检测算法和迭代式地面点扩张算法，
 * 通过粗格网种子点选取和多参考模式分类来分离地面点和非地面点。
 */
class GroundFilterEngine {
 public:
  /**
   * @brief 基于 KNN 统计方法移除 3D 空间中的离群点
   * 计算每个点到其 K 近邻的平均距离，使用标准差阈值判定离群点
   * @param cloud 输入点云
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 过滤后的点云（不含离群点）
   */
  PointCloud removeOutliers(PointCloud cloud, const DEMConfig& config, Logger& logger, ProcessStats& stats) const;

  /**
   * @brief 从过滤后的点云中提取地面点
   * 流程：粗格网种子选取 → 迭代地面点扩张 → 最终分类
   * @param filtered_cloud 输入过滤后的点云（将被修改，标记 ground 属性）
   * @param seed_cloud [输出] 种子点云
   * @param ground_cloud [输出] 最终地面点云
   * @param nonground_cloud [输出] 非地面点云
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   */
  void extractGround(PointCloud& filtered_cloud,
                     PointCloud& seed_cloud,
                     PointCloud& ground_cloud,
                     PointCloud& nonground_cloud,
                     const DEMConfig& config,
                     Logger& logger,
                     ProcessStats& stats) const;

 private:
  /**
   * @brief 获取当前活跃点的索引列表（地面点或非离群点）
   * @param cloud 点云
   * @param ground_only 是否仅返回地面点索引
   * @return 符合条件的点索引列表
   */
  std::vector<std::size_t> activeIndices(const PointCloud& cloud, bool ground_only) const;

  /**
   * @brief 使用粗格网方法选取最低点作为地面种子点
   * 在每个格网单元中选择 Z 值最小的点作为候选种子
   * @param cloud 点云（将被修改以标记种子）
   * @param config DEM 配置参数
   * @return 种子点索引列表
   */
  std::vector<std::size_t> seedGroundPoints(PointCloud& cloud, const DEMConfig& config) const;

  /**
   * @brief 计算邻域内点的 8 扇区空间分布完整度
   * @param points 全部点集合
   * @param neighbor_indices 邻域点索引
   * @param center_x 中心点 X 坐标
   * @param center_y 中心点 Y 坐标
   * @return 完整度比值 [0.0, 1.0]
   */
  double neighborhoodCompleteness(const std::vector<Point3D>& points,
                                  const std::vector<std::size_t>& neighbor_indices,
                                  double center_x,
                                  double center_y) const;

  /**
   * @brief 判断候选点是否应归类为地面点
   * 根据参考高度差和坡度阈值进行判定，支持多种参考模式
   * @param candidate 候选点（将被修改以存储参考值等属性）
   * @param cloud 全部点云
   * @param neighbor_indices 邻域地面点索引
   * @param config DEM 配置参数
   * @param stats 统计信息收集器
   * @return true 表示该点是地面点
   */
  bool classifyCandidate(Point3D& candidate,
                         const PointCloud& cloud,
                         const std::vector<std::size_t>& neighbor_indices,
                         const DEMConfig& config,
                         ProcessStats& stats) const;
};

}  // namespace dem
