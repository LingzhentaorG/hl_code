#pragma once

#include "dem/Logger.hpp"
#include "dem/Types.hpp"

namespace dem {

/**
 * @brief 预处理引擎，负责点云数据的清洗和规范化
 *
 * 在地面滤波之前执行以下预处理步骤：
 * 1. 移除无效点（NaN / Inf）
 * 2. 移除重复点
 * 3. 移除明显离群点（基于统计分布）
 * 4. 重新索引
 * 5. 更新边界和密度信息
 * 6. 估算平均点间距
 */
class PreprocessEngine {
 public:
  /**
   * @brief 执行完整的点云预处理流程
   * @param cloud 输入原始点云（将被移动/修改）
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 预处理后的干净点云
   */
  PointCloud run(PointCloud cloud, const DEMConfig& config, Logger& logger, ProcessStats& stats) const;

 private:
  /**
   * @brief 移除包含 NaN 或 Inf 坐标的无效点
   * @param cloud 点云（将被修改）
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   */
  void removeInvalidPoints(PointCloud& cloud, Logger& logger, ProcessStats& stats) const;

  /**
   * @brief 基于精确坐标匹配移除重复点
   * 使用位模式哈希进行高效去重
   * @param cloud 点云（将被修改）
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   */
  void removeDuplicatePoints(PointCloud& cloud, Logger& logger, ProcessStats& stats) const;

  /**
   * @brief 基于分位数统计范围移除明显的极端离群点
   * 对每个坐标轴计算 1%~99% 分位数范围，超出扩展范围的点被剔除
   * @param cloud 点云（将被修改）
   * @param config DEM 配置参数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   */
  void removeExtremePoints(PointCloud& cloud, const DEMConfig& config, Logger& logger, ProcessStats& stats) const;

  /**
   * @brief 对点云中的所有点重新分配连续索引号
   * @param cloud 点云（将被修改）
   */
  void reindex(PointCloud& cloud) const;

  /**
   * @brief 根据当前点集重新计算空间边界和点密度
   * @param cloud 点云（将被修改）
   */
  void updateBoundsAndDensity(PointCloud& cloud) const;

  /**
   * @brief 通过采样方法估算点云的平均点间距
   * 随机采样部分点，计算每个采样点到最近邻的距离均值
   * @param cloud 点云（将被修改，写入 estimated_avg_spacing 字段）
   * @param config DEM 配置参数
   */
  void estimateSpacing(PointCloud& cloud, const DEMConfig& config) const;
};

}  // namespace dem
