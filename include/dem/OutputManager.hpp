#pragma once

#include "dem/Logger.hpp"
#include "dem/Types.hpp"

namespace dem {

/**
 * @brief 输出管理器，负责所有处理结果的持久化写入
 *
 * 负责：
 * - 分类后点云的 PLY 格式写出
 * - DEM 栅格的 GeoTIFF 格式写出（通过 Python 脚本桥接 rasterio）
 * - PNG 可视化图像的生成
 * - 统计信息的文本记录
 */
class OutputManager {
 public:
  /**
   * @brief 写入所有处理产物到输出目录
   * @param artifacts 处理产物集合
   * @param config DEM 配置参数
   * @param crs 坐标参考系定义
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   */
  void writeArtifacts(const ProcessingArtifacts& artifacts,
                      const DEMConfig& config,
                      const CRSDefinition& crs,
                      Logger& logger,
                      ProcessStats& stats) const;

  /**
   * @brief 写入处理统计信息到文本文件
   * @param artifacts 处理产物集合
   * @param config DEM 配置参数
   * @param stats 统计信息收集器
   * @param path 输出文件路径
   */
  void writeStats(const ProcessingArtifacts& artifacts,
                  const DEMConfig& config,
                  const ProcessStats& stats,
                  const std::filesystem::path& path) const;

 private:
  /**
   * @brief 将点云以 ASCII PLY 格式写入文件
   * @param cloud 待输出的点云
   * @param path 输出文件路径
   */
  void writePointCloudPly(const PointCloud& cloud, const std::filesystem::path& path) const;

  /**
   * @brief 将 DEM 栅格以 GeoTIFF 格式写入文件
   * 通过调用 Python 脚本使用 rasterio 库完成实际写入
   * @param grid 栅格数据
   * @param config DEM 配置参数
   * @param crs 坐标参考系定义
   * @param path 输出文件路径
   * @param logger 日志记录器
   */
  void writeGeoTiff(const RasterGrid& grid,
                    const DEMConfig& config,
                    const CRSDefinition& crs,
                    const std::filesystem::path& path,
                    Logger& logger) const;
};

}  // namespace dem
