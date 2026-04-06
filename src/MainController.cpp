/**
 * @file MainController.cpp
 * @brief 主控制器实现：协调 DEM 处理流水线的完整执行流程
 */

#include "dem/MainController.hpp"

#include "dem/CRSManager.hpp"
#include "dem/DEMEngine.hpp"
#include "dem/GroundFilterEngine.hpp"
#include "dem/InputManager.hpp"
#include "dem/OutputManager.hpp"
#include "dem/PreprocessEngine.hpp"
#include "dem/SpatialIndexManager.hpp"
#include "dem/TileManager.hpp"
#include "dem/Utils.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace dem {

/**
 * @brief 执行完整的 DEM 生成流水线
 *
 * 标准模式（非 Tile 分块）的处理顺序：
 * 1. 输入验证：PLY 文件解析与点云质量检查
 * 2. CRS 解析：确定坐标参考系定义
 * 3. 预处理：NaN/Inf 清洗 → 去重 → 极端离群点剔除
 * 4. 离群点过滤：基于 KNN 统计的 3D 离群点剔除
 * 5. 地面滤波：种子选取 → 迭代扩张 → 地面/非地面分类
 * 6. DEM 插值：最近邻或 IDW 插值生成栅格
 * 7. 空洞填充：迭代式邻域均值填充 NoData 区域
 * 8. 结果输出：PLY 点云 + PNG 栅格（GeoTIFF 由 Python 脚本生成）
 */
void MainController::run(const std::filesystem::path& input_file,
                         const DEMConfig& config,
                         const std::filesystem::path& output_directory,
                         Logger& logger,
                         ProcessStats& stats) const {
  /* 记录处理开始时间用于总耗时统计 */
  const auto pipeline_start = std::chrono::steady_clock::now();

  logger.info("=== DEM Generation Pipeline Started ===");
  logger.info("Input: " + input_file.string());
  logger.info("Output: " + output_directory.string());
  logger.info("Config: " + config.config_path.string());

  /* 阶段1：输入验证 - 解析 PLY 文件头并扫描全部点统计质量指标 */
  InputManager input_manager;
  auto validation_report = input_manager.validateInput(input_file, config, output_directory, logger, stats);
  if (!validation_report.output_directory_writable) {
    throw std::runtime_error("Output directory is not writable: " + output_directory.string());
  }

  /* 阶段2：坐标参考系解析 */
  CRSManager crs_manager;
  auto resolved_crs = crs_manager.resolve(config.crs, logger, stats);

  /* 阶段3：预处理 - 点云清洗流水线 */
  PreprocessEngine preprocess_engine;
  PointCloud raw_cloud = input_manager.readPointCloud(input_file, logger, stats);
  PointCloud preprocessed_cloud = preprocess_engine.run(raw_cloud, config, logger, stats);

  /* 阶段4：离群点过滤 - KNN 统计方法剔除 3D 离群点 */
  GroundFilterEngine ground_filter_engine;
  PointCloud filtered_cloud = ground_filter_engine.removeOutliers(preprocessed_cloud, config, logger, stats);

  /* 阶段5：地面滤波 - 迭代式地面点提取与分类 */
  PointCloud seed_cloud;
  PointCloud ground_cloud;
  PointCloud nonground_cloud;
  ground_filter_engine.extractGround(filtered_cloud, seed_cloud, ground_cloud, nonground_cloud, config, logger, stats);

  /* 阶段6：DEM 插值 - 从地面点云生成高程栅格 */
  DEMEngine dem_engine;
  DEMRaster raster = dem_engine.interpolate(ground_cloud, config, logger, stats);

  /* 阶段7：空洞填充 - 迭代式邻域均值填充 NoData 区域 */
  dem_engine.fillHoles(raster, config, logger, stats);

  /* 阶段8：结果输出 - 写入 PLY 和 PNG 文件 */
  OutputManager output_manager;

  /* 输出原始点云（含分类标记） */
  if (config.output.write_raw_ply) {
    output_manager.writePly(output_directory / "raw.ply", preprocessed_cloud, logger, stats);
  }

  /* 输出种子点集 */
  if (config.output.write_seed_ply) {
    output_manager.writePly(output_directory / "seed.ply", seed_cloud, logger, stats);
  }

  /* 输出地面点集 */
  if (config.output.write_ground_ply) {
    output_manager.writePly(output_directory / "ground.ply", ground_cloud, logger, stats);
  }

  /* 输出非地面点集 */
  if (config.output.write_nonground_ply) {
    output_manager.writePly(output_directory / "nonground.ply", nonground_cloud, logger, stats);
  }

  /* 输出 PNG 格式的 DEM 可视化图像 */
  if (config.output.write_png) {
    output_manager.writePng(output_directory / "dem.png", raster, logger, stats);
  }

  /* 导出 JSON 格式的处理报告 */
  exportJsonReport(output_directory / "report.json", raster, config, stats, resolved_crs);

  /* 记录总耗时 */
  const auto pipeline_end = std::chrono::steady_clock::now();
  stats.addDuration("total_pipeline_seconds",
                    std::chrono::duration<double>(pipeline_end - pipeline_start).count());

  logger.info("=== Pipeline Finished ===");
}

}  // namespace dem
