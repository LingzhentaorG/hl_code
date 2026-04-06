/**
 * @file DEMEngine.cpp
 * @brief DEM 插值引擎实现：最近邻/IDW 插值与空洞填充
 */

#include "dem/DEMEngine.hpp"

#include "dem/SpatialIndexManager.hpp"
#include "dem/Utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace dem {

/**
 * @brief 使用地面点云生成 DEM 栅格
 *
 * 处理流程：
 * 1. 根据点云边界和分辨率计算栅格尺寸
 * 2. 为地面点构建 2D KD-Tree 索引
 * 3. 对每个栅格像素执行插值（最近邻或 IDW）
 * 4. 统计有效像素数量和空洞比例
 */
DEMRaster DEMEngine::interpolate(const PointCloud& ground_cloud,
                                 const DEMConfig& config,
                                 Logger& logger,
                                 ProcessStats& stats) const {
  ScopedTimer timer(stats, "interpolation_seconds");

  /* 根据点云范围和配置的分辨率确定栅格尺寸 */
  const std::size_t cols = static_cast<std::size_t>(std::ceil(ground_cloud.bounds.width() / config.dem.resolution));
  const std::size_t rows = static_cast<std::size_t>(std::ceil(ground_cloud.bounds.height() / config.dem.resolution));
  if (cols == 0 || rows == 0) {
    throw std::runtime_error("DEM grid is empty after resolution adjustment.");
  }

  /* 为所有地面点构建 2D 空间索引用于快速邻域搜索 */
  SpatialIndexManager index;
  {
    const auto started = std::chrono::steady_clock::now();
    std::vector<std::size_t> indices(ground_cloud.points.size());
    std::iota(indices.begin(), indices.end(), 0);
    index.build2D(ground_cloud.points, indices);
    stats.addDuration("kdtree_2d_build_seconds",
                      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
  }

  /* 初始化 DEM 栅格，全部填充 NoData 值 */
  DEMRaster raster;
  raster.cols = cols;
  raster.rows = rows;
  raster.nodata_value = config.dem.nodata_value;
  raster.data.assign(cols * rows, config.dem.nodata_value);
  raster.xmin = ground_cloud.bounds.xmin;
  raster.ymin = ground_cloud.bounds.ymin;
  raster.xmax = ground_cloud.bounds.xmax;
  raster.ymax = ground_cloud.bounds.ymax;
  raster.resolution = config.dem.resolution;

  /* 遍历每个栅格像素执行插值 */
  std::size_t valid_count = 0;
  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t col = 0; col < cols; ++col) {
      /* 计算当前像素中心点的地理坐标 */
      const double pixel_x = ground_cloud.bounds.xmin + (static_cast<double>(col) + 0.5) * config.dem.resolution;
      const double pixel_y = ground_cloud.bounds.ymin + (static_cast<double>(row) + 0.5) * config.dem.resolution;

      /* 在搜索半径内查找最近的地面点 */
      auto neighbors =
          index.radius2D(pixel_x, pixel_y, config.dem.search_radius, std::max<std::size_t>(32, config.dem.knn * 4U));
      if (neighbors.indices.empty()) {
        continue;   /* 无近邻点则保持 NoData（形成空洞） */
      }

      /* 限制参与插值的近邻数量不超过 knn 配置 */
      if (neighbors.indices.size() > config.dem.knn) {
        neighbors.indices.resize(config.dem.knn);
        neighbors.distances.resize(config.dem.knn);
      }

      /* 选择插值方法并计算高程值 */
      const auto pixel_index = row * cols + col;
      if (config.dem.interpolation_method == InterpolationMethod::NearestNeighbor) {
        /* 最近邻法：直接使用最近点的 Z 值 */
        raster.data[pixel_index] = ground_cloud.points[neighbors.indices[0]].z;
      } else {
        /* IDW 反距离加权插值：距离越近权重越大 */
        double numerator = 0.0;
        double denominator = 0.0;
        for (std::size_t j = 0; j < neighbors.indices.size(); ++j) {
          const double distance = neighbors.distances[j];
          if (distance < 1e-6) {
            /* 像素中心恰好落在某个点上时直接取该点高程 */
            raster.data[pixel_index] = ground_cloud.points[neighbors.indices[j]].z;
            numerator = 1.0;
            denominator = 1.0;
            break;
          }
          const double weight = 1.0 / std::pow(distance, config.dem.distance_weight_power);
          numerator += ground_cloud.points[neighbors.indices[j]].z * weight;
          denominator += weight;
        }
        if (denominator > 0.0) {
          raster.data[pixel_index] = numerator / denominator;
        }
      }

      if (raster.data[pixel_index] != config.dem.nodata_value) {
        ++valid_count;   /* 统计有效像素数 */
      }
    }
  }

  /* 记录插值结果统计信息 */
  const std::size_t total_pixels = cols * rows;
  stats.setCount("dem_valid_pixels", valid_count);
  stats.setCount("dem_total_pixels", total_pixels);
  stats.setValue("dem_void_ratio", 1.0 - static_cast<double>(valid_count) / static_cast<double>(total_pixels));
  logger.info("Interpolation complete: " + std::to_string(valid_count) + " / " + std::to_string(total_pixels) + " pixels");

  return raster;
}

/**
 * @brief 对 DEM 栅格中的空洞进行迭代式填充
 *
 * 算法流程：
 * 1. 检测所有 NoData 像素作为待填充空洞
 * 2. 迭代多轮填充：
 *    a. 收集所有已填充的有效像素索引
 *    b. 对每个空洞像素搜索其 8 邻域内的有效邻居
 *    c. 用邻居均值填充空洞
 *    d. 无新增填充或达到最大轮次时终止
 *
 * 边界处理：边缘像素仅检查实际存在的邻域位置。
 */
void DEMEngine::fillHoles(DEMRaster& raster, const DEMConfig& config, Logger& logger, ProcessStats& stats) const {
  ScopedTimer timer(stats, "hole_filling_seconds");

  /* 第一步：收集所有空洞像素坐标 */
  std::vector<std::pair<std::size_t, std::size_t>> void_pixels;
  void_pixels.reserve(raster.cols * raster.rows);
  for (std::size_t row = 0; row < raster.rows; ++row) {
    for (std::size_t col = 0; col < raster.cols; ++col) {
      if (raster.data[row * raster.cols + col] == raster.nodata_value) {
        void_pixels.emplace_back(row, col);
      }
    }
  }

  if (void_pixels.empty()) {
    logger.info("No holes to fill.");
    return;
  }

  stats.setCount("hole_initial_count", void_pixels.size());

  /* 第二步：迭代式空洞填充 */
  std::set<std::pair<std::size_t, std::size_t>> remaining(void_pixels.begin(), void_pixels.end());
  for (std::size_t iteration = 0; iteration < config.hole_fill.max_iterations && !remaining.empty(); ++iteration) {
    /* 收集当前所有有效像素用于邻域查询 */
    std::set<std::pair<std::size_t, std::size_t>> filled;
    for (std::size_t row = 0; row < raster.rows; ++row) {
      for (std::size_t col = 0; col < raster.cols; ++col) {
        if (raster.data[row * raster.cols + col] != raster.nodata_value) {
          filled.emplace(row, col);
        }
      }
    }

    std::size_t newly_filled = 0;

    /* 遍历剩余空洞像素尝试用邻域均值填充 */
    for (auto it = remaining.begin(); it != remaining.end();) {
      const auto [row, col] = *it;

      /* 搜索 8 邻域范围内的有效像素 */
      std::vector<double> neighbor_values;
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          if (dr == 0 && dc == 0) { continue; }   /* 跳过自身 */
          const long long nr = static_cast<long long>(row) + dr;
          const long long nc = static_cast<long long>(col) + dc;
          if (nr >= 0 && nr < static_cast<long long>(raster.rows) &&
              nc >= 0 && nc < static_cast<long long>(raster.cols)) {
            const auto value = raster.data[nr * raster.cols + nc];
            if (value != raster.nodata_value) {
              neighbor_values.push_back(value);
            }
          }
        }
      }

      /* 当存在有效邻域时用均值填充空洞 */
      if (!neighbor_values.empty()) {
        const double sum = std::accumulate(neighbor_values.begin(), neighbor_values.end(), 0.0);
        raster.data[row * raster.cols + col] = sum / static_cast<double>(neighbor_values.size());
        it = remaining.erase(it);   /* 从剩余集合中移除已填充像素 */
        ++newly_filled;
      } else {
        ++it;   /* 无有效邻域则保留到下一轮 */
      }
    }

    stats.hole_filled_per_iteration.push_back(newly_filled);
    logger.info("Hole fill iteration " + std::to_string(iteration + 1) +
                ": filled " + std::to_string(newly_filled) + " pixels");

    /* 收敛判断 */
    if (newly_filled == 0) { break; }
  }

  /* 最终统计 */
  stats.setCount("hole_remaining_count", remaining.size());
  logger.info("Hole fill finished: " + std::to_string(remaining.size()) + " holes remain");
}

}  // namespace dem
