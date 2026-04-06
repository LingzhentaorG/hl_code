/**
 * @file GroundFilterEngine.cpp
 * @brief 地面滤波引擎实现：离群点剔除与迭代式地面点提取
 */

#include "dem/GroundFilterEngine.hpp"

#include "dem/SpatialIndexManager.hpp"
#include "dem/Utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <numbers>
#include <stdexcept>
#include <unordered_map>

namespace dem {
namespace {

/** 粗格网单元键，用于种子点选取时的空间哈希 */
struct CellKey {
  long long x = 0;   /**< 格网列号 */
  long long y = 0;   /**< 格网行号 */

  bool operator==(const CellKey& other) const noexcept { return x == other.x && y == other.y; }
};

/** CellKey 的哈希函数 */
struct CellKeyHash {
  std::size_t operator()(const CellKey& key) const noexcept {
    return static_cast<std::size_t>((static_cast<std::uint64_t>(key.x) << 32U) ^
                                    static_cast<std::uint64_t>(key.y & 0xffffffffLL));
  }
};

/**
 * @brief 根据高程差和水平距离计算坡角（度）
 * @param rise 高程差（垂直方向）
 * @param run 水平距离
 * @return 坡角度数 [0, 90]
 */
double degreesFromRiseRun(double rise, double run) {
  return std::atan2(std::abs(rise), std::max(run, 1e-6)) * 180.0 / std::numbers::pi;
}

}  // namespace

/**
 * @brief 基于 KNN 统计方法移除 3D 空间中的离群点
 *
 * 算法流程：
 * 1. 对所有点构建 3D KD-Tree 索引
 * 2. 对每个点查找 K 个最近邻，计算平均邻域距离
 * 3. 根据配置选择标准差或 MAD（中位绝对偏差）方法确定阈值
 * 4. 平均邻域距离超过阈值的点被标记为离群点并剔除
 */
PointCloud GroundFilterEngine::removeOutliers(PointCloud cloud,
                                              const DEMConfig& config,
                                              Logger& logger,
                                              ProcessStats& stats) const {
  ScopedTimer timer(stats, "outlier_filter_seconds");

  /* 构建全量 3D 索引 */
  std::vector<std::size_t> indices(cloud.points.size());
  std::iota(indices.begin(), indices.end(), 0);

  SpatialIndexManager index;
  {
    const auto started = std::chrono::steady_clock::now();
    index.build3D(cloud.points, indices);
    stats.addDuration("kdtree_3d_build_seconds",
                      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
  }

  /* 计算每个点到其 K 个近邻的平均距离 */
  std::vector<double> mean_distances(cloud.points.size(), 0.0);
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const auto& point = cloud.points[i];
    auto neighbors = index.knn3D(point.x, point.y, point.z, config.outlier.knn + 1);
    double sum = 0.0;
    std::size_t used = 0;
    for (std::size_t j = 0; j < neighbors.indices.size(); ++j) {
      if (neighbors.indices[j] == i) { continue; }   /* 跳过自身 */
      sum += neighbors.distances[j];
      ++used;
      if (used >= config.outlier.knn) { break; }
    }
    mean_distances[i] = used == 0 ? std::numeric_limits<double>::infinity() : sum / static_cast<double>(used);
    cloud.points[i].local_mean_dist_3d = mean_distances[i];
  }

  /* 根据统计方法确定离群点阈值 */
  double threshold = 0.0;
  if (config.outlier.use_robust_stat) {
    /* 鲁棒统计方法：基于中位数和 MAD（乘以 1.4826 转换因子近似正态分布标准差） */
    const double med = median(mean_distances);
    const double mad_value = mad(mean_distances);
    threshold = med + config.outlier.stddev_multiplier * 1.4826 * mad_value;
  } else {
    /* 传统方法：基于均值和标准差 */
    const double mean = std::accumulate(mean_distances.begin(), mean_distances.end(), 0.0) /
                        std::max<std::size_t>(1, mean_distances.size());
    double variance = 0.0;
    for (const double value : mean_distances) {
      const double delta = value - mean;
      variance += delta * delta;
    }
    variance /= std::max<std::size_t>(1, mean_distances.size());
    threshold = mean + config.outlier.stddev_multiplier * std::sqrt(variance);
  }

  /* 过滤掉超过阈值的离群点 */
  PointCloud filtered;
  filtered.raw_point_count = cloud.raw_point_count;
  filtered.property_names = cloud.property_names;
  filtered.estimated_avg_spacing = cloud.estimated_avg_spacing;
  filtered.estimated_density = cloud.estimated_density;
  filtered.staging_xyz_path = cloud.staging_xyz_path;
  filtered.points.reserve(cloud.points.size());
  std::size_t removed = 0;
  for (auto& point : cloud.points) {
    if (point.local_mean_dist_3d > threshold) {
      point.outlier = true;
      ++removed;
      continue;
    }
    filtered.points.push_back(point);
  }

  filtered.bounds = {};
  for (const auto& point : filtered.points) {
    filtered.bounds.expand(point.x, point.y, point.z);
  }
  filtered.stored_point_count = filtered.points.size();
  stats.setCount("outlier_removed_count", removed);
  stats.setCount("filtered_point_count", filtered.points.size());
  stats.setValue("outlier_removed_ratio",
                 cloud.points.empty() ? 0.0 : static_cast<double>(removed) / static_cast<double>(cloud.points.size()));
  logger.info("Removed outliers: " + std::to_string(removed));
  return filtered;
}

/**
 * @brief 从过滤后的点云中提取地面点
 *
 * 完整流程：
 * 1. 种子选取：在粗格网的每个单元内取最低 Z 值点作为地面种子
 * 2. 迭代扩张：
 *    a. 为当前已知地面点构建 2D KD-Tree
 *    b. 对每个未分类候选点搜索邻域内的地面近邻
 *    c. 检查邻域完整度（8 扇区占用率），低于阈值则标记为边缘点跳过
 *    d. 使用多参考模式计算参考高程和坡度
 *    e. 高程差和坡度均满足阈值时判定为地面点
 * 3. 最终分类：分离地面点和非地面点
 */
void GroundFilterEngine::extractGround(PointCloud& filtered_cloud,
                                       PointCloud& seed_cloud,
                                       PointCloud& ground_cloud,
                                       PointCloud& nonground_cloud,
                                       const DEMConfig& config,
                                       Logger& logger,
                                       ProcessStats& stats) const {
  ScopedTimer timer(stats, "ground_extraction_seconds");

  /* 初始化所有点的分类状态 */
  for (auto& point : filtered_cloud.points) {
    point.ground = false;
    point.edge_point = false;
  }

  /* 第一步：粗格网种子选取 */
  const auto seed_indices = seedGroundPoints(filtered_cloud, config);
  seed_cloud = {};
  seed_cloud.raw_point_count = filtered_cloud.raw_point_count;
  seed_cloud.property_names = filtered_cloud.property_names;
  for (const auto idx : seed_indices) {
    filtered_cloud.points[idx].ground = true;
    seed_cloud.points.push_back(filtered_cloud.points[idx]);
    seed_cloud.bounds.expand(filtered_cloud.points[idx].x, filtered_cloud.points[idx].y, filtered_cloud.points[idx].z);
  }
  seed_cloud.stored_point_count = seed_cloud.points.size();
  stats.setCount("ground_seed_count", seed_indices.size());
  logger.info("Ground seeds: " + std::to_string(seed_indices.size()));

  /* 第二步：迭代式地面点扩张 */
  for (std::size_t iteration = 0; iteration < config.ground.max_iterations; ++iteration) {
    /* 收集当前所有已知地面点的索引 */
    auto ground_indices = activeIndices(filtered_cloud, true);
    if (ground_indices.size() < config.ground.min_ground_neighbors) {
      break;   /* 地面点过少无法继续扩张 */
    }

    /* 为当前地面点构建 2D 索引用于邻域搜索 */
    SpatialIndexManager ground_index;
    {
      const auto started = std::chrono::steady_clock::now();
      ground_index.build2D(filtered_cloud.points, ground_indices);
      stats.addDuration("kdtree_2d_build_seconds",
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    }

    std::vector<std::size_t> new_ground_indices;
    const std::size_t max_results =
        std::max<std::size_t>({config.ground.knn * 8U, config.ground.min_ground_neighbors * 4U, 32U});

    /* 遍历所有未分类候选点进行判定 */
    for (std::size_t i = 0; i < filtered_cloud.points.size(); ++i) {
      auto& candidate = filtered_cloud.points[i];
      if (candidate.ground) { continue; }

      /* 在搜索半径内查找地面近邻 */
      auto neighbors = ground_index.radius2D(candidate.x, candidate.y, config.ground.search_radius, max_results);
      if (neighbors.indices.size() < config.ground.min_ground_neighbors) {
        continue;   /* 地面近邻不足，跳过 */
      }

      /* 限制使用的近邻数量不超过 knn 配置 */
      if (neighbors.indices.size() > config.ground.knn) {
        neighbors.indices.resize(config.ground.knn);
        neighbors.distances.resize(config.ground.knn);
      }

      /* 检查邻域完整度（8 扇区），但不再把低完整度直接当作硬拒绝 */
      const auto occupied_sectors = occupiedSectors8(candidate.x, candidate.y, filtered_cloud.points, neighbors.indices);
      const double completeness = static_cast<double>(occupied_sectors) / 8.0;
      candidate.neighborhood_completeness = completeness;
      candidate.edge_point = completeness < config.ground.min_neighbor_completeness;
      if (occupied_sectors < 2U) { continue; }

      /* 使用多参考模式进行最终分类判定 */
      if (classifyCandidate(candidate, filtered_cloud, neighbors.indices, config, stats)) {
        new_ground_indices.push_back(i);
      }
    }

    /* 将本轮新判定的地面点加入集合 */
    for (const auto idx : new_ground_indices) {
      filtered_cloud.points[idx].ground = true;
    }
    stats.ground_added_per_iteration.push_back(new_ground_indices.size());
    logger.info("Ground iteration " + std::to_string(iteration + 1) +
                " added " + std::to_string(new_ground_indices.size()) + " points");

    /* 收敛判断：无新增或新增比例极低则终止迭代 */
    if (new_ground_indices.empty() ||
        static_cast<double>(new_ground_indices.size()) / std::max<std::size_t>(1, filtered_cloud.points.size()) < 1e-4) {
      break;
    }
  }

  /* 第三步：对所有最终地面点重新评估边缘属性 */
  auto final_ground_indices = activeIndices(filtered_cloud, true);
  if (!final_ground_indices.empty()) {
    SpatialIndexManager ground_index;
    {
      const auto started = std::chrono::steady_clock::now();
      ground_index.build2D(filtered_cloud.points, final_ground_indices);
      stats.addDuration("kdtree_2d_build_seconds",
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    }
    for (auto idx : final_ground_indices) {
      auto neighbors = ground_index.radius2D(filtered_cloud.points[idx].x,
                                             filtered_cloud.points[idx].y,
                                             config.ground.search_radius,
                                             std::max<std::size_t>(32, config.ground.knn * 8U));
      filtered_cloud.points[idx].neighborhood_completeness =
          neighborhoodCompleteness(filtered_cloud.points, neighbors.indices, filtered_cloud.points[idx].x, filtered_cloud.points[idx].y);
      filtered_cloud.points[idx].edge_point =
          filtered_cloud.points[idx].neighborhood_completeness < config.ground.min_neighbor_completeness;
    }
  }

  /* 第四步：分离输出四类点云 */
  ground_cloud = {};
  ground_cloud.raw_point_count = filtered_cloud.raw_point_count;
  ground_cloud.property_names = filtered_cloud.property_names;
  nonground_cloud = ground_cloud;

  for (const auto& point : filtered_cloud.points) {
    if (point.ground) {
      ground_cloud.points.push_back(point);
      ground_cloud.bounds.expand(point.x, point.y, point.z);
    } else {
      nonground_cloud.points.push_back(point);
      nonground_cloud.bounds.expand(point.x, point.y, point.z);
    }
  }

  stats.setCount("ground_point_count", ground_cloud.points.size());
  stats.setCount("nonground_point_count", nonground_cloud.points.size());
  ground_cloud.stored_point_count = ground_cloud.points.size();
  nonground_cloud.stored_point_count = nonground_cloud.points.size();
  stats.setValue("ground_point_ratio",
                 filtered_cloud.points.empty()
                     ? 0.0
                     : static_cast<double>(ground_cloud.points.size()) / static_cast<double>(filtered_cloud.points.size()));
  logger.info("Final ground points: " + std::to_string(ground_cloud.points.size()));
}

/**
 * @brief 从点云中收集符合条件（地面/非离群）的点的索引列表
 */
std::vector<std::size_t> GroundFilterEngine::activeIndices(const PointCloud& cloud, bool ground_only) const {
  std::vector<std::size_t> indices;
  indices.reserve(cloud.points.size());
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    if (ground_only) {
      if (cloud.points[i].ground) { indices.push_back(i); }
    } else if (!cloud.points[i].outlier) {
      indices.push_back(i);
    }
  }
  return indices;
}

/**
 * @brief 通过粗格网最低点法选取地面种子点
 *
 * 将点云范围划分为规则网格，在每个网格单元内取 Z 值最小的点作为种子。
 * 这种方法假设地面点通常位于局部区域的最低处。
 */
std::vector<std::size_t> GroundFilterEngine::seedGroundPoints(PointCloud& cloud, const DEMConfig& config) const {
  std::unordered_map<CellKey, std::size_t, CellKeyHash> lowest_by_cell;

  /* 遍历所有点，记录每个格网单元内的最低点索引 */
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const auto& point = cloud.points[i];
    const auto cell_x = static_cast<long long>(std::floor((point.x - cloud.bounds.xmin) / config.ground.seed_grid_size));
    const auto cell_y = static_cast<long long>(std::floor((point.y - cloud.bounds.ymin) / config.ground.seed_grid_size));
    const CellKey key{cell_x, cell_y};
    const auto it = lowest_by_cell.find(key);
    if (it == lowest_by_cell.end() || point.z < cloud.points[it->second].z) {
      lowest_by_cell[key] = i;
    }
  }

  /* 提取所有种子点索引 */
  std::vector<std::size_t> seeds;
  seeds.reserve(lowest_by_cell.size());
  for (const auto& [cell, idx] : lowest_by_cell) {
    std::vector<double> neighbor_lows;
    for (long long dy = -1; dy <= 1; ++dy) {
      for (long long dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const CellKey neighbor_key{cell.x + dx, cell.y + dy};
        const auto it = lowest_by_cell.find(neighbor_key);
        if (it == lowest_by_cell.end()) {
          continue;
        }
        neighbor_lows.push_back(cloud.points[it->second].z);
      }
    }
    if (neighbor_lows.size() >= 4U &&
        cloud.points[idx].z > median(neighbor_lows) + config.ground.max_height_diff) {
      continue;
    }
    seeds.push_back(idx);
  }
  return seeds;
}

/**
 * @brief 计算邻域完整度（8 扇区占用率）
 * 返回值范围为 [0.0, 1.0]，表示邻域点在各方向的覆盖程度。
 */
double GroundFilterEngine::neighborhoodCompleteness(const std::vector<Point3D>& points,
                                                    const std::vector<std::size_t>& neighbor_indices,
                                                    double center_x,
                                                    double center_y) const {
  if (neighbor_indices.empty()) { return 0.0; }
  return static_cast<double>(occupiedSectors8(center_x, center_y, points, neighbor_indices)) / 8.0;
}

/**
 * @brief 使用多参考模式对候选点进行地面/非地面分类判定
 *
 * 判定策略：
 * 1. 优先尝试 LocalPlane 模式（需 ≥4 个近邻点）
 * 2. 若平面拟合失败或不稳定，回退到 fallback_mode（IDW 或 LocalMin）
 * 3. 若 fallback 也失败，再尝试首选模式的非平面版本
 *
 * 判定条件：高度差 ≤ max_height_diff 且 坡度 ≤ max_slope_deg
 */
bool GroundFilterEngine::classifyCandidate(Point3D& candidate,
                                           const PointCloud& cloud,
                                           const std::vector<std::size_t>& neighbor_indices,
                                           const DEMConfig& config,
                                           ProcessStats& stats) const {
  if (neighbor_indices.empty()) { return false; }

  /**
   * @brief IDW 加权参考高程计算辅助 lambda
   * 支持 InverseDistanceSquared 和 LocalMin 两种加权模式
   */
  const auto use_weighted_reference = [&](ReferenceMode mode, double& ref_z, double& slope_deg, std::string& mode_key) {
    if (mode == ReferenceMode::LocalMin) {
      /* LocalMin 模式：取邻域内最低 Z 值作为参考 */
      ref_z = std::numeric_limits<double>::infinity();
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (const auto idx : neighbor_indices) {
        const auto& point = cloud.points[idx];
        ref_z = std::min(ref_z, point.z);
        const double dx = candidate.x - point.x;
        const double dy = candidate.y - point.y;
        nearest_distance = std::min(nearest_distance, std::sqrt(dx * dx + dy * dy));
      }
      slope_deg = degreesFromRiseRun(candidate.z - ref_z, nearest_distance);
      mode_key = "local_min";
      return true;
    }

    /* IDW 加权模式：反距离权重加权平均 */
    double numerator = 0.0;
    double denominator = 0.0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const auto idx : neighbor_indices) {
      const auto& point = cloud.points[idx];
      const double dx = candidate.x - point.x;
      const double dy = candidate.y - point.y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < 1e-6) {
        ref_z = point.z;
        slope_deg = 0.0;
        mode_key = "inverse_distance_squared";
        return true;
      }
      nearest_distance = std::min(nearest_distance, distance);
      const double weight = 1.0 / std::pow(distance, config.ground.distance_weight_power);
      numerator += point.z * weight;
      denominator += weight;
    }
    if (denominator <= 0.0) { return false; }
    ref_z = numerator / denominator;
    slope_deg = degreesFromRiseRun(candidate.z - ref_z, nearest_distance);
    mode_key = "inverse_distance_squared";
    return true;
  };

  double reference_z = 0.0;
  double slope_deg = 0.0;
  std::string mode_key;
  bool success = false;

  /* 尝试首选 LocalPlane 模式（需 ≥4 点才能拟合平面） */
  if (config.ground.reference_mode == ReferenceMode::LocalPlane && neighbor_indices.size() >= 4) {
    const auto plane = fitPlaneLeastSquares(cloud.points, neighbor_indices);
    if (plane.success && plane.condition_indicator < 1e6) {
      reference_z = plane.a * candidate.x + plane.b * candidate.y + plane.c;
      slope_deg = plane.slope_deg;
      mode_key = "local_plane";
      success = true;
    }
  }

  /* 回退到 fallback 模式 */
  if (!success) {
    success = use_weighted_reference(config.ground.reference_fallback_mode, reference_z, slope_deg, mode_key);
  }

  /* 二次回退：尝试首选模式的非平面版本 */
  if (!success && config.ground.reference_mode != config.ground.reference_fallback_mode) {
    success = use_weighted_reference(config.ground.reference_mode, reference_z, slope_deg, mode_key);
  }
  if (!success) { return false; }

  /* 记录分类中间结果并执行最终阈值判定 */
  std::vector<double> neighbor_heights;
  neighbor_heights.reserve(neighbor_indices.size());
  for (const auto idx : neighbor_indices) {
    neighbor_heights.push_back(cloud.points[idx].z);
  }
  const double neighborhood_median_z =
      neighbor_heights.empty() ? std::numeric_limits<double>::infinity() : median(neighbor_heights);

  candidate.reference_z = reference_z;
  candidate.local_height_diff = std::abs(candidate.z - reference_z);
  candidate.local_slope = slope_deg;
  stats.reference_mode_counts[mode_key] += 1;

  return candidate.local_height_diff <= config.ground.max_height_diff &&
         candidate.z <= neighborhood_median_z + config.ground.max_height_diff &&
         candidate.local_slope <= config.ground.max_slope_deg;
}

}  // namespace dem
