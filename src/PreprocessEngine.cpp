/**
 * @file PreprocessEngine.cpp
 * @brief 预处理引擎实现：点云清洗与规范化流水线
 */

#include "dem/PreprocessEngine.hpp"

#include "dem/SpatialIndexManager.hpp"
#include "dem/Utils.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace dem {
namespace {

/**
 * @brief 用于精确去重的位模式哈希键结构
 * 与 InputManager 中的 SamplePointKey 功能相同，
 * 但此处用于预处理阶段的完整点集去重。
 */
struct PointKey {
  std::uint64_t x_bits;   /**< X 坐标的 IEEE-754 位模式 */
  std::uint64_t y_bits;   /**< Y 坐标的 IEEE-754 位模式 */
  std::uint64_t z_bits;   /**< Z 坐标的 IEEE-754 位模式 */

  bool operator==(const PointKey& other) const noexcept {
    return x_bits == other.x_bits && y_bits == other.y_bits && z_bits == other.z_bits;
  }
};

/** PointKey 的哈希函数，通过异或混合三个坐标的位模式 */
struct PointKeyHash {
  std::size_t operator()(const PointKey& key) const noexcept {
    return static_cast<std::size_t>(key.x_bits ^ (key.y_bits << 1U) ^ (key.z_bits << 7U));
  }
};

}  // namespace

/**
 * @brief 执行完整的预处理流水线
 *
 * 处理顺序：
 * 1. 移除 NaN/Inf 无效点
 * 2. 移除精确重复的点
 * 3. 移除统计意义上的极端离群点
 * 4. 重新分配连续索引号
 * 5. 更新空间边界和密度
 * 6. 估算平均点间距
 */
PointCloud PreprocessEngine::run(PointCloud cloud,
                                 const DEMConfig& config,
                                 Logger& logger,
                                 ProcessStats& stats) const {
  ScopedTimer timer(stats, "preprocess_seconds");

  removeInvalidPoints(cloud, logger, stats);
  removeDuplicatePoints(cloud, logger, stats);
  removeExtremePoints(cloud, config, logger, stats);

  reindex(cloud);
  updateBoundsAndDensity(cloud);
  estimateSpacing(cloud, config);

  cloud.stored_point_count = cloud.points.size();

  /* 记录预处理后的关键指标 */
  stats.setCount("preprocessed_point_count", cloud.points.size());
  stats.setValue("preprocess_bounds_xmin", cloud.bounds.xmin);
  stats.setValue("preprocess_bounds_xmax", cloud.bounds.xmax);
  stats.setValue("preprocess_bounds_ymin", cloud.bounds.ymin);
  stats.setValue("preprocess_bounds_ymax", cloud.bounds.ymax);
  stats.setValue("preprocess_bounds_zmin", cloud.bounds.zmin);
  stats.setValue("preprocess_bounds_zmax", cloud.bounds.zmax);
  stats.setValue("average_point_spacing", cloud.estimated_avg_spacing);
  stats.setValue("point_density", cloud.estimated_density);
  logger.info("Preprocess finished with " + std::to_string(cloud.points.size()) + " points");
  return cloud;
}

/**
 * @brief 移除包含 NaN 或 Inf 坐标的无效点
 * 分别统计 NaN 和 Inf 的剔除数量，保留所有坐标均为有限值的点。
 */
void PreprocessEngine::removeInvalidPoints(PointCloud& cloud, Logger& logger, ProcessStats& stats) const {
  std::vector<Point3D> filtered;
  filtered.reserve(cloud.points.size());
  std::size_t nan_removed = 0;
  std::size_t inf_removed = 0;

  for (const auto& point : cloud.points) {
    const bool nan_value = std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z);
    const bool inf_value = !nan_value && (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z));
    if (nan_value) {
      ++nan_removed;
      continue;
    }
    if (inf_value) {
      ++inf_removed;
      continue;
    }
    filtered.push_back(point);
  }

  cloud.points = std::move(filtered);
  stats.setCount("removed_nan_points", nan_removed);
  stats.setCount("removed_inf_points", inf_removed);
  logger.info("Removed invalid points: NaN=" + std::to_string(nan_removed) +
              ", Inf=" + std::to_string(inf_removed));
}

/**
 * @brief 基于 std::bit_cast 位模式哈希移除精确重复的点
 * 使用 unordered_set 进行 O(1) 查重，内存开销约为 O(N)。
 */
void PreprocessEngine::removeDuplicatePoints(PointCloud& cloud, Logger& logger, ProcessStats& stats) const {
  std::unordered_set<PointKey, PointKeyHash> seen;
  seen.reserve(cloud.points.size());
  std::vector<Point3D> unique_points;
  unique_points.reserve(cloud.points.size());
  std::size_t duplicates_removed = 0;

  for (const auto& point : cloud.points) {
    const PointKey key{
        .x_bits = std::bit_cast<std::uint64_t>(point.x),
        .y_bits = std::bit_cast<std::uint64_t>(point.y),
        .z_bits = std::bit_cast<std::uint64_t>(point.z),
    };
    if (seen.insert(key).second) {
      unique_points.push_back(point);
    } else {
      ++duplicates_removed;
    }
  }

  cloud.points = std::move(unique_points);
  stats.setCount("removed_duplicate_points", duplicates_removed);
  logger.info("Removed duplicate points: " + std::to_string(duplicates_removed));
}

/**
 * @brief 基于分位数统计范围移除明显的极端离群点
 *
 * 对每个坐标轴分别计算 1%~99% 分位数范围，
 * 超出扩展范围（X/Y 扩展 10 倍，Z 扩展 5 倍）的点被判定为极端离群点并剔除。
 * 此步骤在 KNN 离群点检测之前执行，用于快速去除明显异常值以加速后续处理。
 */
void PreprocessEngine::removeExtremePoints(PointCloud& cloud,
                                           const DEMConfig& config,
                                           Logger& logger,
                                           ProcessStats& stats) const {
  /* 点数过少时跳过此步骤 */
  if (cloud.points.size() < 32) {
    stats.setCount("removed_extreme_points", 0);
    return;
  }

  /* 对各坐标轴进行等间距采样以减少计算量 */
  const auto xs = sampleValues(cloud.points, config.preprocess.max_extreme_sample, [](const Point3D& p) { return p.x; });
  const auto ys = sampleValues(cloud.points, config.preprocess.max_extreme_sample, [](const Point3D& p) { return p.y; });
  const auto zs = sampleValues(cloud.points, config.preprocess.max_extreme_sample, [](const Point3D& p) { return p.z; });

  /* 计算各轴的分位数范围及扩展边界 */
  auto bounds_for = [](std::vector<double> values, double low_q, double high_q, double multiplier) {
    const double q_low = quantile(values, low_q);
    const double q_high = quantile(values, high_q);
    const double spread = std::max(q_high - q_low, 1e-6);
    return std::pair<double, double>{q_low - spread * multiplier, q_high + spread * multiplier};
  };

  const auto [xmin, xmax] = bounds_for(xs, 0.01, 0.99, 10.0);
  const auto [ymin, ymax] = bounds_for(ys, 0.01, 0.99, 10.0);
  const auto [zmin, zmax] = bounds_for(zs, 0.01, 0.99, 5.0);

  /* 过滤超出范围的点 */
  std::vector<Point3D> filtered;
  filtered.reserve(cloud.points.size());
  std::size_t removed = 0;

  for (const auto& point : cloud.points) {
    if (point.x < xmin || point.x > xmax || point.y < ymin || point.y > ymax || point.z < zmin || point.z > zmax) {
      ++removed;
      continue;
    }
    filtered.push_back(point);
  }

  cloud.points = std::move(filtered);
  stats.setCount("removed_extreme_points", removed);
  logger.info("Removed obvious extreme points: " + std::to_string(removed));
}

/**
 * @brief 为点云中的所有点重新分配从 0 开始的连续索引号
 */
void PreprocessEngine::reindex(PointCloud& cloud) const {
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].index = static_cast<std::uint32_t>(i);
  }
}

/**
 * @brief 根据当前有效点集重新计算空间边界和估算点密度
 */
void PreprocessEngine::updateBoundsAndDensity(PointCloud& cloud) const {
  cloud.bounds = {};
  for (const auto& point : cloud.points) {
    cloud.bounds.expand(point.x, point.y, point.z);
  }
  const double area = std::max(1e-9, cloud.bounds.width() * cloud.bounds.height());
  cloud.estimated_density = static_cast<double>(cloud.points.size()) / area;
}

/**
 * @brief 通过随机采样方法估算点云的平均点间距
 *
 * 算法流程：
 * 1. 从点集中按固定步长采样 N 个点
 * 2. 对每个采样点搜索最近邻距离
 * 3. 取所有最近邻距离的均值作为平均点间距
 */
void PreprocessEngine::estimateSpacing(PointCloud& cloud, const DEMConfig& config) const {
  if (cloud.points.size() < 2) {
    cloud.estimated_avg_spacing = 0.0;
    return;
  }

  /* 按固定步长采样索引，并在采样点集上建立 2D KD-Tree */
  const std::size_t sample_count =
      std::min<std::size_t>(cloud.points.size(), std::max<std::size_t>(2U, config.preprocess.sample_spacing_count));
  const std::size_t step = std::max<std::size_t>(1, cloud.points.size() / sample_count);
  std::vector<std::size_t> sample_indices;
  sample_indices.reserve(sample_count);
  for (std::size_t i = 0; i < cloud.points.size() && sample_indices.size() < sample_count; i += step) {
    sample_indices.push_back(i);
  }
  if (sample_indices.size() < 2U) {
    cloud.estimated_avg_spacing = 0.0;
    return;
  }

  SpatialIndexManager index;
  index.build2D(cloud.points, sample_indices);

  /* 查询采样点的最近非自身邻居，避免“稀疏采样子集互找最近邻”带来的虚高间距 */
  double total_distance = 0.0;
  std::size_t counted = 0;
  for (const auto sample_idx : sample_indices) {
    const auto& point = cloud.points[sample_idx];
    const auto neighbors = index.knn2D(point.x, point.y, 2U);
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < neighbors.indices.size(); ++i) {
      if (neighbors.indices[i] == sample_idx || neighbors.distances[i] <= 1e-9) {
        continue;
      }
      nearest = neighbors.distances[i];
      break;
    }
    if (std::isfinite(nearest)) {
      total_distance += nearest;
      ++counted;
    }
  }
  cloud.estimated_avg_spacing = counted == 0 ? 0.0 : total_distance / static_cast<double>(counted);
}

}  // namespace dem
