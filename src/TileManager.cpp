/**
 * @file TileManager.cpp
 * @brief 分块管理器实现：大点云数据的内存感知自动分块处理
 */

#include "dem/TileManager.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dem {

/**
 * @brief 根据内存限制和点云范围计算最优分块方案
 *
 * 分块策略：
 * 1. 估算每个点的内存占用（基础开销 + 属性数 × 单属性大小）
 * 2. 计算在内存限制下可容纳的最大点数
 * 3. 根据点云密度和最大点数反推每个分块的地理尺寸
 * 4. 将完整范围划分为规则网格，确保每个分块不超过内存限制
 *
 * 返回的分块方案包含各分块的行列号和空间边界。
 */
TilePlan TileManager::calculateTilePlan(const BoundingBox3D& bounds,
                                        double point_density,
                                        std::size_t raw_point_count,
                                        const TileConfig& tile_config) const {
  TilePlan plan;
  plan.total_raw_count = raw_point_count;

  /* 估算单点内存占用（字节）= 基础开销 + 各属性字段大小 */
  const double bytes_per_point =
      tile_config.base_memory_per_point +
      static_cast<double>(tile_config.property_count) * tile_config.bytes_per_property;

  /* 在内存限制下可容纳的最大点数（预留 20% 安全余量） */
  const std::size_t max_points_per_tile =
      static_cast<std::size_t>(static_cast<double>(tile_config.memory_limit_mb) * 1024.0 * 1024.0 /
                               (bytes_per_point * 1.2));

  plan.max_points_per_tile = max_points_per_tile;
  plan.bytes_per_point_estimate = bytes_per_point;

  /* 根据点密度和最大点数计算单个分块的边长 */
  const double area_per_tile = static_cast<double>(max_points_per_tile) / std::max(point_density, 1e-9);
  const double tile_side = std::sqrt(area_per_tile);
  plan.tile_size = tile_side;

  /* 计算所需的行数和列数以覆盖完整范围 */
  const double width = bounds.width();
  const double height = bounds.height();
  const int cols = std::max(1, static_cast<int>(std::ceil(width / tile_side)));
  const int rows = std::max(1, static_cast<int>(std::ceil(height / tile_side)));

  plan.cols = cols;
  plan.rows = rows;

  /* 为每个分块计算精确的空间边界（考虑边缘对齐） */
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      TileInfo info;
      info.row = row;   /**< 行索引 */
      info.col = col;   /**< 列索引 */

      /* 计算该分块的 X/Y 范围（最后一个分块延伸到边界） */
      info.xmin = bounds.xmin + static_cast<double>(col) * tile_side;
      info.ymin = bounds.ymin + static_cast<double>(row) * tile_side;
      info.xmin = (col == cols - 1) ? bounds.xmax : info.xmin + tile_side;
      info.ymin = (row == rows - 1) ? bounds.ymax : info.ymin + tile_side;

      plan.tiles.push_back(info);
    }
  }

  return plan;
}

/**
 * @brief 从完整点云中提取指定空间范围内的子集
 *
 * 使用简单的边界框包含测试进行过滤：
 * 仅保留 x ∈ [xmin, xmax] 且 y ∈ [ymin, ymax] 的点。
 * Z 坐标不参与空间过滤但随点一起保留。
 */
PointCloud TileManager::extractTile(const PointCloud& full_cloud, const TileInfo& tile_info) const {
  PointCloud tile_cloud;
  tile_cloud.raw_point_count = full_cloud.raw_point_count;
  tile_cloud.property_names = full_cloud.property_names;
  tile_cloud.points.reserve(full_cloud.points.size() / 10);   /* 预估约 10% 的点落入此分块 */

  for (const auto& point : full_cloud.points) {
    if (point.x >= tile_info.xmin && point.x <= tile_info.xmax &&
        point.y >= tile_info.ymin && point.y <= tile_info.ymax) {
      tile_cloud.points.push_back(point);
      tile_cloud.bounds.expand(point.x, point.y, point.z);
    }
  }

  tile_cloud.stored_point_count = tile_cloud.points.size();
  return tile_cloud;
}

/**
 * @brief 合并多个分块的 DEM 栅格为单一输出栅格
 *
 * 合并策略：
 * 1. 以第一个有效栅格为基准确定全局网格参数
 * 2. 后续栅格按像素位置逐个覆盖写入（后者覆盖前者）
 * 3. 统一使用第一个栅格的 NoData 值
 *
 * 注意：要求所有输入栅格具有相同的分辨率。
 */
DEMRaster TileManager::mergeTiles(const std::vector<DEMRaster>& tiles) const {
  if (tiles.empty()) {
    throw std::runtime_error("No tiles to merge.");
  }

  /* 找到第一个非空栅格作为合并基准 */
  DEMRaster merged = {};
  bool found_base = false;
  for (const auto& tile : tiles) {
    if (!tile.data.empty()) {
      merged = tile;
      found_base = true;
      break;
    }
  }
  if (!found_base) {
    throw std::runtime_error("All tiles are empty.");
  }

  /* 逐个合并后续栅格的数据 */
  for (std::size_t i = 1; i < tiles.size(); ++i) {
    const auto& tile = tiles[i];
    if (tile.data.empty()) { continue; }   /* 跳过空栅格 */

    /* 检查分辨率一致性 */
    if (std::abs(tile.resolution - merged.resolution) > 1e-6) {
      throw std::runtime_error("Tile resolution mismatch during merge: " +
                               std::to_string(merged.resolution) + " vs " + std::to_string(tile.resolution));
    }

    /* 按像素位置逐个覆盖合并数据 */
    for (std::size_t row = 0; row < tile.rows && row < merged.rows; ++row) {
      for (std::size_t col = 0; col < tile.cols && col < merged.cols; ++col) {
        const auto src_idx = row * tile.cols + col;
        const auto dst_idx = row * merged.cols + col;
        if (tile.data[src_idx] != tile.nodata_value) {
          merged.data[dst_idx] = tile.data[src_idx];   /* 有效值覆盖 NoData 或旧值 */
        }
      }
    }
  }

  return merged;
}

}  // namespace dem
