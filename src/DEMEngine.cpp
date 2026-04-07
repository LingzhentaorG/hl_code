/**
 * @file DEMEngine.cpp
 * @brief DEM 直落格、对象掩膜、域构建、插值、补洞与最终 DTM 实现
 */

#include "dem/DEMEngine.hpp"

#include "dem/Utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <queue>
#include <vector>

namespace dem {
namespace {

std::size_t roundedShrinkCells(double boundary_outline_cells) {
  return static_cast<std::size_t>(std::max(0.0, std::ceil(boundary_outline_cells)));
}

bool isSupportCell(const RasterGrid& grid, std::size_t offset) {
  return offset < grid.valid_mask.size() && grid.valid_mask[offset] != 0U;
}

bool isValidValue(const RasterGrid& grid, std::size_t offset) {
  return offset < grid.values.size() && std::isfinite(grid.values[offset]) && grid.values[offset] != grid.nodata;
}

bool isMaskActive(const RasterGrid& grid, std::size_t offset) {
  return offset < grid.values.size() && std::isfinite(grid.values[offset]) && grid.values[offset] > 0.5;
}

double gridExtentDiagonal(const RasterGrid& grid) {
  const double width = std::max(0.0, static_cast<double>(grid.cols) * grid.cell_size);
  const double height = std::max(0.0, static_cast<double>(grid.rows) * grid.cell_size);
  return std::hypot(width, height);
}

RasterGrid makeBinaryMaskLike(const RasterGrid& source) {
  RasterGrid mask;
  mask.rows = source.rows;
  mask.cols = source.cols;
  mask.cell_size = source.cell_size;
  mask.origin_x = source.origin_x;
  mask.origin_y = source.origin_y;
  mask.nodata = -1.0;
  mask.values.assign(mask.size(), 0.0);
  mask.valid_mask.assign(mask.size(), 1U);
  mask.edge_mask.assign(mask.size(), 0U);
  return mask;
}

struct HullPoint {
  double x = 0.0;
  double y = 0.0;
};

double cross2D(const HullPoint& origin, const HullPoint& lhs, const HullPoint& rhs) {
  return (lhs.x - origin.x) * (rhs.y - origin.y) - (lhs.y - origin.y) * (rhs.x - origin.x);
}

std::vector<HullPoint> convexHullFromPointCloud(const PointCloud& cloud) {
  std::vector<HullPoint> points;
  points.reserve(cloud.points.size());
  for (const auto& point : cloud.points) {
    if (!point.valid || !std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    points.push_back({point.x, point.y});
  }

  if (points.empty()) {
    return {};
  }

  std::sort(points.begin(), points.end(), [](const HullPoint& lhs, const HullPoint& rhs) {
    if (lhs.x != rhs.x) {
      return lhs.x < rhs.x;
    }
    return lhs.y < rhs.y;
  });
  points.erase(std::unique(points.begin(), points.end(), [](const HullPoint& lhs, const HullPoint& rhs) {
                 return lhs.x == rhs.x && lhs.y == rhs.y;
               }),
               points.end());

  if (points.size() <= 1U) {
    return points;
  }

  std::vector<HullPoint> lower;
  lower.reserve(points.size());
  for (const auto& point : points) {
    while (lower.size() >= 2U && cross2D(lower[lower.size() - 2U], lower.back(), point) <= 0.0) {
      lower.pop_back();
    }
    lower.push_back(point);
  }

  std::vector<HullPoint> upper;
  upper.reserve(points.size());
  for (auto it = points.rbegin(); it != points.rend(); ++it) {
    while (upper.size() >= 2U && cross2D(upper[upper.size() - 2U], upper.back(), *it) <= 0.0) {
      upper.pop_back();
    }
    upper.push_back(*it);
  }

  lower.pop_back();
  upper.pop_back();
  lower.insert(lower.end(), upper.begin(), upper.end());
  return lower;
}

bool activateMaskCell(RasterGrid& mask, int row, int col) {
  if (row < 0 || col < 0 || row >= static_cast<int>(mask.rows) || col >= static_cast<int>(mask.cols)) {
    return false;
  }
  const auto offset = mask.offset(static_cast<std::size_t>(row), static_cast<std::size_t>(col));
  mask.values[offset] = 1.0;
  if (offset < mask.valid_mask.size()) {
    mask.valid_mask[offset] = 1U;
  }
  return true;
}

void activateMaskBrush(RasterGrid& mask, int row, int col, std::size_t thickness_cells) {
  if (thickness_cells == 0U) {
    return;
  }
  const int radius = static_cast<int>(thickness_cells) - 1;
  for (int dr = -radius; dr <= radius; ++dr) {
    for (int dc = -radius; dc <= radius; ++dc) {
      if (std::max(std::abs(dr), std::abs(dc)) > radius) {
        continue;
      }
      activateMaskCell(mask, row + dr, col + dc);
    }
  }
}

void rasterizeLineToMask(const HullPoint& start,
                         const HullPoint& end,
                         RasterGrid& mask,
                         std::size_t thickness_cells) {
  if (thickness_cells == 0U || mask.empty()) {
    return;
  }

  const double start_col = (start.x - mask.origin_x) / mask.cell_size - 0.5;
  const double start_row = (mask.origin_y - start.y) / mask.cell_size - 0.5;
  const double end_col = (end.x - mask.origin_x) / mask.cell_size - 0.5;
  const double end_row = (mask.origin_y - end.y) / mask.cell_size - 0.5;
  const double delta_col = end_col - start_col;
  const double delta_row = end_row - start_row;
  const auto steps = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(std::max(std::abs(delta_col), std::abs(delta_row)) * 2.0)));

  for (std::size_t i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const int row = static_cast<int>(std::llround(start_row + delta_row * t));
    const int col = static_cast<int>(std::llround(start_col + delta_col * t));
    activateMaskBrush(mask, row, col, thickness_cells);
  }
}

void rasterizeHullOutline(const std::vector<HullPoint>& hull, RasterGrid& mask, std::size_t thickness_cells) {
  if (thickness_cells == 0U || hull.empty() || mask.empty()) {
    return;
  }
  if (hull.size() == 1U) {
    const int col = static_cast<int>(std::floor((hull.front().x - mask.origin_x) / mask.cell_size));
    const int row = static_cast<int>(std::floor((mask.origin_y - hull.front().y) / mask.cell_size));
    activateMaskBrush(mask, row, col, thickness_cells);
    return;
  }

  for (std::size_t i = 0; i < hull.size(); ++i) {
    rasterizeLineToMask(hull[i], hull[(i + 1U) % hull.size()], mask, thickness_cells);
  }
}

void rasterizeFilledHull(const std::vector<HullPoint>& hull, RasterGrid& mask) {
  if (hull.empty() || mask.empty()) {
    return;
  }
  if (hull.size() <= 2U) {
    rasterizeHullOutline(hull, mask, 1U);
    return;
  }

  std::vector<double> intersections;
  intersections.reserve(hull.size());
  for (std::size_t row = 0; row < mask.rows; ++row) {
    intersections.clear();
    const double y = mask.cellCenterY(row);

    for (std::size_t i = 0; i < hull.size(); ++i) {
      const auto& start = hull[i];
      const auto& end = hull[(i + 1U) % hull.size()];
      if ((start.y <= y && y < end.y) || (end.y <= y && y < start.y)) {
        const double ratio = (y - start.y) / (end.y - start.y);
        intersections.push_back(start.x + ratio * (end.x - start.x));
      }
    }

    if (intersections.empty()) {
      continue;
    }
    std::sort(intersections.begin(), intersections.end());
    for (std::size_t i = 0; i + 1U < intersections.size(); i += 2U) {
      const double left = std::min(intersections[i], intersections[i + 1U]);
      const double right = std::max(intersections[i], intersections[i + 1U]);
      const auto start_col = static_cast<long long>(
          std::ceil((left - mask.origin_x) / mask.cell_size - 0.5 - 1e-9));
      const auto end_col = static_cast<long long>(
          std::floor((right - mask.origin_x) / mask.cell_size - 0.5 + 1e-9));
      for (long long col = std::max<long long>(0, start_col);
           col <= std::min<long long>(static_cast<long long>(mask.cols) - 1LL, end_col);
           ++col) {
        activateMaskCell(mask, static_cast<int>(row), static_cast<int>(col));
      }
    }
  }

  rasterizeHullOutline(hull, mask, 1U);
}

std::vector<Point3D> rasterPointsFromSupport(const RasterGrid& grid) {
  std::vector<Point3D> points;
  points.reserve(grid.size());
  for (std::size_t row = 0; row < grid.rows; ++row) {
    for (std::size_t col = 0; col < grid.cols; ++col) {
      const auto offset = grid.offset(row, col);
      if (!isSupportCell(grid, offset) || !isValidValue(grid, offset)) {
        continue;
      }
      Point3D point;
      point.x = grid.cellCenterX(col);
      point.y = grid.cellCenterY(row);
      point.z = grid.values[offset];
      point.index = static_cast<std::uint32_t>(points.size());
      points.push_back(point);
    }
  }
  return points;
}

RasterGrid initializeIdwGrid(const RasterGrid& support_grid) {
  RasterGrid grid = support_grid;
  std::fill(grid.values.begin(), grid.values.end(), grid.nodata);
  std::fill(grid.edge_mask.begin(), grid.edge_mask.end(), 0U);

  for (std::size_t offset = 0; offset < grid.size(); ++offset) {
    if (isSupportCell(support_grid, offset)) {
      grid.values[offset] = support_grid.values[offset];
    }
  }
  return grid;
}

QueryResult filterNeighborsByMaxDistance(const QueryResult& neighbors, double max_distance) {
  if (max_distance <= 0.0) {
    return neighbors;
  }

  QueryResult filtered;
  filtered.indices.reserve(neighbors.indices.size());
  filtered.distances.reserve(neighbors.distances.size());
  for (std::size_t i = 0; i < neighbors.indices.size() && i < neighbors.distances.size(); ++i) {
    if (neighbors.distances[i] > max_distance) {
      continue;
    }
    filtered.indices.push_back(neighbors.indices[i]);
    filtered.distances.push_back(neighbors.distances[i]);
  }
  return filtered;
}

bool assignIdwValue(RasterGrid& grid,
                    std::size_t offset,
                    const std::vector<Point3D>& support_points,
                    const QueryResult& neighbors,
                    double power) {
  if (neighbors.indices.empty()) {
    return false;
  }

  double numerator = 0.0;
  double denominator = 0.0;
  for (std::size_t i = 0; i < neighbors.indices.size() && i < neighbors.distances.size(); ++i) {
    const double distance = neighbors.distances[i];
    const double point_z = support_points[neighbors.indices[i]].z;
    if (distance < 1e-9) {
      grid.values[offset] = point_z;
      return true;
    }
    const double weight = 1.0 / std::pow(std::max(distance, 1e-6), power);
    numerator += point_z * weight;
    denominator += weight;
  }

  if (denominator <= 0.0) {
    return false;
  }
  grid.values[offset] = numerator / denominator;
  return true;
}

double degreesFromRiseRun(double rise, double run) {
  return std::atan2(std::abs(rise), std::max(run, 1e-6)) * 180.0 / std::numbers::pi;
}

bool classifySupportCandidate(double candidate_x,
                              double candidate_y,
                              double candidate_z,
                              const std::vector<Point3D>& support_points,
                              const std::vector<std::size_t>& neighbor_indices,
                              const DEMConfig& config) {
  if (neighbor_indices.size() < config.ground.min_ground_neighbors) {
    return false;
  }

  auto weighted_reference = [&](ReferenceMode mode, double& ref_z, double& slope_deg) {
    if (mode == ReferenceMode::LocalMin) {
      ref_z = std::numeric_limits<double>::infinity();
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (const auto idx : neighbor_indices) {
        const auto& point = support_points[idx];
        ref_z = std::min(ref_z, point.z);
        const double dx = candidate_x - point.x;
        const double dy = candidate_y - point.y;
        nearest_distance = std::min(nearest_distance, std::sqrt(dx * dx + dy * dy));
      }
      slope_deg = degreesFromRiseRun(candidate_z - ref_z, nearest_distance);
      return true;
    }

    double numerator = 0.0;
    double denominator = 0.0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (const auto idx : neighbor_indices) {
      const auto& point = support_points[idx];
      const double dx = candidate_x - point.x;
      const double dy = candidate_y - point.y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < 1e-6) {
        ref_z = point.z;
        slope_deg = 0.0;
        return true;
      }
      nearest_distance = std::min(nearest_distance, distance);
      const double weight = 1.0 / std::pow(distance, config.ground.distance_weight_power);
      numerator += point.z * weight;
      denominator += weight;
    }
    if (denominator <= 0.0) {
      return false;
    }
    ref_z = numerator / denominator;
    slope_deg = degreesFromRiseRun(candidate_z - ref_z, nearest_distance);
    return true;
  };

  double reference_z = 0.0;
  double slope_deg = 0.0;
  bool success = false;
  if (config.ground.reference_mode == ReferenceMode::LocalPlane && neighbor_indices.size() >= 4U) {
    const auto plane = fitPlaneLeastSquares(support_points, neighbor_indices);
    if (plane.success && plane.condition_indicator < 1e6) {
      reference_z = plane.a * candidate_x + plane.b * candidate_y + plane.c;
      slope_deg = plane.slope_deg;
      success = true;
    }
  }
  if (!success) {
    success = weighted_reference(config.ground.reference_fallback_mode, reference_z, slope_deg);
  }
  if (!success && config.ground.reference_mode != config.ground.reference_fallback_mode) {
    success = weighted_reference(config.ground.reference_mode, reference_z, slope_deg);
  }
  if (!success) {
    return false;
  }

  std::vector<double> heights;
  heights.reserve(neighbor_indices.size());
  for (const auto idx : neighbor_indices) {
    heights.push_back(support_points[idx].z);
  }
  const double neighborhood_median = heights.empty() ? std::numeric_limits<double>::infinity() : median(heights);
  const double local_height_diff = std::abs(candidate_z - reference_z);
  return local_height_diff <= config.ground.max_height_diff &&
         candidate_z <= neighborhood_median + config.ground.max_height_diff &&
         slope_deg <= config.ground.max_slope_deg;
}

}  // namespace

RasterGrid DEMEngine::createGrid(const Bounds& bounds, double cell_size, double nodata) const {
  if (!bounds.valid()) {
    throw std::runtime_error("Cannot create DEM grid from invalid bounds.");
  }

  RasterGrid grid;
  grid.cell_size = cell_size;
  grid.origin_x = bounds.xmin;
  grid.origin_y = bounds.ymax;
  grid.nodata = nodata;
  grid.cols = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(std::max(bounds.width(), cell_size) / cell_size)));
  grid.rows = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(std::max(bounds.height(), cell_size) / cell_size)));
  grid.values.assign(grid.size(), grid.nodata);
  grid.valid_mask.assign(grid.size(), 0U);
  grid.edge_mask.assign(grid.size(), 0U);
  return grid;
}

RasterGrid DEMEngine::createGrid(const Bounds& bounds, const DEMConfig& config) const {
  return createGrid(bounds, config.dem.cell_size, config.dem.nodata);
}

RasterGrid DEMEngine::rasterizeMinimum(const PointCloud& cloud, const RasterGrid& grid_template) const {
  RasterGrid grid = grid_template;
  std::fill(grid.values.begin(), grid.values.end(), grid.nodata);
  std::fill(grid.valid_mask.begin(), grid.valid_mask.end(), 0U);
  std::fill(grid.edge_mask.begin(), grid.edge_mask.end(), 0U);

  for (const auto& point : cloud.points) {
    const double col_f = std::floor((point.x - grid.origin_x) / grid.cell_size);
    const double row_f = std::floor((grid.origin_y - point.y) / grid.cell_size);
    if (col_f < 0.0 || row_f < 0.0) {
      continue;
    }
    const auto col = static_cast<std::size_t>(col_f);
    const auto row = static_cast<std::size_t>(row_f);
    if (row >= grid.rows || col >= grid.cols) {
      continue;
    }
    const auto offset = grid.offset(row, col);
    if (grid.valid_mask[offset] == 0U || point.z < grid.values[offset]) {
      grid.values[offset] = point.z;
    }
    grid.valid_mask[offset] = 1U;
  }
  return grid;
}

RasterGrid DEMEngine::refineAcceptedSupport(const RasterGrid& raw_direct,
                                            const RasterGrid& ground_direct,
                                            const DEMConfig& config,
                                            Logger& logger,
                                            ProcessStats& stats,
                                            const RasterGrid* object_mask) const {
  ScopedTimer timer(stats, "dem_support_refine_seconds");
  RasterGrid accepted = ground_direct;
  if (object_mask != nullptr) {
    for (std::size_t offset = 0; offset < accepted.size() && offset < object_mask->size(); ++offset) {
      if (!isMaskActive(*object_mask, offset)) {
        continue;
      }
      accepted.values[offset] = accepted.nodata;
      accepted.valid_mask[offset] = 0U;
    }
  }

  const auto support_points = rasterPointsFromSupport(accepted);
  if (support_points.size() < config.ground.min_ground_neighbors) {
    logger.warn("Ground direct support is too sparse for supplemental support; using direct support only.");
    return accepted;
  }

  SpatialIndexManager index;
  std::vector<std::size_t> indices(support_points.size());
  std::iota(indices.begin(), indices.end(), 0U);
  index.build2D(support_points, indices);

  const std::size_t max_results =
      std::max<std::size_t>({config.ground.knn * 8U, config.ground.min_ground_neighbors * 4U, 32U});
  std::size_t added = 0U;

  for (std::size_t row = 0; row < raw_direct.rows; ++row) {
    for (std::size_t col = 0; col < raw_direct.cols; ++col) {
      const auto offset = raw_direct.offset(row, col);
      if (!isValidValue(raw_direct, offset) || isSupportCell(accepted, offset) ||
          (object_mask != nullptr && object_mask->size() == raw_direct.size() && isMaskActive(*object_mask, offset))) {
        continue;
      }

      const double x = raw_direct.cellCenterX(col);
      const double y = raw_direct.cellCenterY(row);
      auto neighbors = index.radius2D(x, y, config.ground.search_radius, max_results);
      if (neighbors.indices.size() < config.ground.min_ground_neighbors) {
        continue;
      }
      if (neighbors.indices.size() > config.ground.knn) {
        neighbors.indices.resize(config.ground.knn);
        neighbors.distances.resize(config.ground.knn);
      }
      if (occupiedSectors8(x, y, support_points, neighbors.indices) < 2U) {
        continue;
      }
      if (!classifySupportCandidate(x, y, raw_direct.values[offset], support_points, neighbors.indices, config)) {
        continue;
      }

      accepted.values[offset] = raw_direct.values[offset];
      accepted.valid_mask[offset] = 1U;
      ++added;
    }
  }

  stats.setCount("accepted_support_added_count", added);
  logger.info("Accepted support refinement complete.");
  return accepted;
}

RasterGrid DEMEngine::buildSupportMask(const RasterGrid& source) const {
  RasterGrid mask = makeBinaryMaskLike(source);
  for (std::size_t offset = 0; offset < mask.size(); ++offset) {
    mask.values[offset] = isSupportCell(source, offset) ? 1.0 : 0.0;
  }
  return mask;
}

RasterGrid DEMEngine::buildObjectMask(const RasterGrid& raw_direct,
                                      const RasterGrid& ground_direct,
                                      const DEMConfig& config,
                                      Logger& logger,
                                      ProcessStats& stats) const {
  ScopedTimer timer(stats, "dem_object_mask_seconds");
  RasterGrid object_mask = makeBinaryMaskLike(raw_direct);
  const auto support_points = rasterPointsFromSupport(ground_direct);
  if (support_points.size() < config.ground.min_ground_neighbors) {
    logger.warn("Ground direct support is too sparse for object masking; object mask remains empty.");
    return object_mask;
  }

  SpatialIndexManager index;
  std::vector<std::size_t> indices(support_points.size());
  std::iota(indices.begin(), indices.end(), 0U);
  index.build2D(support_points, indices);

  const std::size_t max_results =
      std::max<std::size_t>({config.ground.knn * 8U, config.ground.min_ground_neighbors * 4U, 32U});
  const double residual_threshold = std::max(1.5, config.ground.max_height_diff * 2.0);

  for (std::size_t row = 0; row < raw_direct.rows; ++row) {
    for (std::size_t col = 0; col < raw_direct.cols; ++col) {
      const auto offset = raw_direct.offset(row, col);
      if (!isValidValue(raw_direct, offset)) {
        continue;
      }

      if (isValidValue(ground_direct, offset) &&
          raw_direct.values[offset] - ground_direct.values[offset] > residual_threshold) {
        object_mask.values[offset] = 1.0;
        continue;
      }

      const double x = raw_direct.cellCenterX(col);
      const double y = raw_direct.cellCenterY(row);
      auto neighbors = index.radius2D(x, y, config.ground.search_radius, max_results);
      if (neighbors.indices.size() < config.ground.min_ground_neighbors) {
        continue;
      }
      if (neighbors.indices.size() > config.ground.knn) {
        neighbors.indices.resize(config.ground.knn);
        neighbors.distances.resize(config.ground.knn);
      }
      if (occupiedSectors8(x, y, support_points, neighbors.indices) < 2U) {
        continue;
      }

      std::vector<double> heights;
      heights.reserve(neighbors.indices.size());
      double local_min = std::numeric_limits<double>::infinity();
      for (const auto neighbor_index : neighbors.indices) {
        heights.push_back(support_points[neighbor_index].z);
        local_min = std::min(local_min, support_points[neighbor_index].z);
      }

      const double local_median = median(heights);
      const double raw_z = raw_direct.values[offset];
      if (classifySupportCandidate(x, y, raw_z, support_points, neighbors.indices, config)) {
        continue;
      }
      if (raw_z > local_median + residual_threshold && raw_z > local_min + residual_threshold) {
        object_mask.values[offset] = 1.0;
      }
    }
  }
  logger.info("Object mask generation complete.");
  return object_mask;
}

RasterGrid DEMEngine::buildDomainMask(const PointCloud& boundary_cloud,
                                      const RasterGrid& grid_template,
                                      const DEMConfig& config,
                                      Logger& logger,
                                      ProcessStats& stats) const {
  ScopedTimer timer(stats, "dem_domain_mask_seconds");
  (void)config;
  RasterGrid mask = makeBinaryMaskLike(grid_template);
  const auto hull = convexHullFromPointCloud(boundary_cloud);
  stats.setCount("dem_hull_vertex_count", hull.size());
  if (hull.empty()) {
    logger.warn("Filtered points are empty; DEM domain mask remains empty.");
    return mask;
  }

  rasterizeFilledHull(hull, mask);
  logger.info("DEM domain mask generation complete.");
  return mask;
}

RasterGrid DEMEngine::interpolateIdw(const RasterGrid& support_grid,
                                     const RasterGrid& domain_mask,
                                     const DEMConfig& config,
                                     Logger& logger,
                                     ProcessStats& stats) const {
  ScopedTimer timer(stats, "dem_idw_seconds");
  RasterGrid grid = initializeIdwGrid(support_grid);
  const auto support_points = rasterPointsFromSupport(support_grid);
  if (support_points.empty()) {
    logger.warn("Support grid is empty; DEM remains nodata.");
    return grid;
  }

  SpatialIndexManager index;
  std::vector<std::size_t> indices(support_points.size());
  std::iota(indices.begin(), indices.end(), 0U);
  index.build2D(support_points, indices);
  const double effective_max_distance =
      config.dem.idw_max_distance > 0.0 ? config.dem.idw_max_distance : gridExtentDiagonal(grid);

  for (std::size_t row = 0; row < grid.rows; ++row) {
    for (std::size_t col = 0; col < grid.cols; ++col) {
      const auto offset = grid.offset(row, col);
      if (!isMaskActive(domain_mask, offset) || isSupportCell(support_grid, offset)) {
        continue;
      }

      QueryResult neighbors = index.knn2D(grid.cellCenterX(col), grid.cellCenterY(row), config.dem.idw_k);
      if (effective_max_distance > 0.0) {
        neighbors = filterNeighborsByMaxDistance(neighbors, effective_max_distance);
      }
      if (neighbors.indices.empty()) {
        continue;
      }
      if (neighbors.indices.size() < config.dem.idw_min_points && !config.dem.idw_allow_fallback) {
        continue;
      }
      assignIdwValue(grid, offset, support_points, neighbors, config.dem.idw_power);
    }
  }

  logger.info("Official DEM interpolation complete.");
  return grid;
}

RasterGrid DEMEngine::buildEdgeMask(const PointCloud& boundary_cloud,
                                    const RasterGrid& grid_template,
                                    const DEMConfig& config,
                                    Logger& logger,
                                    ProcessStats& stats) const {
  ScopedTimer timer(stats, "dem_boundary_mask_seconds");
  RasterGrid edge = makeBinaryMaskLike(grid_template);
  const auto hull = convexHullFromPointCloud(boundary_cloud);
  stats.setCount("dem_hull_vertex_count", hull.size());
  const auto thickness_cells = roundedShrinkCells(config.dem.boundary_outline_cells);
  rasterizeHullOutline(hull, edge, thickness_cells);
  logger.info("DEM boundary mask generation complete.");
  return edge;
}

}  // namespace dem
