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

std::size_t roundedShrinkCells(double edge_shrink_cells) {
  return static_cast<std::size_t>(std::max(0.0, std::ceil(edge_shrink_cells)));
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

std::vector<Point3D> rasterPointsFromValidValues(const RasterGrid& grid, const RasterGrid* mask = nullptr) {
  std::vector<Point3D> points;
  points.reserve(grid.size());
  for (std::size_t row = 0; row < grid.rows; ++row) {
    for (std::size_t col = 0; col < grid.cols; ++col) {
      const auto offset = grid.offset(row, col);
      if ((mask != nullptr && !isMaskActive(*mask, offset)) || !isValidValue(grid, offset)) {
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

RasterGrid dilateMask(const RasterGrid& source, int radius) {
  if (radius <= 0) {
    return source;
  }
  RasterGrid dilated = makeBinaryMaskLike(source);
  for (std::size_t row = 0; row < source.rows; ++row) {
    for (std::size_t col = 0; col < source.cols; ++col) {
      bool active = false;
      for (int dr = -radius; dr <= radius && !active; ++dr) {
        for (int dc = -radius; dc <= radius; ++dc) {
          if (std::max(std::abs(dr), std::abs(dc)) > radius) {
            continue;
          }
          const int nr = static_cast<int>(row) + dr;
          const int nc = static_cast<int>(col) + dc;
          if (nr < 0 || nc < 0 || nr >= static_cast<int>(source.rows) || nc >= static_cast<int>(source.cols)) {
            continue;
          }
          if (isMaskActive(source, source.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc)))) {
            active = true;
            break;
          }
        }
      }
      dilated.values[dilated.offset(row, col)] = active ? 1.0 : 0.0;
    }
  }
  return dilated;
}

RasterGrid erodeMask(const RasterGrid& source, int radius, bool keep_canvas_border = false) {
  if (radius <= 0) {
    return source;
  }
  RasterGrid eroded = makeBinaryMaskLike(source);
  for (std::size_t row = 0; row < source.rows; ++row) {
    for (std::size_t col = 0; col < source.cols; ++col) {
      bool active = true;
      for (int dr = -radius; dr <= radius && active; ++dr) {
        for (int dc = -radius; dc <= radius; ++dc) {
          if (std::max(std::abs(dr), std::abs(dc)) > radius) {
            continue;
          }
          const int nr = static_cast<int>(row) + dr;
          const int nc = static_cast<int>(col) + dc;
          if (nr < 0 || nc < 0 || nr >= static_cast<int>(source.rows) || nc >= static_cast<int>(source.cols)) {
            if (!keep_canvas_border) {
              active = false;
            }
            break;
          }
          if (!isMaskActive(source, source.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc)))) {
            active = false;
            break;
          }
        }
      }
      eroded.values[eroded.offset(row, col)] = active ? 1.0 : 0.0;
    }
  }
  return eroded;
}

void fillSmallClosedHoles(RasterGrid& mask, std::size_t max_area) {
  if (mask.empty() || max_area == 0U) {
    return;
  }

  std::vector<std::uint8_t> visited(mask.size(), 0U);
  const std::array<int, 5> steps{-1, 0, 1, 0, -1};

  for (std::size_t row = 0; row < mask.rows; ++row) {
    for (std::size_t col = 0; col < mask.cols; ++col) {
      const auto start = mask.offset(row, col);
      if (visited[start] != 0U || isMaskActive(mask, start)) {
        continue;
      }

      std::queue<std::pair<std::size_t, std::size_t>> queue;
      std::vector<std::size_t> component;
      bool touches_border = false;
      visited[start] = 1U;
      queue.push({row, col});

      while (!queue.empty()) {
        const auto [current_row, current_col] = queue.front();
        queue.pop();
        component.push_back(mask.offset(current_row, current_col));
        touches_border = touches_border || current_row == 0 || current_col == 0 ||
                         current_row + 1 == mask.rows || current_col + 1 == mask.cols;

        for (std::size_t step = 0; step < 4; ++step) {
          const int nr = static_cast<int>(current_row) + steps[step];
          const int nc = static_cast<int>(current_col) + steps[step + 1];
          if (nr < 0 || nc < 0 || nr >= static_cast<int>(mask.rows) || nc >= static_cast<int>(mask.cols)) {
            continue;
          }
          const auto next = mask.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc));
          if (visited[next] != 0U || isMaskActive(mask, next)) {
            continue;
          }
          visited[next] = 1U;
          queue.push({static_cast<std::size_t>(nr), static_cast<std::size_t>(nc)});
        }
      }

      if (!touches_border && component.size() <= max_area) {
        for (const auto offset : component) {
          mask.values[offset] = 1.0;
        }
      }
    }
  }
}

void fillInteriorHoles(RasterGrid& mask) {
  if (mask.empty()) {
    return;
  }

  std::vector<std::uint8_t> visited(mask.size(), 0U);
  const std::array<int, 5> steps{-1, 0, 1, 0, -1};

  for (std::size_t row = 0; row < mask.rows; ++row) {
    for (std::size_t col = 0; col < mask.cols; ++col) {
      const auto start = mask.offset(row, col);
      if (visited[start] != 0U || isMaskActive(mask, start)) {
        continue;
      }

      std::queue<std::pair<std::size_t, std::size_t>> queue;
      std::vector<std::size_t> component;
      bool touches_border = false;
      visited[start] = 1U;
      queue.push({row, col});

      while (!queue.empty()) {
        const auto [current_row, current_col] = queue.front();
        queue.pop();
        component.push_back(mask.offset(current_row, current_col));
        touches_border = touches_border || current_row == 0 || current_col == 0 ||
                         current_row + 1 == mask.rows || current_col + 1 == mask.cols;

        for (std::size_t step = 0; step < 4; ++step) {
          const int nr = static_cast<int>(current_row) + steps[step];
          const int nc = static_cast<int>(current_col) + steps[step + 1];
          if (nr < 0 || nc < 0 || nr >= static_cast<int>(mask.rows) || nc >= static_cast<int>(mask.cols)) {
            continue;
          }
          const auto next = mask.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc));
          if (visited[next] != 0U || isMaskActive(mask, next)) {
            continue;
          }
          visited[next] = 1U;
          queue.push({static_cast<std::size_t>(nr), static_cast<std::size_t>(nc)});
        }
      }

      if (!touches_border) {
        for (const auto offset : component) {
          mask.values[offset] = 1.0;
        }
      }
    }
  }
}

void keepMajorComponents(RasterGrid& mask, std::size_t min_area) {
  if (mask.empty()) {
    return;
  }

  std::vector<std::uint8_t> visited(mask.size(), 0U);
  const std::array<int, 5> steps{-1, 0, 1, 0, -1};
  std::vector<std::vector<std::size_t>> components;
  std::size_t largest_index = 0U;
  std::size_t largest_area = 0U;

  for (std::size_t row = 0; row < mask.rows; ++row) {
    for (std::size_t col = 0; col < mask.cols; ++col) {
      const auto start = mask.offset(row, col);
      if (visited[start] != 0U || !isMaskActive(mask, start)) {
        continue;
      }

      std::queue<std::pair<std::size_t, std::size_t>> queue;
      std::vector<std::size_t> component;
      visited[start] = 1U;
      queue.push({row, col});

      while (!queue.empty()) {
        const auto [current_row, current_col] = queue.front();
        queue.pop();
        component.push_back(mask.offset(current_row, current_col));
        for (std::size_t step = 0; step < 4; ++step) {
          const int nr = static_cast<int>(current_row) + steps[step];
          const int nc = static_cast<int>(current_col) + steps[step + 1];
          if (nr < 0 || nc < 0 || nr >= static_cast<int>(mask.rows) || nc >= static_cast<int>(mask.cols)) {
            continue;
          }
          const auto next = mask.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc));
          if (visited[next] != 0U || !isMaskActive(mask, next)) {
            continue;
          }
          visited[next] = 1U;
          queue.push({static_cast<std::size_t>(nr), static_cast<std::size_t>(nc)});
        }
      }

      if (component.size() > largest_area) {
        largest_area = component.size();
        largest_index = components.size();
      }
      components.push_back(std::move(component));
    }
  }

  std::fill(mask.values.begin(), mask.values.end(), 0.0);
  for (std::size_t i = 0; i < components.size(); ++i) {
    if (components[i].size() >= min_area || i == largest_index) {
      for (const auto offset : components[i]) {
        mask.values[offset] = 1.0;
      }
    }
  }
}

void applyMaskExclusion(RasterGrid& grid, const RasterGrid* object_mask) {
  if (object_mask == nullptr || object_mask->size() != grid.size()) {
    return;
  }
  for (std::size_t offset = 0; offset < grid.size(); ++offset) {
    if (!isMaskActive(*object_mask, offset)) {
      continue;
    }
    grid.values[offset] = 0.0;
    if (offset < grid.valid_mask.size()) {
      grid.valid_mask[offset] = 0U;
    }
  }
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

void clipMaskToSupportRadius(RasterGrid& mask, const RasterGrid& support, double max_distance) {
  if (max_distance <= 0.0) {
    return;
  }
  const auto support_points = rasterPointsFromSupport(support);
  if (support_points.empty()) {
    std::fill(mask.values.begin(), mask.values.end(), 0.0);
    return;
  }

  SpatialIndexManager index;
  std::vector<std::size_t> indices(support_points.size());
  std::iota(indices.begin(), indices.end(), 0U);
  index.build2D(support_points, indices);

  for (std::size_t row = 0; row < mask.rows; ++row) {
    for (std::size_t col = 0; col < mask.cols; ++col) {
      const auto offset = mask.offset(row, col);
      if (!isMaskActive(mask, offset)) {
        continue;
      }
      const auto neighbors = index.knn2D(mask.cellCenterX(col), mask.cellCenterY(row), 1U);
      if (neighbors.indices.empty() || neighbors.distances.front() > max_distance) {
        mask.values[offset] = 0.0;
      }
    }
  }
}

bool hasOriginalSupportWithinRadius(const RasterGrid& source,
                                    const RasterGrid* object_mask,
                                    std::size_t row,
                                    std::size_t col,
                                    int max_radius) {
  if (max_radius < 0) {
    return false;
  }
  for (int dr = -max_radius; dr <= max_radius; ++dr) {
    for (int dc = -max_radius; dc <= max_radius; ++dc) {
      if (std::max(std::abs(dr), std::abs(dc)) > max_radius) {
        continue;
      }
      const int nr = static_cast<int>(row) + dr;
      const int nc = static_cast<int>(col) + dc;
      if (nr < 0 || nc < 0 || nr >= static_cast<int>(source.rows) || nc >= static_cast<int>(source.cols)) {
        continue;
      }
      const auto offset = source.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc));
      if (object_mask != nullptr && object_mask->size() == source.size() && isMaskActive(*object_mask, offset)) {
        continue;
      }
      if (isValidValue(source, offset)) {
        return true;
      }
    }
  }
  return false;
}

void markNeighbors(std::size_t row, std::size_t col, const RasterGrid& domain_mask, RasterGrid& edge_mask) {
  for (int dr = -1; dr <= 1; ++dr) {
    for (int dc = -1; dc <= 1; ++dc) {
      const int nr = static_cast<int>(row) + dr;
      const int nc = static_cast<int>(col) + dc;
      if (nr < 0 || nc < 0 || nr >= static_cast<int>(domain_mask.rows) || nc >= static_cast<int>(domain_mask.cols)) {
        continue;
      }
      const auto offset = domain_mask.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc));
      if (isMaskActive(domain_mask, offset)) {
        edge_mask.values[offset] = 1.0;
      }
    }
  }
}

RasterGrid regularizeSupportSurface(const RasterGrid& support_grid, const RasterGrid& object_mask, const DEMConfig& config) {
  RasterGrid regularized = support_grid;
  for (std::size_t row = 0; row < support_grid.rows; ++row) {
    for (std::size_t col = 0; col < support_grid.cols; ++col) {
      const auto offset = support_grid.offset(row, col);
      if (!isSupportCell(support_grid, offset) || !isValidValue(support_grid, offset) || isMaskActive(object_mask, offset)) {
        continue;
      }

      std::vector<double> neighbors;
      neighbors.reserve(9);
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          const int nr = static_cast<int>(row) + dr;
          const int nc = static_cast<int>(col) + dc;
          if (nr < 0 || nc < 0 || nr >= static_cast<int>(support_grid.rows) ||
              nc >= static_cast<int>(support_grid.cols)) {
            continue;
          }
          const auto neighbor_offset = support_grid.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc));
          if (!isSupportCell(support_grid, neighbor_offset) || !isValidValue(support_grid, neighbor_offset) ||
              isMaskActive(object_mask, neighbor_offset)) {
            continue;
          }
          neighbors.push_back(support_grid.values[neighbor_offset]);
        }
      }

      if (neighbors.size() < 4U) {
        continue;
      }
      const double local_median = median(neighbors);
      if (std::abs(support_grid.values[offset] - local_median) > config.ground.max_height_diff) {
        regularized.values[offset] = local_median;
      }
    }
  }
  return regularized;
}

RasterGrid buildDisplayDomainMask(const RasterGrid& domain_mask, const DEMConfig& config) {
  RasterGrid display_domain = domain_mask;
  fillInteriorHoles(display_domain);
  fillSmallClosedHoles(
      display_domain,
      std::max<std::size_t>(25U, static_cast<std::size_t>((2 * std::max(1, config.dem.fill_max_radius) + 1) *
                                                          (2 * std::max(1, config.dem.fill_max_radius) + 1))));
  keepMajorComponents(display_domain,
                      std::max<std::size_t>(25U, static_cast<std::size_t>((std::max(1, config.dem.fill_max_radius) + 1) *
                                                                          (std::max(1, config.dem.fill_max_radius) + 1))));
  return display_domain;
}

std::size_t countRemainingNodataInMask(const RasterGrid& grid, const RasterGrid& mask) {
  std::size_t count = 0U;
  for (std::size_t offset = 0; offset < grid.size() && offset < mask.size(); ++offset) {
    if (!isMaskActive(mask, offset) || isValidValue(grid, offset)) {
      continue;
    }
    ++count;
  }
  return count;
}

RasterGrid downsampleMeanRaster(const RasterGrid& source, const RasterGrid& domain_mask, const RasterGrid& object_mask) {
  RasterGrid coarse;
  coarse.rows = std::max<std::size_t>(1U, (source.rows + 1U) / 2U);
  coarse.cols = std::max<std::size_t>(1U, (source.cols + 1U) / 2U);
  coarse.cell_size = source.cell_size * 2.0;
  coarse.origin_x = source.origin_x;
  coarse.origin_y = source.origin_y;
  coarse.nodata = source.nodata;
  coarse.values.assign(coarse.size(), coarse.nodata);
  coarse.valid_mask.assign(coarse.size(), 0U);
  coarse.edge_mask.assign(coarse.size(), 0U);

  for (std::size_t coarse_row = 0; coarse_row < coarse.rows; ++coarse_row) {
    for (std::size_t coarse_col = 0; coarse_col < coarse.cols; ++coarse_col) {
      double sum = 0.0;
      std::size_t count = 0U;
      for (std::size_t dr = 0; dr < 2U; ++dr) {
        for (std::size_t dc = 0; dc < 2U; ++dc) {
          const std::size_t row = coarse_row * 2U + dr;
          const std::size_t col = coarse_col * 2U + dc;
          if (row >= source.rows || col >= source.cols) {
            continue;
          }
          const auto offset = source.offset(row, col);
          if (!isMaskActive(domain_mask, offset) || isMaskActive(object_mask, offset) || !isValidValue(source, offset)) {
            continue;
          }
          sum += source.values[offset];
          ++count;
        }
      }
      if (count == 0U) {
        continue;
      }
      const auto coarse_offset = coarse.offset(coarse_row, coarse_col);
      coarse.values[coarse_offset] = sum / static_cast<double>(count);
      coarse.valid_mask[coarse_offset] = 1U;
    }
  }
  return coarse;
}

RasterGrid downsampleMask(const RasterGrid& source) {
  RasterGrid coarse;
  coarse.rows = std::max<std::size_t>(1U, (source.rows + 1U) / 2U);
  coarse.cols = std::max<std::size_t>(1U, (source.cols + 1U) / 2U);
  coarse.cell_size = source.cell_size * 2.0;
  coarse.origin_x = source.origin_x;
  coarse.origin_y = source.origin_y;
  coarse.nodata = -1.0;
  coarse.values.assign(coarse.size(), 0.0);
  coarse.valid_mask.assign(coarse.size(), 1U);
  coarse.edge_mask.assign(coarse.size(), 0U);

  for (std::size_t coarse_row = 0; coarse_row < coarse.rows; ++coarse_row) {
    for (std::size_t coarse_col = 0; coarse_col < coarse.cols; ++coarse_col) {
      bool active = false;
      for (std::size_t dr = 0; dr < 2U && !active; ++dr) {
        for (std::size_t dc = 0; dc < 2U; ++dc) {
          const std::size_t row = coarse_row * 2U + dr;
          const std::size_t col = coarse_col * 2U + dc;
          if (row >= source.rows || col >= source.cols) {
            continue;
          }
          active = isMaskActive(source, source.offset(row, col));
          if (active) {
            break;
          }
        }
      }
      coarse.values[coarse.offset(coarse_row, coarse_col)] = active ? 1.0 : 0.0;
    }
  }
  return coarse;
}

RasterGrid upsampleInto(const RasterGrid& source, const RasterGrid& coarse, const RasterGrid& domain_mask, const RasterGrid& object_mask) {
  RasterGrid result = source;
  for (std::size_t row = 0; row < result.rows; ++row) {
    for (std::size_t col = 0; col < result.cols; ++col) {
      const auto offset = result.offset(row, col);
      if (!isMaskActive(domain_mask, offset) || isMaskActive(object_mask, offset) || isValidValue(result, offset)) {
        continue;
      }
      const auto coarse_row = std::min(coarse.rows - 1U, row / 2U);
      const auto coarse_col = std::min(coarse.cols - 1U, col / 2U);
      const auto coarse_offset = coarse.offset(coarse_row, coarse_col);
      if (!isValidValue(coarse, coarse_offset)) {
        continue;
      }
      result.values[offset] = coarse.values[coarse_offset];
    }
  }
  return result;
}

RasterGrid coarseToFineFill(const RasterGrid& source,
                            const RasterGrid& domain_mask,
                            const RasterGrid& object_mask,
                            const DEMConfig& config,
                            Logger& logger,
                            ProcessStats& stats,
                            int levels_left) {
  if (levels_left <= 0 || source.rows < 3U || source.cols < 3U) {
    return source;
  }

  RasterGrid coarse_domain = downsampleMask(domain_mask);
  RasterGrid coarse_object = downsampleMask(object_mask);
  RasterGrid coarse_grid = downsampleMeanRaster(source, domain_mask, object_mask);
  if (coarse_grid.empty()) {
    return source;
  }

  DEMConfig coarse_config = config;
  coarse_config.dem.cell_size *= 2.0;
  coarse_config.ground.seed_grid_size *= 2.0;
  coarse_config.ground.search_radius *= 2.0;
  coarse_config.dem.nearest_max_distance *= 2.0;
  coarse_config.dem.idw_radius *= 2.0;

  DEMEngine engine;
  RasterGrid coarse_filled = engine.fillHoles(coarse_grid, coarse_domain, coarse_config, logger, stats, &coarse_object);
  coarse_filled = coarseToFineFill(coarse_filled, coarse_domain, coarse_object, coarse_config, logger, stats, levels_left - 1);
  return upsampleInto(source, coarse_filled, domain_mask, object_mask);
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

RasterGrid DEMEngine::createGrid(const Bounds& bounds, const DEMConfig& config, bool apply_edge_shrink) const {
  RasterGrid grid = createGrid(bounds, config.dem.cell_size, config.dem.nodata);
  if (apply_edge_shrink) {
    applyGlobalEdgeShrink(grid, config.dem.edge_shrink_cells);
  }
  return grid;
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

RasterGrid DEMEngine::buildDomainMask(const RasterGrid& accepted_support,
                                      const DEMConfig& config,
                                      Logger& logger,
                                      ProcessStats& stats,
                                      const RasterGrid* object_mask) const {
  ScopedTimer timer(stats, "dem_domain_mask_seconds");

  RasterGrid mask = buildSupportMask(accepted_support);
  if (object_mask != nullptr) {
    applyMaskExclusion(mask, object_mask);
  }

  const int close_radius = std::max(0, config.dem.fill_max_radius);
  if (close_radius > 0) {
    mask = erodeMask(dilateMask(mask, close_radius), close_radius, true);
    fillSmallClosedHoles(mask, static_cast<std::size_t>((2 * close_radius + 1) * (2 * close_radius + 1)));
  }
  clipMaskToSupportRadius(mask, accepted_support, std::max(config.dem.nearest_max_distance, config.dem.idw_radius));
  if (object_mask != nullptr) {
    applyMaskExclusion(mask, object_mask);
  }
  keepMajorComponents(mask, std::max<std::size_t>(9U, static_cast<std::size_t>((close_radius + 1) * (close_radius + 1))));

  logger.info("DEM domain mask generation complete.");
  return mask;
}

RasterGrid DEMEngine::interpolateNearest(const RasterGrid& support_grid,
                                         const RasterGrid& domain_mask,
                                         const DEMConfig& config,
                                         Logger& logger,
                                         ProcessStats& stats) const {
  ScopedTimer timer(stats, "dem_nearest_seconds");
  RasterGrid grid = support_grid;
  std::fill(grid.values.begin(), grid.values.end(), grid.nodata);
  std::fill(grid.edge_mask.begin(), grid.edge_mask.end(), 0U);

  for (std::size_t offset = 0; offset < grid.size(); ++offset) {
    if (isSupportCell(support_grid, offset)) {
      grid.values[offset] = support_grid.values[offset];
    }
  }

  const auto support_points = rasterPointsFromSupport(support_grid);
  if (support_points.empty()) {
    logger.warn("Support grid is empty; nearest DEM remains nodata.");
    return grid;
  }

  SpatialIndexManager index;
  std::vector<std::size_t> indices(support_points.size());
  std::iota(indices.begin(), indices.end(), 0U);
  index.build2D(support_points, indices);

  for (std::size_t row = 0; row < grid.rows; ++row) {
    for (std::size_t col = 0; col < grid.cols; ++col) {
      const auto offset = grid.offset(row, col);
      if (!isMaskActive(domain_mask, offset) || isSupportCell(support_grid, offset)) {
        continue;
      }

      const auto neighbors = index.knn2D(grid.cellCenterX(col), grid.cellCenterY(row), 1U);
      if (neighbors.indices.empty() || neighbors.distances.front() > config.dem.nearest_max_distance) {
        continue;
      }
      grid.values[offset] = support_points[neighbors.indices.front()].z;
    }
  }

  logger.info("Nearest-neighbor DEM interpolation complete.");
  return grid;
}

RasterGrid DEMEngine::interpolateIdw(const RasterGrid& support_grid,
                                     const RasterGrid& domain_mask,
                                     const DEMConfig& config,
                                     Logger& logger,
                                     ProcessStats& stats) const {
  ScopedTimer timer(stats, "dem_idw_seconds");
  RasterGrid grid = support_grid;
  std::fill(grid.values.begin(), grid.values.end(), grid.nodata);
  std::fill(grid.edge_mask.begin(), grid.edge_mask.end(), 0U);

  for (std::size_t offset = 0; offset < grid.size(); ++offset) {
    if (isSupportCell(support_grid, offset)) {
      grid.values[offset] = support_grid.values[offset];
    }
  }

  const auto support_points = rasterPointsFromSupport(support_grid);
  if (support_points.empty()) {
    logger.warn("Support grid is empty; IDW DEM remains nodata.");
    return grid;
  }

  SpatialIndexManager index;
  std::vector<std::size_t> indices(support_points.size());
  std::iota(indices.begin(), indices.end(), 0U);
  index.build2D(support_points, indices);

  for (std::size_t row = 0; row < grid.rows; ++row) {
    for (std::size_t col = 0; col < grid.cols; ++col) {
      const auto offset = grid.offset(row, col);
      if (!isMaskActive(domain_mask, offset) || isSupportCell(support_grid, offset)) {
        continue;
      }

      auto neighbors = index.radius2D(grid.cellCenterX(col),
                                      grid.cellCenterY(row),
                                      config.dem.idw_radius,
                                      std::max<std::size_t>(config.dem.idw_max_points, config.dem.idw_min_points));
      if (neighbors.indices.size() < config.dem.idw_min_points) {
        continue;
      }
      if (neighbors.indices.size() > config.dem.idw_max_points) {
        neighbors.indices.resize(config.dem.idw_max_points);
        neighbors.distances.resize(config.dem.idw_max_points);
      }

      double numerator = 0.0;
      double denominator = 0.0;
      bool exact_hit = false;
      for (std::size_t i = 0; i < neighbors.indices.size(); ++i) {
        const double distance = neighbors.distances[i];
        const double point_z = support_points[neighbors.indices[i]].z;
        if (distance < 1e-9) {
          grid.values[offset] = point_z;
          exact_hit = true;
          break;
        }
        const double weight = 1.0 / std::pow(distance, config.dem.idw_power);
        numerator += point_z * weight;
        denominator += weight;
      }
      if (!exact_hit && denominator > 0.0) {
        grid.values[offset] = numerator / denominator;
      }
    }
  }

  logger.info("IDW DEM interpolation complete.");
  return grid;
}

RasterGrid DEMEngine::fillHoles(const RasterGrid& source,
                                const RasterGrid& domain_mask,
                                const DEMConfig& config,
                                Logger& logger,
                                ProcessStats& stats,
                                const RasterGrid* object_mask) const {
  ScopedTimer timer(stats, "dem_fill_holes_seconds");
  if (!config.dem.fill_holes) {
    return source;
  }

  RasterGrid result = source;
  const int max_radius = std::max(0, config.dem.fill_max_radius);
  const std::size_t max_iterations = std::max<std::size_t>(1U, static_cast<std::size_t>(max_radius));
  std::size_t total_filled = 0U;

  for (std::size_t iteration = 0; iteration < max_iterations; ++iteration) {
    RasterGrid next = result;
    std::size_t changed = 0U;

    for (std::size_t row = 0; row < source.rows; ++row) {
      for (std::size_t col = 0; col < source.cols; ++col) {
        const auto offset = source.offset(row, col);
        if (!isMaskActive(domain_mask, offset) || isValidValue(result, offset) ||
            (object_mask != nullptr && object_mask->size() == source.size() && isMaskActive(*object_mask, offset))) {
          continue;
        }
        if (!hasOriginalSupportWithinRadius(source, object_mask, row, col, max_radius)) {
          continue;
        }

        std::vector<double> neighbors;
        neighbors.reserve(8);
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) {
              continue;
            }
            const int nr = static_cast<int>(row) + dr;
            const int nc = static_cast<int>(col) + dc;
            if (nr < 0 || nc < 0 || nr >= static_cast<int>(source.rows) || nc >= static_cast<int>(source.cols)) {
              continue;
            }
            const auto neighbor_offset = source.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc));
            if (!isMaskActive(domain_mask, neighbor_offset) ||
                (object_mask != nullptr && object_mask->size() == source.size() && isMaskActive(*object_mask, neighbor_offset)) ||
                !isValidValue(result, neighbor_offset)) {
              continue;
            }
            neighbors.push_back(result.values[neighbor_offset]);
          }
        }

        if (neighbors.size() < config.dem.fill_min_neighbors) {
          continue;
        }
        next.values[offset] = median(neighbors);
        ++changed;
      }
    }

    if (changed == 0U) {
      break;
    }
    total_filled += changed;
    result = std::move(next);
  }

  stats.setCount("dem_fill_changed_count", total_filled);
  logger.info("DEM hole filling complete.");
  return result;
}

RasterGrid DEMEngine::buildAnalysisDtm(const RasterGrid& support_grid,
                                       const RasterGrid& domain_mask,
                                       const RasterGrid& object_mask,
                                       const DEMConfig& config,
                                       Logger& logger,
                                       ProcessStats& stats) const {
  ScopedTimer timer(stats, "dtm_analysis_seconds");
  RasterGrid regularized_support = regularizeSupportSurface(support_grid, object_mask, config);
  RasterGrid analysis = regularized_support;
  std::fill(analysis.values.begin(), analysis.values.end(), analysis.nodata);
  std::fill(analysis.edge_mask.begin(), analysis.edge_mask.end(), 0U);

  for (std::size_t offset = 0; offset < analysis.size(); ++offset) {
    if (isSupportCell(regularized_support, offset) && !isMaskActive(object_mask, offset)) {
      analysis.values[offset] = regularized_support.values[offset];
    }
  }

  const auto support_points = rasterPointsFromSupport(regularized_support);
  if (support_points.empty()) {
    logger.warn("Support grid is empty; analysis DTM remains nodata.");
    return analysis;
  }

  SpatialIndexManager index;
  std::vector<std::size_t> indices(support_points.size());
  std::iota(indices.begin(), indices.end(), 0U);
  index.build2D(support_points, indices);

  for (std::size_t row = 0; row < analysis.rows; ++row) {
    for (std::size_t col = 0; col < analysis.cols; ++col) {
      const auto offset = analysis.offset(row, col);
      if (!isMaskActive(domain_mask, offset) || isMaskActive(object_mask, offset) ||
          isSupportCell(regularized_support, offset)) {
        continue;
      }

      auto neighbors = index.radius2D(analysis.cellCenterX(col),
                                      analysis.cellCenterY(row),
                                      config.dem.idw_radius,
                                      std::max<std::size_t>(config.dem.idw_max_points, config.dem.idw_min_points));
      if (neighbors.indices.size() >= config.dem.idw_min_points) {
        if (neighbors.indices.size() > config.dem.idw_max_points) {
          neighbors.indices.resize(config.dem.idw_max_points);
          neighbors.distances.resize(config.dem.idw_max_points);
        }
        double numerator = 0.0;
        double denominator = 0.0;
        for (std::size_t i = 0; i < neighbors.indices.size(); ++i) {
          const double distance = neighbors.distances[i];
          const double point_z = support_points[neighbors.indices[i]].z;
          if (distance < 1e-9) {
            numerator = point_z;
            denominator = 1.0;
            break;
          }
          const double weight = 1.0 / std::pow(distance, config.dem.idw_power);
          numerator += point_z * weight;
          denominator += weight;
        }
        if (denominator > 0.0) {
          analysis.values[offset] = numerator / denominator;
          continue;
        }
      }

      if (!neighbors.indices.empty()) {
        std::vector<double> neighbor_heights;
        neighbor_heights.reserve(neighbors.indices.size());
        for (const auto neighbor_index : neighbors.indices) {
          neighbor_heights.push_back(support_points[neighbor_index].z);
        }
        analysis.values[offset] = median(neighbor_heights);
      }
    }
  }

  analysis = fillHoles(analysis, domain_mask, config, logger, stats, &object_mask);
  analysis = coarseToFineFill(analysis, domain_mask, object_mask, config, logger, stats, 2);
  analysis = fillHoles(analysis, domain_mask, config, logger, stats, &object_mask);
  logger.info("Analysis DTM generation complete.");
  return analysis;
}

RasterGrid DEMEngine::buildDisplayDtm(const RasterGrid& analysis_dtm,
                                      const RasterGrid& domain_mask,
                                      const RasterGrid& object_mask,
                                      const DEMConfig& config,
                                      Logger& logger,
                                      ProcessStats& stats) const {
  ScopedTimer timer(stats, "dtm_display_seconds");
  const RasterGrid display_domain = buildDisplayDomainMask(domain_mask, config);
  RasterGrid display = analysis_dtm;
  std::vector<std::uint8_t> anchored(display.size(), 0U);
  for (std::size_t offset = 0; offset < display.size() && offset < display_domain.size(); ++offset) {
    if (isMaskActive(display_domain, offset) && isValidValue(analysis_dtm, offset)) {
      anchored[offset] = 1U;
      if (offset < display.valid_mask.size()) {
        display.valid_mask[offset] = 1U;
      }
    }
  }

  const auto known_points = rasterPointsFromValidValues(analysis_dtm, &display_domain);
  if (!known_points.empty()) {
    SpatialIndexManager index;
    std::vector<std::size_t> indices(known_points.size());
    std::iota(indices.begin(), indices.end(), 0U);
    index.build2D(known_points, indices);

    const std::size_t global_knn = std::max<std::size_t>(24U, config.dem.idw_max_points * 2U);
    std::size_t filled_count = 0U;
    for (std::size_t row = 0; row < display.rows; ++row) {
      for (std::size_t col = 0; col < display.cols; ++col) {
        const auto offset = display.offset(row, col);
        if (!isMaskActive(display_domain, offset) || isValidValue(display, offset)) {
          continue;
        }

        const auto neighbors = index.knn2D(display.cellCenterX(col), display.cellCenterY(row), global_knn);
        if (neighbors.indices.empty()) {
          continue;
        }

        double numerator = 0.0;
        double denominator = 0.0;
        bool exact_hit = false;
        for (std::size_t i = 0; i < neighbors.indices.size(); ++i) {
          const double distance = neighbors.distances[i];
          const double point_z = known_points[neighbors.indices[i]].z;
          if (distance < 1e-9) {
            display.values[offset] = point_z;
            exact_hit = true;
            break;
          }
          const double weight = 1.0 / std::pow(std::max(distance, 1e-6), config.dem.idw_power);
          numerator += point_z * weight;
          denominator += weight;
        }
        if (!exact_hit && denominator > 0.0) {
          display.values[offset] = numerator / denominator;
        }
        if (isValidValue(display, offset)) {
          if (offset < display.valid_mask.size()) {
            display.valid_mask[offset] = 1U;
          }
          ++filled_count;
        }
      }
    }
    stats.setCount("dtm_display_fill_changed_count", filled_count);
  } else {
    stats.setCount("dtm_display_fill_changed_count", 0U);
  }

  for (int iteration = 0; iteration < 3; ++iteration) {
    RasterGrid next = display;
    for (std::size_t row = 0; row < display.rows; ++row) {
      for (std::size_t col = 0; col < display.cols; ++col) {
        const auto offset = display.offset(row, col);
        if (!isMaskActive(display_domain, offset) || !isValidValue(display, offset)) {
          continue;
        }

        std::vector<double> neighbors;
        neighbors.reserve(9);
        double min_value = std::numeric_limits<double>::infinity();
        double max_value = -std::numeric_limits<double>::infinity();
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            const int nr = static_cast<int>(row) + dr;
            const int nc = static_cast<int>(col) + dc;
            if (nr < 0 || nc < 0 || nr >= static_cast<int>(display.rows) || nc >= static_cast<int>(display.cols)) {
              continue;
            }
            const auto neighbor_offset = display.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc));
            if (!isMaskActive(display_domain, neighbor_offset) || !isValidValue(display, neighbor_offset)) {
              continue;
            }
            neighbors.push_back(display.values[neighbor_offset]);
            min_value = std::min(min_value, display.values[neighbor_offset]);
            max_value = std::max(max_value, display.values[neighbor_offset]);
          }
        }
        if (neighbors.size() < 4U) {
          continue;
        }

        const double local_median = median(neighbors);
        const double local_range = max_value - min_value;
        double blend = anchored[offset] != 0U ? 0.18 : 0.55;
        if (local_range > config.ground.max_height_diff * 4.0) {
          blend *= 0.45;
        }
        if (isMaskActive(object_mask, offset)) {
          blend = std::max(blend, 0.45);
        }
        next.values[offset] = display.values[offset] * (1.0 - blend) + local_median * blend;
      }
    }
    display = std::move(next);
  }

  const auto remaining_nodata = countRemainingNodataInMask(display, display_domain);
  const auto active_cells = std::max<std::size_t>(
      1U, std::count_if(display_domain.values.begin(), display_domain.values.end(), [](double value) { return std::isfinite(value) && value > 0.5; }));
  stats.setCount("dtm_display_remaining_nodata_count", remaining_nodata);
  stats.setValue("dtm_display_remaining_nodata_ratio",
                 static_cast<double>(remaining_nodata) / static_cast<double>(active_cells));
  stats.setValue("dtm_display_domain_active_ratio",
                 static_cast<double>(active_cells) / static_cast<double>(std::max<std::size_t>(1U, display_domain.size())));
  logger.info("Display DTM generation complete.");
  return display;
}

RasterGrid DEMEngine::buildEdgeMask(const RasterGrid& domain_mask,
                                    const RasterGrid& dem_nearest,
                                    const RasterGrid& dem_nearest_filled,
                                    const RasterGrid& dem_idw,
                                    const RasterGrid& dem_idw_filled,
                                    const DEMConfig& config,
                                    Logger& logger,
                                    ProcessStats& stats) const {
  RasterGrid empty = makeBinaryMaskLike(domain_mask);
  RasterGrid before_fill = dem_idw.empty() ? dem_nearest : dem_idw;
  RasterGrid after_fill = dem_idw_filled.empty() ? dem_nearest_filled : dem_idw_filled;
  return buildStructuralEdgeMask(domain_mask, empty, before_fill, after_fill, empty, config, logger, stats);
}

RasterGrid DEMEngine::buildStructuralEdgeMask(const RasterGrid& domain_mask,
                                              const RasterGrid& object_mask,
                                              const RasterGrid& before_fill,
                                              const RasterGrid& after_fill,
                                              const RasterGrid& seam_mask,
                                              const DEMConfig& config,
                                              Logger& logger,
                                              ProcessStats& stats) const {
  ScopedTimer timer(stats, "dem_edge_mask_seconds");
  RasterGrid edge = makeBinaryMaskLike(domain_mask);

  const auto shrink_cells = roundedShrinkCells(config.dem.edge_shrink_cells);
  if (shrink_cells > 0U) {
    const auto eroded = erodeMask(domain_mask, static_cast<int>(shrink_cells), false);
    for (std::size_t offset = 0; offset < edge.size(); ++offset) {
      if (isMaskActive(domain_mask, offset) && !isMaskActive(eroded, offset)) {
        edge.values[offset] = 1.0;
      }
    }
  }

  const std::size_t large_hole_threshold =
      static_cast<std::size_t>((2 * std::max(0, config.dem.fill_max_radius) + 1) *
                               (2 * std::max(0, config.dem.fill_max_radius) + 1));
  std::vector<std::uint8_t> visited(domain_mask.size(), 0U);
  const std::array<int, 5> steps{-1, 0, 1, 0, -1};
  for (std::size_t row = 0; row < domain_mask.rows; ++row) {
    for (std::size_t col = 0; col < domain_mask.cols; ++col) {
      const auto start = domain_mask.offset(row, col);
      if (visited[start] != 0U || isMaskActive(domain_mask, start)) {
        continue;
      }

      std::queue<std::pair<std::size_t, std::size_t>> queue;
      std::vector<std::pair<std::size_t, std::size_t>> component;
      bool touches_border = false;
      visited[start] = 1U;
      queue.push({row, col});
      while (!queue.empty()) {
        const auto [current_row, current_col] = queue.front();
        queue.pop();
        component.push_back({current_row, current_col});
        touches_border = touches_border || current_row == 0 || current_col == 0 ||
                         current_row + 1 == domain_mask.rows || current_col + 1 == domain_mask.cols;
        for (std::size_t step = 0; step < 4; ++step) {
          const int nr = static_cast<int>(current_row) + steps[step];
          const int nc = static_cast<int>(current_col) + steps[step + 1];
          if (nr < 0 || nc < 0 || nr >= static_cast<int>(domain_mask.rows) || nc >= static_cast<int>(domain_mask.cols)) {
            continue;
          }
          const auto next = domain_mask.offset(static_cast<std::size_t>(nr), static_cast<std::size_t>(nc));
          if (visited[next] != 0U || isMaskActive(domain_mask, next)) {
            continue;
          }
          visited[next] = 1U;
          queue.push({static_cast<std::size_t>(nr), static_cast<std::size_t>(nc)});
        }
      }

      if (!touches_border && component.size() > large_hole_threshold) {
        for (const auto& [hole_row, hole_col] : component) {
          markNeighbors(hole_row, hole_col, domain_mask, edge);
        }
      }
    }
  }

  for (std::size_t row = 0; row < object_mask.rows; ++row) {
    for (std::size_t col = 0; col < object_mask.cols; ++col) {
      const auto offset = object_mask.offset(row, col);
      if (!isMaskActive(object_mask, offset)) {
        continue;
      }
      markNeighbors(row, col, domain_mask, edge);
    }
  }

  if (seam_mask.size() == edge.size()) {
    for (std::size_t offset = 0; offset < edge.size(); ++offset) {
      if (isMaskActive(seam_mask, offset)) {
        edge.values[offset] = 1.0;
      }
    }
  }

  std::size_t fill_frontier_count = 0U;
  for (std::size_t row = 0; row < domain_mask.rows; ++row) {
    for (std::size_t col = 0; col < domain_mask.cols; ++col) {
      const auto offset = domain_mask.offset(row, col);
      const bool filled = !isValidValue(before_fill, offset) && isValidValue(after_fill, offset);
      if (!filled) {
        continue;
      }
      ++fill_frontier_count;
      markNeighbors(row, col, domain_mask, edge);
    }
  }

  stats.setCount("fill_frontier_count", fill_frontier_count);
  stats.setValue("fill_frontier_ratio",
                 domain_mask.size() == 0U ? 0.0
                                          : static_cast<double>(fill_frontier_count) /
                                                static_cast<double>(std::max<std::size_t>(1U, domain_mask.size())));
  logger.info("DEM edge mask generation complete.");
  return edge;
}

void DEMEngine::applyGlobalEdgeShrink(RasterGrid& grid, double edge_shrink_cells) const {
  const auto shrink_cells = roundedShrinkCells(edge_shrink_cells);
  if (shrink_cells == 0U || grid.empty()) {
    return;
  }

  for (std::size_t row = 0; row < grid.rows; ++row) {
    for (std::size_t col = 0; col < grid.cols; ++col) {
      if (row < shrink_cells || col < shrink_cells ||
          row + shrink_cells >= grid.rows || col + shrink_cells >= grid.cols) {
        grid.edge_mask[grid.offset(row, col)] = 1U;
      }
    }
  }
}

}  // namespace dem
