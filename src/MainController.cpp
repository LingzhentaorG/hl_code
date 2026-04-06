/**
 * @file MainController.cpp
 * @brief 主控制器实现：协调 DEM 处理流水线
 */

#include "dem/MainController.hpp"

#include "dem/Utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>

namespace dem {
namespace {

struct ContinuousRasterSummary {
  std::size_t valid_count = 0U;
  std::size_t nodata_count = 0U;
  double valid_ratio = 0.0;
  double nodata_ratio = 0.0;
};

struct MaskRasterSummary {
  std::size_t active_count = 0U;
  std::size_t inactive_count = 0U;
  double active_ratio = 0.0;
  double inactive_ratio = 0.0;
};

RasterGrid makeBinaryMaskFromTemplate(const RasterGrid& grid) {
  RasterGrid mask;
  mask.rows = grid.rows;
  mask.cols = grid.cols;
  mask.cell_size = grid.cell_size;
  mask.origin_x = grid.origin_x;
  mask.origin_y = grid.origin_y;
  mask.nodata = -1.0;
  mask.values.assign(grid.size(), 0.0);
  mask.valid_mask.assign(grid.size(), 1U);
  mask.edge_mask.assign(grid.size(), 0U);
  return mask;
}

Bounds rasterBounds(const RasterGrid& grid) {
  Bounds bounds;
  if (grid.empty()) {
    return bounds;
  }
  bounds.xmin = grid.origin_x;
  bounds.xmax = grid.origin_x + static_cast<double>(grid.cols) * grid.cell_size;
  bounds.ymax = grid.origin_y;
  bounds.ymin = grid.origin_y - static_cast<double>(grid.rows) * grid.cell_size;
  bounds.zmin = 0.0;
  bounds.zmax = 1.0;
  return bounds;
}

ContinuousRasterSummary summarizeContinuousRaster(const RasterGrid& grid) {
  ContinuousRasterSummary summary;
  const auto total = std::max<std::size_t>(1U, grid.size());
  for (const double value : grid.values) {
    if (!std::isfinite(value) || value == grid.nodata) {
      ++summary.nodata_count;
      continue;
    }
    ++summary.valid_count;
  }
  summary.valid_ratio = static_cast<double>(summary.valid_count) / static_cast<double>(total);
  summary.nodata_ratio = static_cast<double>(summary.nodata_count) / static_cast<double>(total);
  return summary;
}

MaskRasterSummary summarizeMaskRaster(const RasterGrid& grid) {
  MaskRasterSummary summary;
  const auto total = std::max<std::size_t>(1U, grid.size());
  for (const double value : grid.values) {
    if (std::isfinite(value) && value > 0.5) {
      ++summary.active_count;
    } else {
      ++summary.inactive_count;
    }
  }
  summary.active_ratio = static_cast<double>(summary.active_count) / static_cast<double>(total);
  summary.inactive_ratio = static_cast<double>(summary.inactive_count) / static_cast<double>(total);
  return summary;
}

void recordContinuousRasterStats(const std::string& name, const RasterGrid& grid, ProcessStats& stats) {
  if (grid.empty()) {
    return;
  }
  const auto summary = summarizeContinuousRaster(grid);
  stats.setCount(name + "_valid_count", summary.valid_count);
  stats.setCount(name + "_nodata_count", summary.nodata_count);
  stats.setValue(name + "_valid_ratio", summary.valid_ratio);
  stats.setValue(name + "_nodata_ratio", summary.nodata_ratio);
}

void recordMaskRasterStats(const std::string& name, const RasterGrid& grid, ProcessStats& stats) {
  if (grid.empty()) {
    return;
  }
  const auto summary = summarizeMaskRaster(grid);
  stats.setCount(name + "_active_count", summary.active_count);
  stats.setCount(name + "_inactive_count", summary.inactive_count);
  stats.setValue(name + "_active_ratio", summary.active_ratio);
  stats.setValue(name + "_inactive_ratio", summary.inactive_ratio);
}

std::size_t countChangedPixels(const RasterGrid& before, const RasterGrid& after) {
  std::size_t changed = 0U;
  for (std::size_t i = 0; i < before.values.size() && i < after.values.size(); ++i) {
    if (before.values[i] != after.values[i]) {
      ++changed;
    }
  }
  return changed;
}

PointCloud loadTilePointCloud(const std::filesystem::path& xyz_path) {
  PointCloud cloud;
  std::ifstream stream(xyz_path);
  if (!stream) {
    throw std::runtime_error("Unable to open tile file: " + xyz_path.string());
  }
  Point3D point;
  while (stream >> point.x >> point.y >> point.z) {
    point.index = static_cast<std::uint32_t>(cloud.points.size());
    cloud.points.push_back(point);
    cloud.bounds.expand(point.x, point.y, point.z);
  }
  cloud.raw_point_count = cloud.points.size();
  cloud.stored_point_count = cloud.points.size();
  return cloud;
}

void appendPointsInBounds(const PointCloud& source, PointCloud& destination, const Bounds& bounds) {
  for (const auto& point : source.points) {
    if (!bounds.containsXY(point.x, point.y)) {
      continue;
    }
    destination.points.push_back(point);
    destination.bounds.expand(point.x, point.y, point.z);
  }
  destination.raw_point_count = destination.points.size();
  destination.stored_point_count = destination.points.size();
}

void mergeMinimumRasterInto(const RasterGrid& tile, RasterGrid& merged) {
  if (tile.empty() || merged.empty()) {
    return;
  }
  const auto row_offset = static_cast<std::size_t>(std::llround((merged.origin_y - tile.origin_y) / merged.cell_size));
  const auto col_offset = static_cast<std::size_t>(std::llround((tile.origin_x - merged.origin_x) / merged.cell_size));

  for (std::size_t row = 0; row < tile.rows; ++row) {
    for (std::size_t col = 0; col < tile.cols; ++col) {
      const auto src_offset = tile.offset(row, col);
      if (src_offset >= tile.valid_mask.size() || tile.valid_mask[src_offset] == 0U ||
          !std::isfinite(tile.values[src_offset]) || tile.values[src_offset] == tile.nodata) {
        continue;
      }

      const auto dst_row = row_offset + row;
      const auto dst_col = col_offset + col;
      if (dst_row >= merged.rows || dst_col >= merged.cols) {
        continue;
      }

      const auto dst_offset = merged.offset(dst_row, dst_col);
      if (dst_offset >= merged.valid_mask.size() || merged.valid_mask[dst_offset] == 0U ||
          !std::isfinite(merged.values[dst_offset]) || merged.values[dst_offset] == merged.nodata) {
        merged.values[dst_offset] = tile.values[src_offset];
      } else {
        merged.values[dst_offset] = std::min(merged.values[dst_offset], tile.values[src_offset]);
      }
      merged.valid_mask[dst_offset] = 1U;
    }
  }
}

RasterGrid aggregateMinimumRaster(const RasterGrid& source, const RasterGrid& target) {
  RasterGrid aggregated = target;
  std::fill(aggregated.values.begin(), aggregated.values.end(), aggregated.nodata);
  std::fill(aggregated.valid_mask.begin(), aggregated.valid_mask.end(), 0U);
  std::fill(aggregated.edge_mask.begin(), aggregated.edge_mask.end(), 0U);

  for (std::size_t row = 0; row < source.rows; ++row) {
    for (std::size_t col = 0; col < source.cols; ++col) {
      const auto source_offset = source.offset(row, col);
      if (source_offset >= source.valid_mask.size() || source.valid_mask[source_offset] == 0U ||
          !std::isfinite(source.values[source_offset]) || source.values[source_offset] == source.nodata) {
        continue;
      }

      const double x = source.cellCenterX(col);
      const double y = source.cellCenterY(row);
      const double target_col_f = std::floor((x - target.origin_x) / target.cell_size);
      const double target_row_f = std::floor((target.origin_y - y) / target.cell_size);
      if (target_col_f < 0.0 || target_row_f < 0.0) {
        continue;
      }
      const auto target_col = static_cast<std::size_t>(target_col_f);
      const auto target_row = static_cast<std::size_t>(target_row_f);
      if (target_row >= target.rows || target_col >= target.cols) {
        continue;
      }

      const auto target_offset = aggregated.offset(target_row, target_col);
      if (aggregated.valid_mask[target_offset] == 0U || source.values[source_offset] < aggregated.values[target_offset]) {
        aggregated.values[target_offset] = source.values[source_offset];
      }
      aggregated.valid_mask[target_offset] = 1U;
    }
  }
  return aggregated;
}

DEMConfig makeResolutionConfig(const DEMConfig& base_config, double cell_size) {
  DEMConfig config = base_config;
  config.dem.cell_size = cell_size;
  config.ground.seed_grid_size = 3.0 * cell_size;
  config.ground.search_radius = std::max(6.0 * cell_size, 2.0 * config.ground.seed_grid_size);
  config.dem.nearest_max_distance = std::max(6.0 * cell_size, config.ground.search_radius);
  config.dem.idw_radius = std::max(8.0 * cell_size, config.ground.search_radius + 2.0 * cell_size);
  config.tile.tile_buffer =
      std::max({config.ground.search_radius, config.dem.nearest_max_distance, config.dem.idw_radius}) +
      static_cast<double>(std::max(0, config.dem.fill_max_radius)) * cell_size;
  return config;
}

std::string cellSizeKey(double cell_size) {
  std::ostringstream stream;
  if (std::abs(cell_size - std::round(cell_size)) < 1e-6) {
    stream << static_cast<long long>(std::llround(cell_size));
  } else {
    stream << std::fixed << std::setprecision(1) << cell_size;
  }
  auto key = stream.str();
  std::replace(key.begin(), key.end(), '.', '_');
  return key;
}

double computeRmse(const RasterGrid& lhs, const RasterGrid& rhs) {
  double squared_error = 0.0;
  std::size_t count = 0U;
  for (std::size_t i = 0; i < lhs.values.size() && i < rhs.values.size(); ++i) {
    if (!std::isfinite(lhs.values[i]) || lhs.values[i] == lhs.nodata || !std::isfinite(rhs.values[i]) ||
        rhs.values[i] == rhs.nodata) {
      continue;
    }
    const double delta = lhs.values[i] - rhs.values[i];
    squared_error += delta * delta;
    ++count;
  }
  return count == 0U ? 0.0 : std::sqrt(squared_error / static_cast<double>(count));
}

RasterGrid buildTileSeamMask(const RasterGrid& analysis,
                             const RasterGrid& domain_mask,
                             const std::vector<TileDefinition>* tiles,
                             double& seam_score) {
  RasterGrid seam_mask = makeBinaryMaskFromTemplate(domain_mask);
  seam_score = 0.0;
  if (tiles == nullptr || tiles->empty() || analysis.empty()) {
    return seam_mask;
  }

  double global_diff_sum = 0.0;
  std::size_t global_diff_count = 0U;
  for (std::size_t row = 0; row < analysis.rows; ++row) {
    for (std::size_t col = 0; col + 1 < analysis.cols; ++col) {
      const auto left = analysis.offset(row, col);
      const auto right = analysis.offset(row, col + 1);
      if (!std::isfinite(analysis.values[left]) || analysis.values[left] == analysis.nodata ||
          !std::isfinite(analysis.values[right]) || analysis.values[right] == analysis.nodata) {
        continue;
      }
      global_diff_sum += std::abs(analysis.values[left] - analysis.values[right]);
      ++global_diff_count;
    }
  }
  for (std::size_t row = 0; row + 1 < analysis.rows; ++row) {
    for (std::size_t col = 0; col < analysis.cols; ++col) {
      const auto top = analysis.offset(row, col);
      const auto bottom = analysis.offset(row + 1, col);
      if (!std::isfinite(analysis.values[top]) || analysis.values[top] == analysis.nodata ||
          !std::isfinite(analysis.values[bottom]) || analysis.values[bottom] == analysis.nodata) {
        continue;
      }
      global_diff_sum += std::abs(analysis.values[top] - analysis.values[bottom]);
      ++global_diff_count;
    }
  }

  const double global_mean_diff =
      global_diff_count == 0U ? 0.0 : global_diff_sum / static_cast<double>(global_diff_count);
  const double seam_threshold = global_mean_diff > 0.0 ? global_mean_diff * 1.5 : std::numeric_limits<double>::infinity();

  std::set<std::size_t> vertical_boundaries;
  std::set<std::size_t> horizontal_boundaries;
  const Bounds extent = rasterBounds(analysis);
  for (const auto& tile : *tiles) {
    if (tile.main_bounds.xmax + 1e-6 < extent.xmax) {
      const double boundary_col_f = (tile.main_bounds.xmax - analysis.origin_x) / analysis.cell_size;
      if (boundary_col_f > 0.5) {
        vertical_boundaries.insert(static_cast<std::size_t>(std::llround(boundary_col_f)));
      }
    }
    if (tile.main_bounds.ymin - 1e-6 > extent.ymin) {
      const double boundary_row_f = (analysis.origin_y - tile.main_bounds.ymin) / analysis.cell_size;
      if (boundary_row_f > 0.5) {
        horizontal_boundaries.insert(static_cast<std::size_t>(std::llround(boundary_row_f)));
      }
    }
  }

  double seam_diff_sum = 0.0;
  std::size_t seam_diff_count = 0U;
  for (const auto boundary : vertical_boundaries) {
    if (boundary == 0U || boundary >= analysis.cols) {
      continue;
    }
    for (std::size_t row = 0; row < analysis.rows; ++row) {
      const auto left = analysis.offset(row, boundary - 1U);
      const auto right = analysis.offset(row, boundary);
      if (!std::isfinite(analysis.values[left]) || analysis.values[left] == analysis.nodata ||
          !std::isfinite(analysis.values[right]) || analysis.values[right] == analysis.nodata) {
        continue;
      }
      const double diff = std::abs(analysis.values[left] - analysis.values[right]);
      seam_diff_sum += diff;
      ++seam_diff_count;
      if (diff > seam_threshold) {
        if (domain_mask.values[left] > 0.5) {
          seam_mask.values[left] = 1.0;
        }
        if (domain_mask.values[right] > 0.5) {
          seam_mask.values[right] = 1.0;
        }
      }
    }
  }

  for (const auto boundary : horizontal_boundaries) {
    if (boundary == 0U || boundary >= analysis.rows) {
      continue;
    }
    for (std::size_t col = 0; col < analysis.cols; ++col) {
      const auto top = analysis.offset(boundary - 1U, col);
      const auto bottom = analysis.offset(boundary, col);
      if (!std::isfinite(analysis.values[top]) || analysis.values[top] == analysis.nodata ||
          !std::isfinite(analysis.values[bottom]) || analysis.values[bottom] == analysis.nodata) {
        continue;
      }
      const double diff = std::abs(analysis.values[top] - analysis.values[bottom]);
      seam_diff_sum += diff;
      ++seam_diff_count;
      if (diff > seam_threshold) {
        if (domain_mask.values[top] > 0.5) {
          seam_mask.values[top] = 1.0;
        }
        if (domain_mask.values[bottom] > 0.5) {
          seam_mask.values[bottom] = 1.0;
        }
      }
    }
  }

  const double seam_mean_diff = seam_diff_count == 0U ? 0.0 : seam_diff_sum / static_cast<double>(seam_diff_count);
  seam_score = global_mean_diff > 0.0 ? seam_mean_diff / global_mean_diff : 0.0;
  return seam_mask;
}

void recordAllRasterStats(const ProcessingArtifacts& artifacts, ProcessStats& stats) {
  const auto raw_summary = summarizeContinuousRaster(artifacts.dem_raw_direct);
  const auto ground_summary = summarizeContinuousRaster(artifacts.dem_ground_direct);
  const auto support_summary = summarizeMaskRaster(artifacts.dem_support_mask);
  const auto domain_summary = summarizeMaskRaster(artifacts.dem_mask);

  recordContinuousRasterStats("dem_raw_direct", artifacts.dem_raw_direct, stats);
  recordContinuousRasterStats("dem_ground_direct", artifacts.dem_ground_direct, stats);
  recordContinuousRasterStats("dem_nearest", artifacts.dem_nearest, stats);
  recordContinuousRasterStats("dem_idw", artifacts.dem_idw, stats);
  recordContinuousRasterStats("dem_nearest_filled", artifacts.dem_nearest_filled, stats);
  recordContinuousRasterStats("dem_idw_filled", artifacts.dem_idw_filled, stats);
  recordContinuousRasterStats("dtm_analysis", artifacts.dtm_analysis, stats);
  recordContinuousRasterStats("dtm_display", artifacts.dtm_display, stats);
  recordMaskRasterStats("dem_support_mask", artifacts.dem_support_mask, stats);
  recordMaskRasterStats("dem_mask", artifacts.dem_mask, stats);
  recordMaskRasterStats("dem_edge_mask", artifacts.dem_edge_mask, stats);
  recordMaskRasterStats("dtm_object_mask", artifacts.dtm_object_mask, stats);

  stats.setValue("raw_direct_active_ratio", raw_summary.valid_ratio);
  stats.setValue("ground_direct_active_ratio", ground_summary.valid_ratio);
  stats.setValue("support_active_ratio", support_summary.active_ratio);
  stats.setValue("dem_mask_active_ratio", domain_summary.active_ratio);
}

}  // namespace

int MainController::run(const std::filesystem::path& config_path, const std::vector<std::string>& overrides) {
  auto config = config_manager_.load(config_path, overrides);
  if (config.input.file_path.is_relative()) {
    config.input.file_path = std::filesystem::weakly_canonical(config_path.parent_path() / config.input.file_path);
  }
  if (config.output.directory.is_relative()) {
    config.output.directory = std::filesystem::weakly_canonical(config_path.parent_path() / config.output.directory);
  }
  if (config.output.timestamp_subdir) {
    config.output.directory = makeTimestampedOutputDir(config.output.directory);
  }

  ensureDirectory(config.output.directory);
  ensureDirectory(config.output.directory / "log");

  Logger logger;
  logger.open(config.output.directory / "log" / "run_log.txt");

  ProcessStats stats;
  const auto pipeline_start = std::chrono::steady_clock::now();

  logger.info("Starting DEM generation.");
  config_manager_.writeResolvedConfig(config, config.output.directory / "log" / "config_used.txt");

  auto resolved_crs = crs_manager_.resolve(config.crs, logger, stats);
  auto validation_report =
      input_manager_.validateInput(config.input.file_path, config, config.output.directory, logger, stats);

  stats.tile_mode_used = tile_manager_.shouldUseTiles(validation_report.summary, config);
  stats.setCount("tile_mode_used", stats.tile_mode_used ? 1U : 0U);
  logger.info(std::string("Tile mode: ") + (stats.tile_mode_used ? "enabled" : "disabled"));

  ProcessingArtifacts artifacts = stats.tile_mode_used
                                      ? processTiles(config, logger, stats, validation_report.summary)
                                      : processSingleCloud(input_manager_.readPointCloud(config.input.file_path, logger, stats),
                                                           config,
                                                           logger,
                                                           stats,
                                                           nullptr);

  output_manager_.writeArtifacts(artifacts, config, resolved_crs, logger, stats);

  stats.addDuration("total_seconds",
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - pipeline_start).count());
  logger.info("Finished DEM generation in " + formatSeconds(stats.durations_seconds["total_seconds"]));
  return 0;
}

ProcessingArtifacts MainController::processSupportStage(PointCloud cloud,
                                                        const DEMConfig& config,
                                                        Logger& logger,
                                                        ProcessStats& stats,
                                                        const Bounds* forced_grid_bounds) const {
  ProcessingArtifacts artifacts;

  PointCloud preprocessed = preprocess_engine_.run(std::move(cloud), config, logger, stats);
  stats.setValue("cell_size_vs_spacing_ratio",
                 preprocessed.estimated_avg_spacing > 0.0 ? config.dem.cell_size / preprocessed.estimated_avg_spacing
                                                          : 0.0);
  if (preprocessed.estimated_avg_spacing > 0.0 && config.dem.cell_size < preprocessed.estimated_avg_spacing * 0.75) {
    logger.warn("DEM cell size is finer than estimated point spacing; expect sparse support and holes.");
  }

  const Bounds grid_bounds = forced_grid_bounds != nullptr ? *forced_grid_bounds : preprocessed.bounds;
  if (!grid_bounds.valid()) {
    throw std::runtime_error("Unable to derive valid DEM bounds.");
  }

  const RasterGrid grid_template = dem_engine_.createGrid(grid_bounds, config, false);
  artifacts.dem_raw_direct = dem_engine_.rasterizeMinimum(preprocessed, grid_template);

  artifacts.filtered_points = ground_filter_engine_.removeOutliers(std::move(preprocessed), config, logger, stats);
  ground_filter_engine_.extractGround(
      artifacts.filtered_points, artifacts.seed_points, artifacts.ground_points, artifacts.nonground_points, config, logger, stats);

  artifacts.dem_ground_direct = dem_engine_.rasterizeMinimum(artifacts.ground_points, grid_template);
  artifacts.dem_support_mask = dem_engine_.buildSupportMask(artifacts.dem_ground_direct);
  return artifacts;
}

void MainController::finalizeArtifacts(ProcessingArtifacts& artifacts,
                                       const DEMConfig& config,
                                       Logger& logger,
                                       ProcessStats& stats,
                                       const std::vector<TileDefinition>* tiles) const {
  const auto raw_summary = summarizeContinuousRaster(artifacts.dem_raw_direct);
  const auto ground_summary = summarizeContinuousRaster(artifacts.dem_ground_direct);
  if (raw_summary.valid_ratio > 0.0 && ground_summary.valid_ratio < raw_summary.valid_ratio * 0.25) {
    logger.warn("Ground coverage is significantly lower than raw direct coverage; ground filter may be too conservative.");
  }

  artifacts.dem_support_mask = dem_engine_.buildSupportMask(artifacts.dem_ground_direct);
  auto qc_object_mask = dem_engine_.buildObjectMask(artifacts.dem_raw_direct, artifacts.dem_ground_direct, config, logger, stats);
  auto qc_support =
      dem_engine_.refineAcceptedSupport(artifacts.dem_raw_direct, artifacts.dem_ground_direct, config, logger, stats, &qc_object_mask);
  auto qc_domain = dem_engine_.buildDomainMask(qc_support, config, logger, stats, &qc_object_mask);

  artifacts.dem_nearest = dem_engine_.interpolateNearest(qc_support, qc_domain, config, logger, stats);
  artifacts.dem_idw = dem_engine_.interpolateIdw(qc_support, qc_domain, config, logger, stats);
  artifacts.dem_nearest_filled = dem_engine_.fillHoles(artifacts.dem_nearest, qc_domain, config, logger, stats, &qc_object_mask);
  artifacts.dem_idw_filled = dem_engine_.fillHoles(artifacts.dem_idw, qc_domain, config, logger, stats, &qc_object_mask);

  std::vector<double> candidate_sizes{config.dem.cell_size, config.dem.cell_size * 1.5, config.dem.cell_size * 2.0};
  std::sort(candidate_sizes.begin(), candidate_sizes.end());
  candidate_sizes.erase(std::unique(candidate_sizes.begin(), candidate_sizes.end(),
                                    [](double lhs, double rhs) { return std::abs(lhs - rhs) < 1e-6; }),
                        candidate_sizes.end());

  const Bounds target_bounds = rasterBounds(artifacts.dem_raw_direct);
  DEMConfig selected_config = config;
  RasterGrid selected_raw = artifacts.dem_raw_direct;
  RasterGrid selected_ground = artifacts.dem_ground_direct;
  RasterGrid selected_support_mask = artifacts.dem_support_mask;
  RasterGrid selected_object = qc_object_mask;
  RasterGrid selected_accepted = qc_support;
  RasterGrid selected_domain = qc_domain;

  bool selected = false;
  for (std::size_t i = 0; i < candidate_sizes.size(); ++i) {
    const double candidate_size = candidate_sizes[i];
    DEMConfig candidate_config = makeResolutionConfig(config, candidate_size);
    const RasterGrid candidate_template = dem_engine_.createGrid(target_bounds, candidate_config, false);
    RasterGrid candidate_raw = std::abs(candidate_size - config.dem.cell_size) < 1e-6
                                   ? artifacts.dem_raw_direct
                                   : aggregateMinimumRaster(artifacts.dem_raw_direct, candidate_template);
    RasterGrid candidate_ground = std::abs(candidate_size - config.dem.cell_size) < 1e-6
                                      ? artifacts.dem_ground_direct
                                      : dem_engine_.rasterizeMinimum(artifacts.ground_points, candidate_template);
    RasterGrid candidate_support_mask = dem_engine_.buildSupportMask(candidate_ground);
    RasterGrid candidate_object =
        dem_engine_.buildObjectMask(candidate_raw, candidate_ground, candidate_config, logger, stats);
    RasterGrid candidate_accepted =
        dem_engine_.refineAcceptedSupport(candidate_raw, candidate_ground, candidate_config, logger, stats, &candidate_object);
    RasterGrid candidate_domain =
        dem_engine_.buildDomainMask(candidate_accepted, candidate_config, logger, stats, &candidate_object);

    const auto support_summary = summarizeMaskRaster(candidate_support_mask);
    const auto domain_summary = summarizeMaskRaster(candidate_domain);
    stats.setValue("support_active_ratio_" + cellSizeKey(candidate_size), support_summary.active_ratio);
    stats.setValue("domain_active_ratio_" + cellSizeKey(candidate_size), domain_summary.active_ratio);

    selected_config = candidate_config;
    selected_raw = std::move(candidate_raw);
    selected_ground = std::move(candidate_ground);
    selected_support_mask = std::move(candidate_support_mask);
    selected_object = std::move(candidate_object);
    selected_accepted = std::move(candidate_accepted);
    selected_domain = std::move(candidate_domain);

    if (!selected && support_summary.active_ratio >= 0.55 && domain_summary.active_ratio >= 0.75) {
      selected = true;
      break;
    }
  }

  artifacts.dem_mask = selected_domain;
  artifacts.dtm_object_mask = selected_object;
  artifacts.dtm_analysis = dem_engine_.buildAnalysisDtm(
      selected_accepted, artifacts.dem_mask, artifacts.dtm_object_mask, selected_config, logger, stats);
  artifacts.dtm_display = dem_engine_.buildDisplayDtm(
      artifacts.dtm_analysis, artifacts.dem_mask, artifacts.dtm_object_mask, selected_config, logger, stats);

  RasterGrid seam_mask = makeBinaryMaskFromTemplate(artifacts.dem_mask);
  double seam_score = 0.0;
  if (stats.tile_mode_used) {
    seam_mask = buildTileSeamMask(artifacts.dtm_analysis, artifacts.dem_mask, tiles, seam_score);
  }
  stats.setValue("tile_seam_score", seam_score);

  RasterGrid analysis_prefill =
      dem_engine_.interpolateIdw(selected_accepted, artifacts.dem_mask, selected_config, logger, stats);
  artifacts.dem_edge_mask = dem_engine_.buildStructuralEdgeMask(artifacts.dem_mask,
                                                                artifacts.dtm_object_mask,
                                                                analysis_prefill,
                                                                artifacts.dtm_analysis,
                                                                seam_mask,
                                                                selected_config,
                                                                logger,
                                                                stats);

  stats.setCount("dem_nearest_fill_changed_count", countChangedPixels(artifacts.dem_nearest, artifacts.dem_nearest_filled));
  stats.setCount("dem_idw_fill_changed_count", countChangedPixels(artifacts.dem_idw, artifacts.dem_idw_filled));
  stats.setCount("filtered_point_count", artifacts.filtered_points.points.size());
  stats.setCount("ground_seed_count", artifacts.seed_points.points.size());
  stats.setCount("ground_point_count", artifacts.ground_points.points.size());
  stats.setCount("nonground_point_count", artifacts.nonground_points.points.size());
  stats.setCount("dem_cols", artifacts.dem_mask.cols);
  stats.setCount("dem_rows", artifacts.dem_mask.rows);
  stats.setValue("dem_cell_size", config.dem.cell_size);
  stats.setValue("selected_output_cell_size", selected_config.dem.cell_size);
  stats.setValue("edge_shrink_cells", selected_config.dem.edge_shrink_cells);
  stats.setValue("object_mask_ratio", summarizeMaskRaster(artifacts.dtm_object_mask).active_ratio);
  stats.setValue("analysis_vs_display_rmse", computeRmse(artifacts.dtm_analysis, artifacts.dtm_display));

  if (raw_summary.valid_ratio > 0.0 && ground_summary.valid_ratio < raw_summary.valid_ratio * 0.25) {
    logger.warn("Ground direct support remains much lower than raw direct support; final DTM will rely on coarser domain.");
  }

  recordAllRasterStats(artifacts, stats);
}

ProcessingArtifacts MainController::processSingleCloud(PointCloud cloud,
                                                       const DEMConfig& config,
                                                       Logger& logger,
                                                       ProcessStats& stats,
                                                       const Bounds* forced_grid_bounds) const {
  auto artifacts = processSupportStage(std::move(cloud), config, logger, stats, forced_grid_bounds);
  finalizeArtifacts(artifacts, config, logger, stats, nullptr);
  return artifacts;
}

ProcessingArtifacts MainController::processTiles(const DEMConfig& config,
                                                 Logger& logger,
                                                 ProcessStats& stats,
                                                 const PointCloudScanSummary& summary) const {
  const auto build_result = tile_manager_.materializeTiles(config.input.file_path, summary.bounds, config, logger, stats);

  ProcessingArtifacts merged;
  merged.dem_raw_direct = dem_engine_.createGrid(summary.bounds, config, false);
  merged.dem_ground_direct = dem_engine_.createGrid(summary.bounds, config, false);

  for (const auto& tile : build_result.tiles) {
    if (tile.point_count == 0U) {
      continue;
    }
    logger.info("Processing tile " + std::to_string(tile.id) + " with " + std::to_string(tile.point_count) + " buffered points");
    auto tile_cloud = loadTilePointCloud(tile.temp_points_file);
    auto tile_artifacts = processSupportStage(std::move(tile_cloud), config, logger, stats, &tile.main_bounds);

    mergeMinimumRasterInto(tile_artifacts.dem_raw_direct, merged.dem_raw_direct);
    mergeMinimumRasterInto(tile_artifacts.dem_ground_direct, merged.dem_ground_direct);

    appendPointsInBounds(tile_artifacts.filtered_points, merged.filtered_points, tile.main_bounds);
    appendPointsInBounds(tile_artifacts.seed_points, merged.seed_points, tile.main_bounds);
    appendPointsInBounds(tile_artifacts.ground_points, merged.ground_points, tile.main_bounds);
    appendPointsInBounds(tile_artifacts.nonground_points, merged.nonground_points, tile.main_bounds);
  }

  merged.dem_support_mask = dem_engine_.buildSupportMask(merged.dem_ground_direct);
  finalizeArtifacts(merged, config, logger, stats, &build_result.tiles);
  return merged;
}

}  // namespace dem
