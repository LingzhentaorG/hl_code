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

PointCloud loadTilePointCloud(const std::filesystem::path& xyz_path) {
  PointCloud cloud;
  std::ifstream stream(xyz_path);
  if (!stream) {
    throw std::runtime_error("Unable to open tile file: " + pathToUtf8String(xyz_path));
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

void recordAllRasterStats(const ProcessingArtifacts& artifacts, ProcessStats& stats) {
  recordContinuousRasterStats("dem_raw", artifacts.dem_raw, stats);
  const auto dem_summary = summarizeContinuousRaster(artifacts.dem);
  const auto domain_summary = summarizeMaskRaster(artifacts.dem_domain_mask);
  const auto boundary_summary = summarizeMaskRaster(artifacts.dem_boundary_mask);
  const auto object_summary = summarizeMaskRaster(artifacts.dem_object_mask);

  recordContinuousRasterStats("dem", artifacts.dem, stats);
  recordMaskRasterStats("dem_domain_mask", artifacts.dem_domain_mask, stats);
  recordMaskRasterStats("dem_boundary_mask", artifacts.dem_boundary_mask, stats);
  recordMaskRasterStats("dem_object_mask", artifacts.dem_object_mask, stats);

  stats.setValue("dem_valid_ratio", dem_summary.valid_ratio);
  stats.setValue("dem_domain_mask_active_ratio", domain_summary.active_ratio);
  stats.setValue("dem_boundary_mask_active_ratio", boundary_summary.active_ratio);
  stats.setValue("dem_object_mask_active_ratio", object_summary.active_ratio);
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
  config_manager_.writeResolvedConfig(config, config.output.directory / "log" / "config_used.json");

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

MainController::SupportStageArtifacts MainController::processSupportStage(PointCloud cloud,
                                                                          const DEMConfig& config,
                                                                          Logger& logger,
                                                                          ProcessStats& stats,
                                                                          const Bounds* forced_grid_bounds) const {
  SupportStageArtifacts artifacts;

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

  const RasterGrid grid_template = dem_engine_.createGrid(grid_bounds, config);
  artifacts.raw_direct = dem_engine_.rasterizeMinimum(preprocessed, grid_template);

  artifacts.filtered_points = ground_filter_engine_.removeOutliers(std::move(preprocessed), config, logger, stats);
  ground_filter_engine_.extractGround(
      artifacts.filtered_points, artifacts.seed_points, artifacts.ground_points, artifacts.nonground_points, config, logger, stats);

  artifacts.ground_direct = dem_engine_.rasterizeMinimum(artifacts.ground_points, grid_template);
  return artifacts;
}

ProcessingArtifacts MainController::finalizeArtifacts(const SupportStageArtifacts& support_stage,
                                                      const DEMConfig& config,
                                                      Logger& logger,
                                                      ProcessStats& stats) const {
  ProcessingArtifacts artifacts;
  artifacts.filtered_points = support_stage.filtered_points;
  artifacts.ground_points = support_stage.ground_points;

  const auto raw_summary = summarizeContinuousRaster(support_stage.raw_direct);
  const auto ground_summary = summarizeContinuousRaster(support_stage.ground_direct);
  if (raw_summary.valid_ratio > 0.0 && ground_summary.valid_ratio < raw_summary.valid_ratio * 0.25) {
    logger.warn("Ground coverage is significantly lower than raw direct coverage; ground filter may be too conservative.");
  }

  auto qc_object_mask = dem_engine_.buildObjectMask(support_stage.raw_direct, support_stage.ground_direct, config, logger, stats);
  auto qc_support =
      dem_engine_.refineAcceptedSupport(support_stage.raw_direct, support_stage.ground_direct, config, logger, stats, &qc_object_mask);
  auto qc_domain = dem_engine_.buildDomainMask(support_stage.filtered_points, support_stage.raw_direct, config, logger, stats);

  artifacts.dem_raw = qc_support;
  artifacts.dem_domain_mask = qc_domain;
  artifacts.dem_object_mask = qc_object_mask;
  artifacts.dem = dem_engine_.interpolateIdw(artifacts.dem_raw, artifacts.dem_domain_mask, config, logger, stats);
  artifacts.dem_boundary_mask =
      dem_engine_.buildEdgeMask(support_stage.filtered_points, artifacts.dem_domain_mask, config, logger, stats);

  stats.setCount("filtered_point_count", support_stage.filtered_points.points.size());
  stats.setCount("ground_seed_count", support_stage.seed_points.points.size());
  stats.setCount("ground_point_count", support_stage.ground_points.points.size());
  stats.setCount("nonground_point_count", support_stage.nonground_points.points.size());
  stats.setCount("dem_cols", artifacts.dem_domain_mask.cols);
  stats.setCount("dem_rows", artifacts.dem_domain_mask.rows);
  stats.setValue("dem_cell_size", config.dem.cell_size);
  stats.setValue("selected_output_cell_size", config.dem.cell_size);
  stats.setValue("dem_boundary_outline_cells", config.dem.boundary_outline_cells);
  stats.setValue("dem_object_mask_ratio", summarizeMaskRaster(artifacts.dem_object_mask).active_ratio);
  stats.setValue("raw_direct_active_ratio", raw_summary.valid_ratio);
  stats.setValue("ground_direct_active_ratio", ground_summary.valid_ratio);
  stats.setValue("dem_raw_valid_ratio", summarizeContinuousRaster(artifacts.dem_raw).valid_ratio);

  if (raw_summary.valid_ratio > 0.0 && ground_summary.valid_ratio < raw_summary.valid_ratio * 0.25) {
    logger.warn("Ground direct support remains much lower than raw direct support; final DEM may contain larger interpolated gaps.");
  }

  recordAllRasterStats(artifacts, stats);
  return artifacts;
}

ProcessingArtifacts MainController::processSingleCloud(PointCloud cloud,
                                                       const DEMConfig& config,
                                                       Logger& logger,
                                                       ProcessStats& stats,
                                                       const Bounds* forced_grid_bounds) const {
  auto support_stage = processSupportStage(std::move(cloud), config, logger, stats, forced_grid_bounds);
  return finalizeArtifacts(support_stage, config, logger, stats);
}

ProcessingArtifacts MainController::processTiles(const DEMConfig& config,
                                                 Logger& logger,
                                                 ProcessStats& stats,
                                                 const PointCloudScanSummary& summary) const {
  const auto build_result = tile_manager_.materializeTiles(config.input.file_path, summary.bounds, config, logger, stats);

  SupportStageArtifacts merged;
  merged.raw_direct = dem_engine_.createGrid(summary.bounds, config);
  merged.ground_direct = dem_engine_.createGrid(summary.bounds, config);

  for (const auto& tile : build_result.tiles) {
    if (tile.point_count == 0U) {
      continue;
    }
    logger.info("Processing tile " + std::to_string(tile.id) + " with " + std::to_string(tile.point_count) + " buffered points");
    auto tile_cloud = loadTilePointCloud(tile.temp_points_file);
    auto tile_artifacts = processSupportStage(std::move(tile_cloud), config, logger, stats, &tile.main_bounds);

    mergeMinimumRasterInto(tile_artifacts.raw_direct, merged.raw_direct);
    mergeMinimumRasterInto(tile_artifacts.ground_direct, merged.ground_direct);

    appendPointsInBounds(tile_artifacts.filtered_points, merged.filtered_points, tile.main_bounds);
    appendPointsInBounds(tile_artifacts.seed_points, merged.seed_points, tile.main_bounds);
    appendPointsInBounds(tile_artifacts.ground_points, merged.ground_points, tile.main_bounds);
    appendPointsInBounds(tile_artifacts.nonground_points, merged.nonground_points, tile.main_bounds);
  }

  return finalizeArtifacts(merged, config, logger, stats);
}

}  // namespace dem
