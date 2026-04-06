/**
 * @file OutputManager.cpp
 * @brief 输出管理器实现：点云、GeoTIFF、PNG、统计和报告写出
 */

#include "dem/OutputManager.hpp"

#include "dem/Utils.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
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

#ifndef _WIN32
std::string quoteForShell(const std::filesystem::path& path) {
  return "\"" + path.generic_string() + "\"";
}
#endif

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

void writeContinuousRasterSection(std::ofstream& stream, const std::string& name, const RasterGrid& grid) {
  const auto summary = summarizeContinuousRaster(grid);
  stream << "[raster." << name << "]\n";
  stream << "rows=" << grid.rows << "\n";
  stream << "cols=" << grid.cols << "\n";
  stream << "cell_size=" << grid.cell_size << "\n";
  stream << "nodata=" << grid.nodata << "\n";
  stream << "valid_count=" << summary.valid_count << "\n";
  stream << "nodata_count=" << summary.nodata_count << "\n";
  stream << "valid_ratio=" << summary.valid_ratio << "\n";
  stream << "nodata_ratio=" << summary.nodata_ratio << "\n\n";
}

void writeMaskSection(std::ofstream& stream, const std::string& name, const RasterGrid& grid) {
  const auto summary = summarizeMaskRaster(grid);
  stream << "[raster." << name << "]\n";
  stream << "rows=" << grid.rows << "\n";
  stream << "cols=" << grid.cols << "\n";
  stream << "cell_size=" << grid.cell_size << "\n";
  stream << "active_count=" << summary.active_count << "\n";
  stream << "inactive_count=" << summary.inactive_count << "\n";
  stream << "active_ratio=" << summary.active_ratio << "\n";
  stream << "inactive_ratio=" << summary.inactive_ratio << "\n\n";
}

}  // namespace

void OutputManager::writeArtifacts(const ProcessingArtifacts& artifacts,
                                   const DEMConfig& config,
                                   const CRSDefinition& crs,
                                   Logger& logger,
                                   ProcessStats& stats) const {
  ScopedTimer timer(stats, "output_write_seconds");

  const auto dem_dir = config.output.directory / "dem";
  const auto pointcloud_dir = config.output.directory / "pointcloud";
  const auto log_dir = config.output.directory / "log";
  ensureDirectory(dem_dir);
  ensureDirectory(pointcloud_dir);
  ensureDirectory(log_dir);

  writePointCloudPly(artifacts.filtered_points, pointcloud_dir / "filtered_points.ply");
  writePointCloudPly(artifacts.seed_points, pointcloud_dir / "seed_points.ply");
  writePointCloudPly(artifacts.ground_points, pointcloud_dir / "ground_points.ply");
  writePointCloudPly(artifacts.nonground_points, pointcloud_dir / "nonground_points.ply");

  writeGeoTiff(artifacts.dem_raw_direct, config, crs, dem_dir / "dem_raw_direct.tif", logger);
  writeGeoTiff(artifacts.dem_ground_direct, config, crs, dem_dir / "dem_ground_direct.tif", logger);
  writeGeoTiff(artifacts.dem_nearest, config, crs, dem_dir / "dem_nearest.tif", logger);
  writeGeoTiff(artifacts.dem_idw, config, crs, dem_dir / "dem_idw.tif", logger);
  writeGeoTiff(artifacts.dem_nearest_filled, config, crs, dem_dir / "dem_nearest_filled.tif", logger);
  writeGeoTiff(artifacts.dem_idw_filled, config, crs, dem_dir / "dem_idw_filled.tif", logger);
  writeGeoTiff(artifacts.dem_support_mask, config, crs, dem_dir / "dem_support_mask.tif", logger);
  writeGeoTiff(artifacts.dem_mask, config, crs, dem_dir / "dem_mask.tif", logger);
  writeGeoTiff(artifacts.dem_edge_mask, config, crs, dem_dir / "dem_edge_mask.tif", logger);
  writeGeoTiff(artifacts.dtm_analysis, config, crs, dem_dir / "dtm_analysis.tif", logger);
  writeGeoTiff(artifacts.dtm_display, config, crs, dem_dir / "dtm_display.tif", logger);
  writeGeoTiff(artifacts.dtm_object_mask, config, crs, dem_dir / "dtm_object_mask.tif", logger);

  writeStats(artifacts, config, stats, log_dir / "stats.txt");

  nlohmann::json report;
  report["metadata"]["input_file"] = config.input.file_path.string();
  report["metadata"]["output_directory"] = config.output.directory.string();
  report["crs"]["authority_name"] = crs.authority_name;
  report["crs"]["epsg_code"] = crs.epsg_code;
  report["crs"]["known"] = crs.known;
  report["raster"]["dem_raw_direct"]["valid_ratio"] = summarizeContinuousRaster(artifacts.dem_raw_direct).valid_ratio;
  report["raster"]["dem_ground_direct"]["valid_ratio"] = summarizeContinuousRaster(artifacts.dem_ground_direct).valid_ratio;
  report["raster"]["dem_nearest"]["valid_ratio"] = summarizeContinuousRaster(artifacts.dem_nearest).valid_ratio;
  report["raster"]["dem_idw"]["valid_ratio"] = summarizeContinuousRaster(artifacts.dem_idw).valid_ratio;
  report["raster"]["dem_mask"]["active_ratio"] = summarizeMaskRaster(artifacts.dem_mask).active_ratio;
  report["raster"]["dem_support_mask"]["active_ratio"] = summarizeMaskRaster(artifacts.dem_support_mask).active_ratio;
  report["raster"]["dem_edge_mask"]["active_ratio"] = summarizeMaskRaster(artifacts.dem_edge_mask).active_ratio;
  report["raster"]["dtm_analysis"]["valid_ratio"] = summarizeContinuousRaster(artifacts.dtm_analysis).valid_ratio;
  report["raster"]["dtm_display"]["valid_ratio"] = summarizeContinuousRaster(artifacts.dtm_display).valid_ratio;
  report["raster"]["dtm_object_mask"]["active_ratio"] = summarizeMaskRaster(artifacts.dtm_object_mask).active_ratio;
  report["pointcloud"]["ground_points"] = artifacts.ground_points.points.size();
  report["pointcloud"]["nonground_points"] = artifacts.nonground_points.points.size();
  report["stats"]["counts"] = stats.counts;
  report["stats"]["values"] = stats.values;
  report["stats"]["durations_seconds"] = stats.durations_seconds;
  std::ofstream report_stream(log_dir / "report.json");
  report_stream << std::setw(2) << report << '\n';

  logger.info("Output files written to " + config.output.directory.string());
}

void OutputManager::writeStats(const ProcessingArtifacts& artifacts,
                               const DEMConfig&,
                               const ProcessStats& stats,
                               const std::filesystem::path& path) const {
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("Unable to open stats file: " + path.string());
  }

  stream << "tile_mode_used=" << (stats.tile_mode_used ? 1 : 0) << "\n\n";

  stream << "[counts]\n";
  {
    std::vector<std::pair<std::string, std::size_t>> items(stats.counts.begin(), stats.counts.end());
    std::sort(items.begin(), items.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    for (const auto& [key, value] : items) {
      stream << key << "=" << value << "\n";
    }
    stream << "\n";
  }

  stream << "[values]\n";
  {
    std::vector<std::pair<std::string, double>> items(stats.values.begin(), stats.values.end());
    std::sort(items.begin(), items.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    for (const auto& [key, value] : items) {
      stream << key << "=" << value << "\n";
    }
    stream << "\n";
  }

  stream << "[durations_seconds]\n";
  {
    std::vector<std::pair<std::string, double>> items(stats.durations_seconds.begin(), stats.durations_seconds.end());
    std::sort(items.begin(), items.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    for (const auto& [key, value] : items) {
      stream << key << "=" << value << "\n";
    }
    stream << "\n";
  }

  stream << "[ground_added_per_iteration]\n";
  for (std::size_t i = 0; i < stats.ground_added_per_iteration.size(); ++i) {
    stream << "iter_" << (i + 1) << "=" << stats.ground_added_per_iteration[i] << "\n";
  }
  stream << "\n";

  stream << "[reference_mode_counts]\n";
  {
    std::vector<std::pair<std::string, std::size_t>> items(
        stats.reference_mode_counts.begin(), stats.reference_mode_counts.end());
    std::sort(items.begin(), items.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    for (const auto& [key, value] : items) {
      stream << key << "=" << value << "\n";
    }
    stream << "\n";
  }

  writeContinuousRasterSection(stream, "dem_raw_direct", artifacts.dem_raw_direct);
  writeContinuousRasterSection(stream, "dem_ground_direct", artifacts.dem_ground_direct);
  writeContinuousRasterSection(stream, "dem_nearest", artifacts.dem_nearest);
  writeContinuousRasterSection(stream, "dem_idw", artifacts.dem_idw);
  writeContinuousRasterSection(stream, "dem_nearest_filled", artifacts.dem_nearest_filled);
  writeContinuousRasterSection(stream, "dem_idw_filled", artifacts.dem_idw_filled);
  writeContinuousRasterSection(stream, "dtm_analysis", artifacts.dtm_analysis);
  writeContinuousRasterSection(stream, "dtm_display", artifacts.dtm_display);
  writeMaskSection(stream, "dem_support_mask", artifacts.dem_support_mask);
  writeMaskSection(stream, "dem_mask", artifacts.dem_mask);
  writeMaskSection(stream, "dem_edge_mask", artifacts.dem_edge_mask);
  writeMaskSection(stream, "dtm_object_mask", artifacts.dtm_object_mask);

  stream << "[pointclouds]\n";
  stream << "filtered_points=" << artifacts.filtered_points.points.size() << "\n";
  stream << "seed_points=" << artifacts.seed_points.points.size() << "\n";
  stream << "ground_points=" << artifacts.ground_points.points.size() << "\n";
  stream << "nonground_points=" << artifacts.nonground_points.points.size() << "\n";
}

void OutputManager::writePointCloudPly(const PointCloud& cloud, const std::filesystem::path& path) const {
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("Unable to open PLY output: " + path.string());
  }

  stream << "ply\n";
  stream << "format ascii 1.0\n";
  stream << "element vertex " << cloud.points.size() << "\n";
  stream << "property float x\n";
  stream << "property float y\n";
  stream << "property float z\n";
  stream << "property uchar classification\n";
  stream << "end_header\n";

  for (const auto& point : cloud.points) {
    const int classification = point.ground ? 1 : 0;
    stream << point.x << " " << point.y << " " << point.z << " " << classification << "\n";
  }
}

void OutputManager::writeGeoTiff(const RasterGrid& grid,
                                 const DEMConfig&,
                                 const CRSDefinition& crs,
                                 const std::filesystem::path& path,
                                 Logger& logger) const {
  ensureDirectory(path.parent_path());

  const auto stem = path.stem().string();
  const auto png_path = path.parent_path() / (stem + ".png");
  const auto unique_id = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto bridge_dir = std::filesystem::temp_directory_path() / "dem_gtiff_bridge";
  ensureDirectory(bridge_dir);
  const auto bin_path = bridge_dir / (stem + "_" + unique_id + ".bin");
  const auto meta_path = bridge_dir / (stem + "_" + unique_id + ".meta.json");

  {
    std::ofstream bin_stream(bin_path, std::ios::binary);
    if (!bin_stream) {
      throw std::runtime_error("Unable to open raster bin output: " + bin_path.string());
    }
    bin_stream.write(reinterpret_cast<const char*>(grid.values.data()),
                     static_cast<std::streamsize>(grid.values.size() * sizeof(double)));
  }

  std::string png_mode = "dem";
  if (stem.find("edge_mask") != std::string::npos) {
    png_mode = "edge";
  } else if (stem.find("object_mask") != std::string::npos) {
    png_mode = "object_mask";
  } else if (stem.find("support_mask") != std::string::npos) {
    png_mode = "support_mask";
  } else if (stem == "dem_mask") {
    png_mode = "mask";
  }

  nlohmann::json meta;
  meta["bin_path"] = bin_path.generic_string();
  meta["rows"] = grid.rows;
  meta["cols"] = grid.cols;
  meta["nodata"] = grid.nodata;
  meta["origin_x"] = grid.origin_x;
  meta["origin_y"] = grid.origin_y;
  meta["cell_size"] = grid.cell_size;
  meta["output_path"] = path.generic_string();
  meta["png_path"] = png_path.generic_string();
  meta["png_mode"] = png_mode;
  if (!crs.resolved_wkt.empty()) {
    meta["crs_wkt"] = crs.resolved_wkt;
  } else if (crs.epsg_code > 0) {
    meta["epsg_code"] = crs.epsg_code;
  }
  {
    std::ofstream meta_stream(meta_path);
    if (!meta_stream) {
      throw std::runtime_error("Unable to open GeoTIFF meta output: " + meta_path.string());
    }
    meta_stream << meta.dump(2);
  }

  const std::filesystem::path script_path = std::filesystem::path(DEM_SOURCE_DIR) / "scripts" / "write_geotiff.py";
  const std::filesystem::path python_path = DEM_PYTHON_EXECUTABLE;
#ifdef _WIN32
  const std::string command = "powershell -NoProfile -ExecutionPolicy Bypass -Command \"& '" +
                              python_path.generic_string() + "' '" + script_path.generic_string() + "' --meta '" +
                              meta_path.generic_string() + "'\"";
#else
  const std::string command =
      quoteForShell(python_path) + " " + quoteForShell(script_path) + " --meta " + quoteForShell(meta_path);
#endif
  if (std::system(command.c_str()) != 0) {
    throw std::runtime_error("GeoTIFF writer script failed for: " + path.string());
  }

  std::error_code ec;
  std::filesystem::remove(bin_path, ec);
  std::filesystem::remove(meta_path, ec);
  logger.info("Wrote GeoTIFF " + path.string());
}

}  // namespace dem
