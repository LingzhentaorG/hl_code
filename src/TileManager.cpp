/**
 * @file TileManager.cpp
 * @brief Tile 分块管理器实现
 */

#include "dem/TileManager.hpp"

#include "dem/Utils.hpp"

#include <algorithm>
#include <fstream>
#include <limits>
#include <unordered_map>

namespace dem {
namespace {

constexpr std::size_t kMaxOpenTileStreams = 64U;

struct CachedTileStream {
  std::ofstream stream;
  std::size_t last_used = 0U;
};

class TileStreamCache {
 public:
  TileStreamCache(const std::vector<TileDefinition>& tiles, std::size_t max_open_streams)
      : tiles_(tiles), max_open_streams_(std::max<std::size_t>(1U, max_open_streams)) {}

  std::ofstream& acquire(std::size_t tile_index) {
    ++access_counter_;
    auto it = cache_.find(tile_index);
    if (it != cache_.end()) {
      it->second.last_used = access_counter_;
      return it->second.stream;
    }

    if (cache_.size() >= max_open_streams_) {
      evictLeastRecentlyUsed();
    }

    CachedTileStream entry;
    entry.stream.open(tiles_[tile_index].temp_points_file, std::ios::out | std::ios::app);
    if (!entry.stream) {
      throw std::runtime_error("Unable to open tile temp file: " + tiles_[tile_index].temp_points_file.string());
    }
    entry.last_used = access_counter_;
    auto [inserted, _] = cache_.emplace(tile_index, std::move(entry));
    return inserted->second.stream;
  }

 private:
  void evictLeastRecentlyUsed() {
    auto oldest = cache_.end();
    for (auto it = cache_.begin(); it != cache_.end(); ++it) {
      if (oldest == cache_.end() || it->second.last_used < oldest->second.last_used) {
        oldest = it;
      }
    }
    if (oldest != cache_.end()) {
      oldest->second.stream.close();
      cache_.erase(oldest);
    }
  }

  const std::vector<TileDefinition>& tiles_;
  std::size_t max_open_streams_ = 0U;
  std::size_t access_counter_ = 0U;
  std::unordered_map<std::size_t, CachedTileStream> cache_;
};

}  // namespace

bool TileManager::shouldUseTiles(const PointCloudScanSummary& summary, const DEMConfig& config) const {
  if (config.tile.mode == TileMode::Forced) {
    return true;
  }
  if (config.tile.mode == TileMode::Disabled) {
    return false;
  }
  return summary.estimated_memory_mb > static_cast<double>(config.tile.memory_limit_mb);
}

std::vector<TileDefinition> TileManager::createTiles(const Bounds& bounds, const DEMConfig& config) const {
  if (!bounds.valid()) {
    return {};
  }

  const double tile_span = std::max(1.0, static_cast<double>(config.tile.tile_size_cells)) * config.dem.cell_size;
  const std::size_t cols =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(bounds.width() / tile_span)));
  const std::size_t rows =
      std::max<std::size_t>(1U, static_cast<std::size_t>(std::ceil(bounds.height() / tile_span)));

  std::vector<TileDefinition> tiles;
  tiles.reserve(rows * cols);
  std::size_t next_id = 0;
  for (std::size_t row = 0; row < rows; ++row) {
    for (std::size_t col = 0; col < cols; ++col) {
      TileDefinition tile;
      tile.id = next_id++;
      tile.row = row;
      tile.col = col;
      tile.main_bounds.xmin = bounds.xmin + static_cast<double>(col) * tile_span;
      tile.main_bounds.xmax = std::min(bounds.xmax, tile.main_bounds.xmin + tile_span);
      tile.main_bounds.ymin = bounds.ymin + static_cast<double>(row) * tile_span;
      tile.main_bounds.ymax = std::min(bounds.ymax, tile.main_bounds.ymin + tile_span);
      tile.main_bounds.zmin = bounds.zmin;
      tile.main_bounds.zmax = bounds.zmax;

      tile.buffered_bounds = tile.main_bounds.expanded(config.tile.tile_buffer);
      tile.buffered_bounds.xmin = std::max(bounds.xmin, tile.buffered_bounds.xmin);
      tile.buffered_bounds.xmax = std::min(bounds.xmax, tile.buffered_bounds.xmax);
      tile.buffered_bounds.ymin = std::max(bounds.ymin, tile.buffered_bounds.ymin);
      tile.buffered_bounds.ymax = std::min(bounds.ymax, tile.buffered_bounds.ymax);
      tile.buffered_bounds.zmin = bounds.zmin;
      tile.buffered_bounds.zmax = bounds.zmax;
      tiles.push_back(tile);
    }
  }
  return tiles;
}

TileBuildResult TileManager::materializeTiles(const std::filesystem::path& input_path,
                                              const Bounds& bounds,
                                              const DEMConfig& config,
                                              Logger& logger,
                                              ProcessStats& stats) const {
  ScopedTimer timer(stats, "tile_materialize_seconds");
  TileBuildResult result;
  result.tiles = createTiles(bounds, config);
  result.temp_directory = config.output.directory / "tile";
  std::error_code remove_error;
  std::filesystem::remove_all(result.temp_directory, remove_error);
  ensureDirectory(result.temp_directory);

  for (std::size_t i = 0; i < result.tiles.size(); ++i) {
    result.tiles[i].temp_points_file = result.temp_directory / ("tile_" + std::to_string(result.tiles[i].id) + ".xyz");
  }

  const double tile_span = std::max(1.0, static_cast<double>(config.tile.tile_size_cells)) * config.dem.cell_size;
  const std::size_t tile_cols = result.tiles.empty() ? 0U : (result.tiles.back().col + 1U);
  const std::size_t tile_rows = result.tiles.empty() ? 0U : (result.tiles.back().row + 1U);
  TileStreamCache stream_cache(result.tiles, kMaxOpenTileStreams);
  stats.setCount("tile_open_stream_limit", kMaxOpenTileStreams);

  InputManager input_manager;
  input_manager.streamPoints(input_path, [&](const Point3D& point) {
    if (tile_rows == 0U || tile_cols == 0U) {
      return;
    }

    const double relative_x = point.x - bounds.xmin;
    const double relative_y = point.y - bounds.ymin;
    const int min_col = std::max(
        0,
        static_cast<int>(std::floor((relative_x - config.tile.tile_buffer) / tile_span)) - 1);
    const int max_col = std::min(
        static_cast<int>(tile_cols) - 1,
        static_cast<int>(std::floor((relative_x + config.tile.tile_buffer) / tile_span)) + 1);
    const int min_row = std::max(
        0,
        static_cast<int>(std::floor((relative_y - config.tile.tile_buffer) / tile_span)) - 1);
    const int max_row = std::min(
        static_cast<int>(tile_rows) - 1,
        static_cast<int>(std::floor((relative_y + config.tile.tile_buffer) / tile_span)) + 1);

    for (int row = min_row; row <= max_row; ++row) {
      for (int col = min_col; col <= max_col; ++col) {
        const auto tile_index = static_cast<std::size_t>(row) * tile_cols + static_cast<std::size_t>(col);
        if (tile_index >= result.tiles.size() || !result.tiles[tile_index].buffered_bounds.containsXY(point.x, point.y)) {
          continue;
        }
        auto& stream = stream_cache.acquire(tile_index);
        stream << point.x << " " << point.y << " " << point.z << "\n";
        result.tiles[tile_index].point_count += 1U;
      }
    }
  });

  logger.info("Materialized " + std::to_string(result.tiles.size()) + " tiles.");
  return result;
}

}  // namespace dem
