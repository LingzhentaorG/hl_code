#include "dem/Config.hpp"

#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>

TEST_CASE("config manager loads and normalizes derived values") {
  const auto temp_dir = std::filesystem::temp_directory_path() / "dem_config_test";
  std::filesystem::create_directories(temp_dir);
  const auto config_path = temp_dir / "config.json";

  std::ofstream out(config_path);
  out << R"({
    "input": { "file_path": "sample.ply" },
    "crs": { "epsg_code": 4326 },
    "ground": { "search_radius": 0.0, "seed_grid_size": 0.0 },
    "dem": { "cell_size": 2.0, "idw_radius": 0.0, "nearest_max_distance": 0.0, "idw_power": 2.0 },
    "tile": { "mode": "auto", "tile_buffer": 0.0, "tile_size_cells": 1000 },
    "output": { "directory": "out" }
  })";
  out.close();

  dem::ConfigManager manager;
  const auto config = manager.load(config_path, {"dem.cell_size=3.0", "tile.mode=\"forced\""});

  CHECK(config.dem.cell_size == doctest::Approx(3.0));
  CHECK(config.ground.search_radius == doctest::Approx(9.0));
  CHECK(config.ground.seed_grid_size == doctest::Approx(9.0));
  CHECK(config.dem.idw_radius == doctest::Approx(9.0));
  CHECK(config.tile.tile_buffer == doctest::Approx(9.0));
  CHECK(config.tile.mode == dem::TileMode::Forced);
}
