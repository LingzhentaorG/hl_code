#include "dem/Config.hpp"

#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

std::filesystem::path writeConfig(const std::filesystem::path& dir,
                                  const std::string& file_name,
                                  const std::string& dem_body) {
  std::filesystem::create_directories(dir);
  const auto config_path = dir / file_name;
  std::ofstream out(config_path);
  out << "{\n"
         "  \"input\": { \"file_path\": \"sample.ply\" },\n"
         "  \"crs\": { \"epsg_code\": 4326 },\n"
         "  \"ground\": { \"search_radius\": 0.0, \"seed_grid_size\": 0.0 },\n"
      << "  \"dem\": " << dem_body << ",\n"
      << "  \"tile\": { \"mode\": \"auto\", \"tile_buffer\": 0.0, \"tile_size_cells\": 1000 },\n"
         "  \"output\": { \"directory\": \"out\" }\n"
         "}\n";
  return config_path;
}

}  // namespace

TEST_CASE("config manager loads current defaults and normalizes derived values") {
  const auto temp_dir = std::filesystem::temp_directory_path() / "dem_config_current_test";
  std::filesystem::remove_all(temp_dir);

  const auto config_path = writeConfig(
      temp_dir, "config.json", R"({ "idw_power": 2.0, "idw_max_distance": 0.0 })");

  dem::ConfigManager manager;
  const auto config = manager.load(config_path, {"tile.mode=\"forced\""});

  CHECK(config.dem.cell_size == doctest::Approx(8.0));
  CHECK(config.ground.seed_grid_size == doctest::Approx(24.0));
  CHECK(config.ground.search_radius == doctest::Approx(48.0));
  CHECK(config.dem.idw_k == 12U);
  CHECK(config.dem.idw_min_points == 3U);
  CHECK(config.dem.idw_max_distance == doctest::Approx(0.0));
  CHECK(config.dem.idw_allow_fallback);
  CHECK(config.dem.idw_power == doctest::Approx(2.0));
  CHECK(config.dem.boundary_outline_cells == doctest::Approx(1.0));
  CHECK(config.tile.tile_buffer == doctest::Approx(48.0));
  CHECK(config.tile.mode == dem::TileMode::Forced);
  CHECK_FALSE(config.output.write_debug_pointclouds);
}

TEST_CASE("config manager rejects removed DEM fields") {
  const auto temp_dir = std::filesystem::temp_directory_path() / "dem_config_removed_keys_test";
  std::filesystem::remove_all(temp_dir);
  dem::ConfigManager manager;

  const std::vector<std::pair<std::string, std::string>> removed_fields{
      {"idw_mode", R"({ "cell_size": 2.0, "idw_mode": "arcgis_like", "idw_power": 2.0 })"},
      {"idw_radius", R"({ "cell_size": 2.0, "idw_radius": 24.0, "idw_power": 2.0 })"},
      {"nearest_max_distance", R"({ "cell_size": 2.0, "nearest_max_distance": 18.0, "idw_power": 2.0 })"},
      {"idw_max_points", R"({ "cell_size": 2.0, "idw_max_points": 12, "idw_power": 2.0 })"},
      {"edge_shrink_cells", R"({ "cell_size": 2.0, "edge_shrink_cells": 1.0, "idw_power": 2.0 })"},
      {"enable_edge_mask", R"({ "cell_size": 2.0, "enable_edge_mask": true, "idw_power": 2.0 })"},
      {"fill_holes", R"({ "cell_size": 2.0, "fill_holes": true, "idw_power": 2.0 })"},
      {"fill_max_radius", R"({ "cell_size": 2.0, "fill_max_radius": 2, "idw_power": 2.0 })"},
      {"fill_min_neighbors", R"({ "cell_size": 2.0, "fill_min_neighbors": 4, "idw_power": 2.0 })"},
  };

  for (const auto& [field_name, dem_body] : removed_fields) {
    CAPTURE(field_name);
    const auto config_path = writeConfig(temp_dir, field_name + ".json", dem_body);
    CHECK_THROWS(manager.load(config_path, {}));
  }
}

TEST_CASE("config manager validates official DEM constraints") {
  const auto temp_dir = std::filesystem::temp_directory_path() / "dem_config_invalid_values_test";
  std::filesystem::remove_all(temp_dir);
  dem::ConfigManager manager;

  const auto invalid_k =
      writeConfig(temp_dir, "invalid_k.json", R"({ "cell_size": 2.0, "idw_k": 0, "idw_power": 2.0 })");
  CHECK_THROWS(manager.load(invalid_k, {}));

  const auto invalid_neighbors = writeConfig(
      temp_dir, "invalid_neighbors.json", R"({ "cell_size": 2.0, "idw_k": 2, "idw_min_points": 3, "idw_power": 2.0 })");
  CHECK_THROWS(manager.load(invalid_neighbors, {}));

  const auto invalid_power =
      writeConfig(temp_dir, "invalid_power.json", R"({ "cell_size": 2.0, "idw_power": 0.0 })");
  CHECK_THROWS(manager.load(invalid_power, {}));

  const auto invalid_outline = writeConfig(
      temp_dir, "invalid_outline.json", R"({ "cell_size": 2.0, "idw_power": 2.0, "boundary_outline_cells": -1.0 })");
  CHECK_THROWS(manager.load(invalid_outline, {}));
}
