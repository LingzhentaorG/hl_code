#include "dem/InputManager.hpp"
#include "dem/MainController.hpp"
#include "dem/SpatialIndexManager.hpp"

#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>

TEST_CASE("input manager inspects benchmark ply header") {
  dem::InputManager input_manager;
  const auto sample_path = std::filesystem::path(DEM_SOURCE_DIR) / "data" / "基准.ply";
  REQUIRE(std::filesystem::exists(sample_path));

  const auto header = input_manager.inspectHeader(sample_path);
  CHECK(header.ascii);
  CHECK(header.vertex_count == 17044);
  CHECK(header.x_index >= 0);
  CHECK(header.y_index >= 0);
  CHECK(header.z_index >= 0);
}

TEST_CASE("spatial index returns expected nearest neighbor") {
  std::vector<dem::Point3D> points(3);
  points[0].x = 0.0;
  points[0].y = 0.0;
  points[1].x = 10.0;
  points[1].y = 0.0;
  points[2].x = 0.0;
  points[2].y = 10.0;
  std::vector<std::size_t> indices{0, 1, 2};

  dem::SpatialIndexManager index;
  index.build2D(points, indices);
  const auto result = index.knn2D(0.1, 0.1, 1);
  REQUIRE(result.indices.size() == 1);
  CHECK(result.indices.front() == 0);
}

TEST_CASE("controller runs end-to-end on benchmark sample") {
  const auto source_dir = std::filesystem::path(DEM_SOURCE_DIR);
  const auto sample_path = source_dir / "data" / "基准.ply";
  REQUIRE(std::filesystem::exists(sample_path));

  const auto temp_root = std::filesystem::temp_directory_path() / "dem_pipeline_test";
  std::filesystem::remove_all(temp_root);
  std::filesystem::create_directories(temp_root);

  const auto config_path = temp_root / "pipeline_config.json";
  std::ofstream out(config_path);
  const auto sample_path_json = sample_path.generic_string();
  const auto output_dir_json = (temp_root / "out").generic_string();
  out << "{\n"
         "  \"input\": { \"file_path\": \"" << sample_path_json << "\" },\n"
         "  \"crs\": { \"epsg_code\": 4326 },\n"
         "  \"dem\": {\n"
         "    \"cell_size\": 8.0,\n"
         "    \"idw_power\": 2.0,\n"
         "    \"fill_holes\": true,\n"
         "    \"fill_max_radius\": 2,\n"
         "    \"fill_min_neighbors\": 4\n"
         "  },\n"
         "  \"tile\": { \"mode\": \"disabled\", \"tile_size_cells\": 1000, \"memory_limit_mb\": 512 },\n"
         "  \"output\": { \"directory\": \"" << output_dir_json << "\", \"write_png\": true, \"timestamp_subdir\": true }\n"
         "}\n";
  out.close();

  dem::MainController controller;
  CHECK(controller.run(config_path, {}) == 0);

  std::filesystem::path run_dir;
  for (const auto& entry : std::filesystem::directory_iterator(temp_root / "out")) {
    if (entry.is_directory()) {
      run_dir = entry.path();
      break;
    }
  }
  REQUIRE_FALSE(run_dir.empty());
  CHECK(std::filesystem::exists(run_dir / "dem" / "dem_idw.tif"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dem_idw.png"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dem_edge_mask.png"));
  CHECK(std::filesystem::exists(run_dir / "pointcloud" / "ground_points.ply"));
  CHECK(std::filesystem::exists(run_dir / "pointcloud" / "seed_points.ply"));
  CHECK(std::filesystem::exists(run_dir / "log" / "stats.txt"));
}

TEST_CASE("batch processing script produces summary and dataset outputs") {
  const auto source_dir = std::filesystem::path(DEM_SOURCE_DIR);
  const auto sample_path = source_dir / "data" / "基准.ply";
  REQUIRE(std::filesystem::exists(sample_path));

  const auto temp_root = std::filesystem::temp_directory_path() / "dem_batch_test";
  std::filesystem::remove_all(temp_root);
  std::filesystem::create_directories(temp_root / "data");
  std::filesystem::copy_file(sample_path, temp_root / "data" / "基准.ply", std::filesystem::copy_options::overwrite_existing);

  const auto config_path = temp_root / "batch_config.json";
  std::ofstream out(config_path);
  out << "{\n"
         "  \"input\": { \"file_path\": \"" << sample_path.generic_string() << "\" },\n"
         "  \"crs\": { \"epsg_code\": 4326 },\n"
         "  \"dem\": { \"cell_size\": 8.0, \"idw_power\": 2.0 },\n"
         "  \"tile\": { \"mode\": \"disabled\", \"tile_size_cells\": 1000, \"memory_limit_mb\": 512 },\n"
         "  \"output\": { \"directory\": \"" << (temp_root / "out").generic_string()
      << "\", \"write_png\": true, \"timestamp_subdir\": false }\n"
         "}\n";
  out.close();

  const auto script_path = source_dir / "scripts" / "batch_process.py";
  const auto cli_path = source_dir / "build" / "dem_cli.exe";
  std::string command;
#ifdef _WIN32
  command = "cmd /c \"\"" + std::string(DEM_PYTHON_EXECUTABLE) + "\" \"" + script_path.generic_string() +
            "\" --cli \"" + cli_path.generic_string() + "\" --config \"" + config_path.generic_string() +
            "\" --input-dir \"" + (temp_root / "data").generic_string() + "\" --output-root \"" +
            (temp_root / "batch_out").generic_string() + "\"\"";
#else
  command = "\"" + std::string(DEM_PYTHON_EXECUTABLE) + "\" \"" + script_path.generic_string() + "\" --cli \"" +
            cli_path.generic_string() + "\" --config \"" + config_path.generic_string() + "\" --input-dir \"" +
            (temp_root / "data").generic_string() + "\" --output-root \"" +
            (temp_root / "batch_out").generic_string() + "\"";
#endif
  REQUIRE(std::system(command.c_str()) == 0);

  std::filesystem::path batch_root;
  for (const auto& entry : std::filesystem::directory_iterator(temp_root / "batch_out")) {
    if (entry.is_directory()) {
      batch_root = entry.path();
      break;
    }
  }
  REQUIRE_FALSE(batch_root.empty());
  CHECK(std::filesystem::exists(batch_root / "summary.csv"));
  CHECK(std::filesystem::exists(batch_root / "summary.json"));
  CHECK(std::filesystem::exists(batch_root / "基准" / "dem" / "dem_idw.png"));
}
