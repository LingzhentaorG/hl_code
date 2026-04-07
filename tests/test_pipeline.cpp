#include "dem/DEMEngine.hpp"
#include "dem/InputManager.hpp"
#include "dem/MainController.hpp"
#include "dem/SpatialIndexManager.hpp"

#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

dem::DEMConfig makeTestConfig() {
  dem::DEMConfig config;
  config.crs.allow_unknown_crs = true;
  config.dem.cell_size = 1.0;
  config.dem.idw_k = 8;
  config.dem.idw_min_points = 3;
  config.dem.idw_max_distance = 0.0;
  config.dem.idw_allow_fallback = true;
  config.dem.idw_power = 2.0;
  config.dem.nodata = -9999.0;
  config.dem.boundary_outline_cells = 1.0;
  config.ground.seed_grid_size = 1.0;
  config.ground.search_radius = 1.5;
  config.ground.knn = 8;
  config.ground.max_height_diff = 0.6;
  config.ground.max_slope_deg = 25.0;
  config.ground.max_iterations = 4;
  config.ground.min_ground_neighbors = 3;
  config.ground.min_neighbor_completeness = 0.5;
  config.output.directory = std::filesystem::temp_directory_path() / "dem_test_output";
  return config;
}

dem::PointCloud makeCloud(const std::vector<dem::Point3D>& points) {
  dem::PointCloud cloud;
  cloud.points = points;
  cloud.raw_point_count = points.size();
  cloud.stored_point_count = points.size();
  for (const auto& point : points) {
    cloud.bounds.expand(point.x, point.y, point.z);
  }
  return cloud;
}

dem::RasterGrid makeDirectRaster(const dem::Bounds& bounds,
                                 const dem::PointCloud& cloud,
                                 const dem::DEMConfig& config,
                                 dem::DEMEngine& engine) {
  const auto grid = engine.createGrid(bounds, config);
  return engine.rasterizeMinimum(cloud, grid);
}

std::size_t activeCount(const dem::RasterGrid& grid) {
  return std::count_if(grid.values.begin(), grid.values.end(), [](double value) { return std::isfinite(value) && value > 0.5; });
}

std::size_t validValueCount(const dem::RasterGrid& grid) {
  return std::count_if(grid.values.begin(), grid.values.end(), [&](double value) {
    return std::isfinite(value) && value != grid.nodata;
  });
}

dem::RasterGrid makeFullMaskLike(const dem::RasterGrid& grid) {
  dem::RasterGrid mask = grid;
  std::fill(mask.values.begin(), mask.values.end(), 1.0);
  mask.nodata = 0.0;
  mask.valid_mask.assign(mask.values.size(), true);
  return mask;
}

}  // namespace

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

TEST_CASE("controller writes only official DEM outputs by default") {
  const auto source_dir = std::filesystem::path(DEM_SOURCE_DIR);
  const auto sample_path = source_dir / "data" / "基准.ply";
  REQUIRE(std::filesystem::exists(sample_path));

  const auto temp_root = std::filesystem::temp_directory_path() / "dem_pipeline_test";
  std::filesystem::remove_all(temp_root);
  std::filesystem::create_directories(temp_root);

  const auto config_path = temp_root / "pipeline_config.json";
  std::ofstream out(config_path);
  out << "{\n"
         "  \"input\": { \"file_path\": \"" << sample_path.generic_string() << "\" },\n"
         "  \"crs\": { \"epsg_code\": 4326 },\n"
         "  \"dem\": {\n"
         "    \"cell_size\": 8.0,\n"
         "    \"idw_k\": 12,\n"
         "    \"idw_min_points\": 3,\n"
         "    \"idw_max_distance\": 0.0,\n"
         "    \"idw_allow_fallback\": true,\n"
         "    \"idw_power\": 2.0,\n"
         "    \"boundary_outline_cells\": 1.0\n"
         "  },\n"
         "  \"tile\": { \"mode\": \"disabled\", \"tile_size_cells\": 1000, \"memory_limit_mb\": 512 },\n"
         "  \"output\": {\n"
      << "    \"directory\": \"" << (temp_root / "out").generic_string() << "\",\n"
         "    \"write_png\": true,\n"
         "    \"timestamp_subdir\": true,\n"
         "    \"write_debug_pointclouds\": false\n"
         "  }\n"
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

  CHECK(std::filesystem::exists(run_dir / "surface" / "dem.tif"));
  CHECK(std::filesystem::exists(run_dir / "surface" / "dem.png"));
  CHECK(std::filesystem::exists(run_dir / "surface" / "dem_raw.tif"));
  CHECK(std::filesystem::exists(run_dir / "surface" / "dem_raw.png"));
  CHECK(std::filesystem::exists(run_dir / "mask" / "dem_domain_mask.tif"));
  CHECK(std::filesystem::exists(run_dir / "mask" / "dem_domain_mask.png"));
  CHECK(std::filesystem::exists(run_dir / "mask" / "dem_boundary_mask.tif"));
  CHECK(std::filesystem::exists(run_dir / "mask" / "dem_boundary_mask.png"));
  CHECK(std::filesystem::exists(run_dir / "mask" / "dem_object_mask.tif"));
  CHECK(std::filesystem::exists(run_dir / "mask" / "dem_object_mask.png"));
  CHECK(std::filesystem::exists(run_dir / "log" / "config_used.json"));
  CHECK(std::filesystem::exists(run_dir / "log" / "stats.txt"));
  CHECK(std::filesystem::exists(run_dir / "log" / "report.json"));

  CHECK_FALSE(std::filesystem::exists(run_dir / "pointcloud"));
  CHECK_FALSE(std::filesystem::exists(run_dir / "dem"));
  CHECK_FALSE(std::filesystem::exists(run_dir / "surface" / "dtm_analysis.tif"));
  CHECK_FALSE(std::filesystem::exists(run_dir / "surface" / "dtm_display.tif"));
  CHECK_FALSE(std::filesystem::exists(run_dir / "surface" / "dem_idw.tif"));
  CHECK_FALSE(std::filesystem::exists(run_dir / "mask" / "dem_mask.png"));
  CHECK_FALSE(std::filesystem::exists(run_dir / "mask" / "dem_edge_mask.png"));
  CHECK_FALSE(std::filesystem::exists(run_dir / "mask" / "dtm_object_mask.png"));
}

TEST_CASE("batch processing script writes datasets subtree without configs directory") {
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
         "  \"dem\": {\n"
         "    \"cell_size\": 8.0,\n"
         "    \"idw_k\": 12,\n"
         "    \"idw_min_points\": 3,\n"
         "    \"idw_max_distance\": 0.0,\n"
         "    \"idw_allow_fallback\": true,\n"
         "    \"idw_power\": 2.0,\n"
         "    \"boundary_outline_cells\": 1.0\n"
         "  },\n"
         "  \"tile\": { \"mode\": \"disabled\", \"tile_size_cells\": 1000, \"memory_limit_mb\": 512 },\n"
         "  \"output\": {\n"
      << "    \"directory\": \"" << (temp_root / "out").generic_string() << "\",\n"
         "    \"write_png\": true,\n"
         "    \"timestamp_subdir\": false,\n"
         "    \"write_debug_pointclouds\": false\n"
         "  }\n"
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
  CHECK(std::filesystem::exists(batch_root / "datasets" / "基准" / "surface" / "dem.png"));
  CHECK(std::filesystem::exists(batch_root / "datasets" / "基准" / "surface" / "dem_raw.png"));
  CHECK(std::filesystem::exists(batch_root / "datasets" / "基准" / "mask" / "dem_domain_mask.png"));
  CHECK(std::filesystem::exists(batch_root / "datasets" / "基准" / "mask" / "dem_boundary_mask.png"));
  CHECK(std::filesystem::exists(batch_root / "datasets" / "基准" / "mask" / "dem_object_mask.png"));
  CHECK(std::filesystem::exists(batch_root / "datasets" / "基准" / "log" / "stats.txt"));
  CHECK(std::filesystem::exists(batch_root / "datasets" / "基准" / "log" / "config_used.json"));
  CHECK_FALSE(std::filesystem::exists(batch_root / "configs"));
  CHECK_FALSE(std::filesystem::exists(batch_root / "基准"));
  CHECK_FALSE(std::filesystem::exists(batch_root / "datasets" / "基准" / "dem"));
}

TEST_CASE("domain mask fills convex hull interior without internal holes") {
  auto config = makeTestConfig();

  std::vector<dem::Point3D> points;
  for (int row = 0; row < 5; ++row) {
    for (int col = 0; col < 5; ++col) {
      if (row >= 1 && row <= 3 && col >= 1 && col <= 3) {
        continue;
      }
      dem::Point3D point;
      point.x = col + 0.5;
      point.y = 4.5 - row;
      point.z = static_cast<double>(row + col);
      points.push_back(point);
    }
  }

  auto cloud = makeCloud(points);
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::DEMEngine engine;
  const dem::Bounds bounds{0.0, 5.0, 0.0, 5.0, 0.0, 10.0};
  const auto support = makeDirectRaster(bounds, cloud, config, engine);
  const auto support_mask = engine.buildSupportMask(support);
  const auto grid_template = engine.createGrid(bounds, config);
  const auto domain_mask = engine.buildDomainMask(cloud, grid_template, config, logger, stats);

  CHECK(support_mask.values[support_mask.offset(1, 1)] == doctest::Approx(0.0));
  CHECK(domain_mask.values[domain_mask.offset(1, 1)] == doctest::Approx(1.0));
  CHECK(domain_mask.values[domain_mask.offset(2, 2)] == doctest::Approx(1.0));
  CHECK(activeCount(domain_mask) == 25U);
  CHECK(activeCount(support_mask) < activeCount(domain_mask));
}

TEST_CASE("boundary mask is a hull outline with configurable thickness") {
  auto config = makeTestConfig();

  std::vector<dem::Point3D> points;
  for (int row = 0; row < 9; ++row) {
    for (int col = 0; col < 9; ++col) {
      dem::Point3D point;
      point.x = col + 0.5;
      point.y = 8.5 - row;
      point.z = static_cast<double>(row + col);
      points.push_back(point);
    }
  }

  auto cloud = makeCloud(points);
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::DEMEngine engine;
  const dem::Bounds bounds{0.0, 9.0, 0.0, 9.0, 0.0, 20.0};
  const auto grid_template = engine.createGrid(bounds, config);

  config.dem.boundary_outline_cells = 1.0;
  const auto outline1 = engine.buildEdgeMask(cloud, grid_template, config, logger, stats);

  config.dem.boundary_outline_cells = 2.0;
  const auto outline2 = engine.buildEdgeMask(cloud, grid_template, config, logger, stats);

  CHECK(outline1.values[outline1.offset(4, 4)] == doctest::Approx(0.0));
  CHECK(outline2.values[outline2.offset(4, 4)] == doctest::Approx(0.0));
  CHECK(activeCount(outline2) > activeCount(outline1));
}

TEST_CASE("official IDW uses output diagonal when max distance is zero") {
  auto config = makeTestConfig();
  config.dem.idw_k = 4;
  config.dem.idw_min_points = 3;
  config.dem.idw_max_distance = 0.0;
  config.dem.idw_allow_fallback = true;

  std::vector<dem::Point3D> support_points;
  for (const auto& xy : std::vector<std::pair<double, double>>{{0.5, 0.5}, {4.5, 0.5}, {0.5, 4.5}, {4.5, 4.5}}) {
    dem::Point3D point;
    point.x = xy.first;
    point.y = xy.second;
    point.z = 10.0 + xy.first + xy.second;
    point.ground = true;
    support_points.push_back(point);
  }

  auto cloud = makeCloud(support_points);
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::DEMEngine engine;
  const dem::Bounds bounds{0.0, 5.0, 0.0, 5.0, 0.0, 20.0};
  const auto support = makeDirectRaster(bounds, cloud, config, engine);
  const auto grid_template = engine.createGrid(bounds, config);
  const auto domain = engine.buildDomainMask(cloud, grid_template, config, logger, stats);
  const auto dem = engine.interpolateIdw(support, domain, config, logger, stats);

  CHECK(validValueCount(dem) == 25U);
  CHECK(dem.values[dem.offset(2, 2)] != doctest::Approx(dem.nodata));
}

TEST_CASE("official IDW fallback controls whether sparse holes are interpolated") {
  auto config = makeTestConfig();
  config.dem.idw_k = 2;
  config.dem.idw_min_points = 3;
  config.dem.idw_max_distance = 0.0;

  std::vector<dem::Point3D> support_points;
  for (const auto& xy : std::vector<std::pair<double, double>>{{0.5, 0.5}, {4.5, 4.5}}) {
    dem::Point3D point;
    point.x = xy.first;
    point.y = xy.second;
    point.z = 10.0 + xy.first + xy.second;
    point.ground = true;
    support_points.push_back(point);
  }

  auto cloud = makeCloud(support_points);
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::DEMEngine engine;
  const dem::Bounds bounds{0.0, 5.0, 0.0, 5.0, 0.0, 20.0};
  const auto support = makeDirectRaster(bounds, cloud, config, engine);
  auto domain = makeFullMaskLike(support);

  config.dem.idw_allow_fallback = false;
  const auto no_fallback = engine.interpolateIdw(support, domain, config, logger, stats);
  CHECK(no_fallback.values[no_fallback.offset(2, 2)] == doctest::Approx(no_fallback.nodata));

  config.dem.idw_allow_fallback = true;
  const auto with_fallback = engine.interpolateIdw(support, domain, config, logger, stats);
  CHECK(with_fallback.values[with_fallback.offset(2, 2)] != doctest::Approx(with_fallback.nodata));
}

TEST_CASE("object mask does not punch holes in the official domain or DEM") {
  auto config = makeTestConfig();

  std::vector<dem::Point3D> boundary_points;
  for (int row = 0; row < 5; ++row) {
    for (int col = 0; col < 5; ++col) {
      dem::Point3D point;
      point.x = col + 0.5;
      point.y = 4.5 - row;
      point.z = static_cast<double>(row + col);
      point.ground = true;
      boundary_points.push_back(point);
    }
  }

  auto boundary_cloud = makeCloud(boundary_points);
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::DEMEngine engine;
  const dem::Bounds bounds{0.0, 5.0, 0.0, 5.0, 0.0, 10.0};
  const auto grid_template = engine.createGrid(bounds, config);
  const auto domain = engine.buildDomainMask(boundary_cloud, grid_template, config, logger, stats);

  dem::RasterGrid object_mask = grid_template;
  object_mask.nodata = 0.0;
  std::fill(object_mask.values.begin(), object_mask.values.end(), 0.0);
  object_mask.valid_mask.assign(object_mask.values.size(), true);
  object_mask.values[object_mask.offset(2, 2)] = 1.0;

  dem::RasterGrid support = grid_template;
  std::fill(support.values.begin(), support.values.end(), support.nodata);
  support.valid_mask.assign(support.values.size(), false);
  for (std::size_t row = 0; row < support.rows; ++row) {
    for (std::size_t col = 0; col < support.cols; ++col) {
      if (row == 2 && col == 2) {
        continue;
      }
      const auto offset = support.offset(row, col);
      support.values[offset] = static_cast<double>(row + col);
      support.valid_mask[offset] = true;
    }
  }

  const auto dem = engine.interpolateIdw(support, domain, config, logger, stats);

  CHECK(object_mask.values[object_mask.offset(2, 2)] == doctest::Approx(1.0));
  CHECK(domain.values[domain.offset(2, 2)] == doctest::Approx(1.0));
  CHECK(dem.values[dem.offset(2, 2)] != doctest::Approx(dem.nodata));
}
