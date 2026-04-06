#include "dem/DEMEngine.hpp"
#include "dem/GroundFilterEngine.hpp"
#include "dem/InputManager.hpp"
#include "dem/MainController.hpp"
#include "dem/SpatialIndexManager.hpp"
#include "dem/Utils.hpp"

#include <doctest/doctest.h>

#include <filesystem>
#include <fstream>
#include <tuple>

namespace {

dem::DEMConfig makeTestConfig() {
  dem::DEMConfig config;
  config.crs.allow_unknown_crs = true;
  config.dem.cell_size = 1.0;
  config.dem.nearest_max_distance = 0.6;
  config.dem.idw_radius = 1.5;
  config.dem.idw_min_points = 3;
  config.dem.idw_max_points = 8;
  config.dem.idw_power = 2.0;
  config.dem.nodata = -9999.0;
  config.dem.edge_shrink_cells = 1.0;
  config.dem.fill_holes = true;
  config.dem.fill_max_radius = 1;
  config.dem.fill_min_neighbors = 4;
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
  const auto grid = engine.createGrid(bounds, config, false);
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

std::size_t changedValueCount(const dem::RasterGrid& before, const dem::RasterGrid& after) {
  std::size_t changed = 0;
  for (std::size_t i = 0; i < before.values.size() && i < after.values.size(); ++i) {
    if (before.values[i] != after.values[i]) {
      ++changed;
    }
  }
  return changed;
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
  CHECK(std::filesystem::exists(run_dir / "dem" / "dem_raw_direct.png"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dem_ground_direct.png"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dem_support_mask.png"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dem_mask.png"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dem_edge_mask.png"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dtm_analysis.tif"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dtm_analysis.png"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dtm_display.png"));
  CHECK(std::filesystem::exists(run_dir / "dem" / "dtm_object_mask.png"));
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
  CHECK(std::filesystem::exists(batch_root / "基准" / "dem" / "dem_raw_direct.png"));
  CHECK(std::filesystem::exists(batch_root / "基准" / "dem" / "dem_idw.png"));
  CHECK(std::filesystem::exists(batch_root / "基准" / "dem" / "dtm_analysis.png"));
  CHECK(std::filesystem::exists(batch_root / "基准" / "dem" / "dtm_object_mask.png"));
}

TEST_CASE("domain mask expands beyond direct support but does not cross large holes") {
  auto config = makeTestConfig();
  config.dem.nearest_max_distance = 1.0;
  config.dem.idw_radius = 1.5;
  config.dem.fill_max_radius = 1;
  config.dem.fill_min_neighbors = 4;

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
  auto support = makeDirectRaster(bounds, cloud, config, engine);
  auto support_mask = engine.buildSupportMask(support);
  auto domain_mask = engine.buildDomainMask(support, config, logger, stats);
  auto nearest = engine.interpolateNearest(support, domain_mask, config, logger, stats);
  auto filled = engine.fillHoles(nearest, domain_mask, config, logger, stats);

  CHECK(nearest.values[nearest.offset(2, 2)] == doctest::Approx(config.dem.nodata));
  CHECK(filled.values[filled.offset(2, 2)] == doctest::Approx(config.dem.nodata));
  CHECK(filled.values[filled.offset(1, 1)] != doctest::Approx(config.dem.nodata));
  CHECK(support_mask.values[support_mask.offset(1, 1)] == doctest::Approx(0.0));
  CHECK(domain_mask.values[domain_mask.offset(1, 1)] == doctest::Approx(1.0));
  CHECK(domain_mask.values[domain_mask.offset(2, 2)] == doctest::Approx(0.0));
  CHECK(activeCount(support_mask) < activeCount(domain_mask));
}

TEST_CASE("edge mask only marks structural outer boundary when no fill frontier exists") {
  auto config = makeTestConfig();
  std::vector<dem::Point3D> points;
  for (int row = 0; row < 5; ++row) {
    for (int col = 0; col < 5; ++col) {
      dem::Point3D point;
      point.x = col + 0.5;
      point.y = 4.5 - row;
      point.z = 0.0;
      points.push_back(point);
    }
  }

  auto cloud = makeCloud(points);
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::DEMEngine engine;
  const dem::Bounds bounds{0.0, 5.0, 0.0, 5.0, 0.0, 1.0};
  auto support = makeDirectRaster(bounds, cloud, config, engine);
  auto domain = engine.buildDomainMask(support, config, logger, stats);
  auto nearest = engine.interpolateNearest(support, domain, config, logger, stats);
  auto edge = engine.buildEdgeMask(domain, nearest, nearest, nearest, nearest, config, logger, stats);

  CHECK(activeCount(edge) == 16);
  CHECK(edge.values[edge.offset(2, 2)] == doctest::Approx(0.0));
  CHECK(edge.values[edge.offset(0, 0)] == doctest::Approx(1.0));
}

TEST_CASE("idw interpolation honors radius and minimum neighbor constraints") {
  auto config = makeTestConfig();
  config.dem.idw_radius = 0.75;
  config.dem.idw_min_points = 2;

  std::vector<dem::Point3D> points;
  for (const auto [x, y, z] : std::vector<std::tuple<double, double, double>>{
           {0.5, 2.5, 10.0},
           {2.5, 2.5, 20.0},
       }) {
    dem::Point3D point;
    point.x = x;
    point.y = y;
    point.z = z;
    points.push_back(point);
  }

  auto cloud = makeCloud(points);
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::DEMEngine engine;
  const dem::Bounds bounds{0.0, 3.0, 0.0, 3.0, 0.0, 20.0};
  auto support = makeDirectRaster(bounds, cloud, config, engine);
  auto domain = engine.buildSupportMask(support);
  domain.values[domain.offset(0, 1)] = 1.0;
  domain.values[domain.offset(1, 1)] = 1.0;
  auto idw = engine.interpolateIdw(support, domain, config, logger, stats);

  CHECK(idw.values[idw.offset(1, 1)] == doctest::Approx(config.dem.nodata));
  CHECK(idw.values[idw.offset(0, 0)] == doctest::Approx(10.0));
  CHECK(idw.values[idw.offset(0, 1)] == doctest::Approx(config.dem.nodata));
}

TEST_CASE("ground filter keeps roof points out of ground direct raster") {
  auto config = makeTestConfig();
  config.ground.max_height_diff = 0.5;

  std::vector<dem::Point3D> points;
  for (int row = 0; row < 5; ++row) {
    for (int col = 0; col < 5; ++col) {
      dem::Point3D point;
      point.x = col + 0.5;
      point.y = 4.5 - row;
      point.z = (row >= 1 && row <= 2 && col >= 1 && col <= 2) ? 3.0 : 0.0;
      points.push_back(point);
    }
  }

  auto filtered = makeCloud(points);
  dem::PointCloud seeds;
  dem::PointCloud ground;
  dem::PointCloud nonground;
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::GroundFilterEngine engine;
  dem::DEMEngine dem_engine;

  engine.extractGround(filtered, seeds, ground, nonground, config, logger, stats);
  const dem::Bounds bounds{0.0, 5.0, 0.0, 5.0, 0.0, 3.0};
  auto raw_direct = makeDirectRaster(bounds, makeCloud(points), config, dem_engine);
  auto ground_direct = makeDirectRaster(bounds, ground, config, dem_engine);

  CHECK(std::none_of(ground.points.begin(), ground.points.end(), [](const dem::Point3D& point) { return point.z > 1.0; }));
  CHECK(std::any_of(nonground.points.begin(), nonground.points.end(), [](const dem::Point3D& point) { return point.z > 1.0; }));
  CHECK(raw_direct.values[raw_direct.offset(1, 1)] == doctest::Approx(3.0));
  CHECK(ground_direct.values[ground_direct.offset(1, 1)] == doctest::Approx(config.dem.nodata));
}

TEST_CASE("object mask excludes roof block from final bare-earth domain and display dtm remains valid") {
  auto config = makeTestConfig();
  config.dem.idw_radius = 2.5;
  config.dem.nearest_max_distance = 2.0;

  std::vector<dem::Point3D> points;
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      dem::Point3D point;
      point.x = col + 0.5;
      point.y = 5.5 - row;
      point.z = (row >= 2 && row <= 3 && col >= 2 && col <= 3) ? 4.0 : 0.0;
      points.push_back(point);
    }
  }

  auto filtered = makeCloud(points);
  dem::PointCloud seeds;
  dem::PointCloud ground;
  dem::PointCloud nonground;
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::GroundFilterEngine ground_engine;
  dem::DEMEngine dem_engine;

  ground_engine.extractGround(filtered, seeds, ground, nonground, config, logger, stats);
  const dem::Bounds bounds{0.0, 6.0, 0.0, 6.0, 0.0, 4.0};
  auto raw_direct = makeDirectRaster(bounds, makeCloud(points), config, dem_engine);
  auto ground_direct = makeDirectRaster(bounds, ground, config, dem_engine);
  auto object_mask = dem_engine.buildObjectMask(raw_direct, ground_direct, config, logger, stats);
  auto accepted = dem_engine.refineAcceptedSupport(raw_direct, ground_direct, config, logger, stats, &object_mask);
  auto domain = dem_engine.buildDomainMask(accepted, config, logger, stats, &object_mask);
  auto analysis = dem_engine.buildAnalysisDtm(accepted, domain, object_mask, config, logger, stats);
  auto display = dem_engine.buildDisplayDtm(analysis, domain, object_mask, config, logger, stats);

  CHECK(object_mask.values[object_mask.offset(2, 2)] == doctest::Approx(1.0));
  CHECK(domain.values[domain.offset(2, 2)] == doctest::Approx(0.0));
  CHECK(analysis.values[analysis.offset(2, 2)] == doctest::Approx(config.dem.nodata));
  CHECK(display.values[display.offset(2, 2)] != doctest::Approx(config.dem.nodata));
  CHECK(validValueCount(display) > validValueCount(analysis));
  CHECK(validValueCount(display) > 0U);
}

TEST_CASE("display dtm fills enclosed holes without expanding border-connected gaps") {
  auto config = makeTestConfig();
  dem::Logger logger;
  dem::ProcessStats stats;
  dem::DEMEngine engine;
  const dem::Bounds bounds{0.0, 6.0, 0.0, 6.0, 0.0, 10.0};

  auto analysis = engine.createGrid(bounds, config, false);
  auto domain = engine.createGrid(bounds, config, false);
  domain.nodata = -1.0;
  std::fill(domain.values.begin(), domain.values.end(), 0.0);
  std::fill(domain.valid_mask.begin(), domain.valid_mask.end(), 1U);
  auto object_mask = domain;

  for (std::size_t row = 0; row < domain.rows; ++row) {
    for (std::size_t col = 1; col + 1 < domain.cols; ++col) {
      domain.values[domain.offset(row, col)] = 1.0;
    }
  }

  for (std::size_t row = 0; row < analysis.rows; ++row) {
    for (std::size_t col = 1; col + 1 < analysis.cols; ++col) {
      const auto offset = analysis.offset(row, col);
      analysis.values[offset] = static_cast<double>(row + col);
      analysis.valid_mask[offset] = 1U;
    }
  }

  analysis.values[analysis.offset(2, 2)] = analysis.nodata;
  analysis.valid_mask[analysis.offset(2, 2)] = 0U;
  domain.values[domain.offset(2, 2)] = 1.0;

  domain.values[domain.offset(3, 0)] = 0.0;
  analysis.values[analysis.offset(3, 1)] = analysis.nodata;
  analysis.valid_mask[analysis.offset(3, 1)] = 0U;
  domain.values[domain.offset(3, 1)] = 0.0;

  auto display = engine.buildDisplayDtm(analysis, domain, object_mask, config, logger, stats);

  CHECK(display.values[display.offset(2, 2)] != doctest::Approx(config.dem.nodata));
  CHECK(display.values[display.offset(3, 1)] == doctest::Approx(config.dem.nodata));
}
