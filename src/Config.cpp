/**
 * @file Config.cpp
 * @brief 配置管理器实现：JSON 配置文件的加载、解析、校验与回写
 */

#include "dem/Config.hpp"

#include "dem/Utils.hpp"

#include <fstream>
#include <stdexcept>

namespace dem {
namespace {

/**
 * @brief 从 JSON 节点中安全读取指定键值，不存在时返回默认值
 * @tparam T 目标类型（需支持 nlohmann::json 的 get<T> 方法）
 * @param root JSON 根节点
 * @param key 键名
 * @param fallback 默认值
 * @return 解析后的值或默认值
 */
template <typename T>
T readOr(const nlohmann::json& root, const char* key, const T& fallback) {
  return root.contains(key) ? root.at(key).get<T>() : fallback;
}

/**
 * @brief 将字符串解析为参考模式枚举值
 * 支持的格式: "local_plane", "inverse_distance_squared"/"idw2", "local_min"
 */
ReferenceMode parseReferenceMode(const std::string& value) {
  const auto normalized = toLower(value);
  if (normalized == "local_plane") {
    return ReferenceMode::LocalPlane;
  }
  if (normalized == "inverse_distance_squared" || normalized == "idw2") {
    return ReferenceMode::InverseDistanceSquared;
  }
  if (normalized == "local_min") {
    return ReferenceMode::LocalMin;
  }
  throw std::runtime_error("Unsupported reference mode: " + value);
}

/**
 * @brief 将字符串解析为 Tile 分块模式枚举值
 * 支持的格式: "auto", "disabled"/"false", "forced"/"true"
 */
TileMode parseTileMode(const std::string& value) {
  const auto normalized = toLower(value);
  if (normalized == "auto") {
    return TileMode::Auto;
  }
  if (normalized == "disabled" || normalized == "false") {
    return TileMode::Disabled;
  }
  if (normalized == "forced" || normalized == "true") {
    return TileMode::Forced;
  }
  throw std::runtime_error("Unsupported tile mode: " + value);
}

/**
 * @brief 将命令行覆盖参数的原始值解析为合适的 JSON 类型
 * 自动识别布尔值、数值和纯字符串
 */
nlohmann::json parseOverrideValue(const std::string& raw_value) {
  const auto trimmed = trim(raw_value);
  /* 布尔值直接识别 */
  if (trimmed == "true" || trimmed == "false") {
    return trimmed == "true";
  }
  /* 尝试作为 JSON 数值/对象/数组解析 */
  const auto parsed = nlohmann::json::parse(trimmed, nullptr, false);
  if (!parsed.is_discarded()) {
    return parsed;
  }
  /* 回退为纯字符串 */
  return trimmed;
}

}  // namespace

/**
 * @brief 从指定路径加载配置文件，应用运行时覆盖参数，并完成归一化和校验
 *
 * 处理流程：
 * 1. 打开并解析 JSON 文件
 * 2. 应用 --set 命令行覆盖参数（支持点分隔路径如 ground.knn=10）
 * 3. 将各模块配置从 JSON 映射到 DEMConfig 结构体
 * 4. 归一化派生值（自动计算 seed_grid_size、search_radius 等）
 * 5. 验证关键参数合法性
 */
DEMConfig ConfigManager::load(const std::filesystem::path& path, const std::vector<std::string>& overrides) const {
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("Unable to open config file: " + path.string());
  }

  nlohmann::json root;
  stream >> root;
  applyOverrides(root, overrides);
  DEMConfig config = parse(root);
  normalizeDerivedValues(config);
  validate(config);
  return config;
}

/**
 * @brief 将解析后的完整配置以 JSON 格式写入文件，用于记录实际使用的参数
 *
 * 输出文件通常命名为 config_used.txt，存放在 log 子目录下，
 * 方便用户复现和审计每次运行的实际参数。
 */
void ConfigManager::writeResolvedConfig(const DEMConfig& config, const std::filesystem::path& output_path) const {
  nlohmann::json root;
  root["input"]["file_path"] = config.input.file_path.string();
  root["crs"]["epsg_code"] = config.crs.epsg_code;
  root["crs"]["crs_wkt"] = config.crs.crs_wkt;
  root["crs"]["crs_wkt_file"] = config.crs.crs_wkt_file.string();
  root["crs"]["allow_unknown_crs"] = config.crs.allow_unknown_crs;
  root["preprocess"]["sample_spacing_count"] = config.preprocess.sample_spacing_count;
  root["preprocess"]["max_extreme_sample"] = config.preprocess.max_extreme_sample;
  root["outlier"]["knn"] = config.outlier.knn;
  root["outlier"]["stddev_multiplier"] = config.outlier.stddev_multiplier;
  root["outlier"]["use_robust_stat"] = config.outlier.use_robust_stat;
  root["ground"]["seed_grid_size"] = config.ground.seed_grid_size;
  root["ground"]["knn"] = config.ground.knn;
  root["ground"]["search_radius"] = config.ground.search_radius;
  root["ground"]["max_height_diff"] = config.ground.max_height_diff;
  root["ground"]["max_slope_deg"] = config.ground.max_slope_deg;
  root["ground"]["max_iterations"] = config.ground.max_iterations;
  root["ground"]["min_ground_neighbors"] = config.ground.min_ground_neighbors;
  root["ground"]["reference_mode"] =
      config.ground.reference_mode == ReferenceMode::LocalPlane
          ? "local_plane"
          : (config.ground.reference_mode == ReferenceMode::InverseDistanceSquared ? "inverse_distance_squared"
                                                                                   : "local_min");
  root["ground"]["reference_fallback_mode"] =
      config.ground.reference_fallback_mode == ReferenceMode::LocalPlane
          ? "local_plane"
          : (config.ground.reference_fallback_mode == ReferenceMode::InverseDistanceSquared
                 ? "inverse_distance_squared"
                 : "local_min");
  root["ground"]["distance_weight_power"] = config.ground.distance_weight_power;
  root["ground"]["min_neighbor_completeness"] = config.ground.min_neighbor_completeness;
  root["dem"]["cell_size"] = config.dem.cell_size;
  root["dem"]["nearest_max_distance"] = config.dem.nearest_max_distance;
  root["dem"]["idw_radius"] = config.dem.idw_radius;
  root["dem"]["idw_max_points"] = config.dem.idw_max_points;
  root["dem"]["idw_min_points"] = config.dem.idw_min_points;
  root["dem"]["idw_power"] = config.dem.idw_power;
  root["dem"]["nodata"] = config.dem.nodata;
  root["dem"]["edge_shrink_cells"] = config.dem.edge_shrink_cells;
  root["dem"]["enable_edge_mask"] = config.dem.enable_edge_mask;
  root["dem"]["fill_holes"] = config.dem.fill_holes;
  root["dem"]["fill_max_radius"] = config.dem.fill_max_radius;
  root["dem"]["fill_min_neighbors"] = config.dem.fill_min_neighbors;
  root["tile"]["mode"] =
      config.tile.mode == TileMode::Auto ? "auto" : (config.tile.mode == TileMode::Disabled ? "disabled" : "forced");
  root["tile"]["tile_size_cells"] = config.tile.tile_size_cells;
  root["tile"]["tile_buffer"] = config.tile.tile_buffer;
  root["tile"]["memory_limit_mb"] = config.tile.memory_limit_mb;
  root["output"]["directory"] = config.output.directory.string();
  root["output"]["overwrite"] = config.output.overwrite;
  root["output"]["write_png"] = config.output.write_png;
  root["output"]["timestamp_subdir"] = config.output.timestamp_subdir;

  ensureDirectory(output_path.parent_path());
  std::ofstream out(output_path);
  out << root.dump(2);
}

/**
 * @brief 将命令行 --set 覆盖参数应用到已加载的 JSON 根节点上
 *
 * 支持点分隔路径语法（如 "ground.knn=10"），
 * 会自动创建中间节点来设置深层嵌套的值。
 */
void ConfigManager::applyOverrides(nlohmann::json& root, const std::vector<std::string>& overrides) const {
  for (const auto& item : overrides) {
    /* 解析 key=value 格式 */
    const auto pos = item.find('=');
    if (pos == std::string::npos || pos == 0) {
      throw std::runtime_error("Invalid override syntax: " + item);
    }
    const auto path = item.substr(0, pos);
    const auto value = parseOverrideValue(item.substr(pos + 1));

    /* 沿着点分隔路径逐级导航到目标位置 */
    nlohmann::json* cursor = &root;
    std::size_t start = 0;
    while (true) {
      const auto dot = path.find('.', start);
      const auto part = path.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
      if (part.empty()) {
        throw std::runtime_error("Invalid override path: " + item);
      }
      if (dot == std::string::npos) {
        (*cursor)[part] = value;
        break;
      }
      cursor = &(*cursor)[part];
      start = dot + 1;
    }
  }
}

/**
 * @brief 将 JSON 根节点完整映射到 DEMConfig 结构体
 *
 * 对每个可选配置段进行按需解析，未提供的字段使用合理的默认值。
 * 必填字段（如 input.file_path）缺失时会抛出异常。
 */
DEMConfig ConfigManager::parse(const nlohmann::json& root) const {
  DEMConfig config;

  /* 输入段 - 必须包含 file_path */
  if (!root.contains("input") || !root.at("input").contains("file_path")) {
    throw std::runtime_error("Config must define input.file_path.");
  }

  const auto& input = root.at("input");
  config.input.file_path = input.at("file_path").get<std::string>();

  /* 坐标参考系段 */
  if (root.contains("crs")) {
    const auto& crs = root.at("crs");
    config.crs.epsg_code = readOr<int>(crs, "epsg_code", 0);
    config.crs.crs_wkt = readOr<std::string>(crs, "crs_wkt", "");
    if (crs.contains("crs_wkt_file")) {
      config.crs.crs_wkt_file = crs.at("crs_wkt_file").get<std::string>();
    }
    config.crs.allow_unknown_crs = readOr<bool>(crs, "allow_unknown_crs", false);
  }

  /* 预处理段 */
  if (root.contains("preprocess")) {
    const auto& preprocess = root.at("preprocess");
    config.preprocess.sample_spacing_count = readOr<std::size_t>(preprocess, "sample_spacing_count", 1024);
    config.preprocess.max_extreme_sample = readOr<std::size_t>(preprocess, "max_extreme_sample", 50000);
  }

  /* 离群点剔除段 */
  if (root.contains("outlier")) {
    const auto& outlier = root.at("outlier");
    config.outlier.knn = readOr<std::size_t>(outlier, "knn", 16);
    config.outlier.stddev_multiplier = readOr<double>(outlier, "stddev_multiplier", 1.5);
    config.outlier.use_robust_stat = readOr<bool>(outlier, "use_robust_stat", false);
  }

  /* 地面滤波段 */
  if (root.contains("ground")) {
    const auto& ground = root.at("ground");
    config.ground.seed_grid_size = readOr<double>(ground, "seed_grid_size", 0.0);
    config.ground.knn = readOr<std::size_t>(ground, "knn", 8);
    config.ground.search_radius = readOr<double>(ground, "search_radius", 0.0);
    config.ground.max_height_diff = readOr<double>(ground, "max_height_diff", 1.0);
    config.ground.max_slope_deg = readOr<double>(ground, "max_slope_deg", 20.0);
    config.ground.max_iterations = readOr<std::size_t>(ground, "max_iterations", 6);
    config.ground.min_ground_neighbors = readOr<std::size_t>(ground, "min_ground_neighbors", 3);
    config.ground.reference_mode =
        parseReferenceMode(readOr<std::string>(ground, "reference_mode", "local_plane"));
    config.ground.reference_fallback_mode =
        parseReferenceMode(readOr<std::string>(ground, "reference_fallback_mode", "inverse_distance_squared"));
    config.ground.distance_weight_power = readOr<double>(ground, "distance_weight_power", 2.0);
    config.ground.min_neighbor_completeness = readOr<double>(ground, "min_neighbor_completeness", 0.5);
  }

  /* DEM 生成段 */
  if (root.contains("dem")) {
    const auto& dem = root.at("dem");
    config.dem.cell_size = readOr<double>(dem, "cell_size", 1.0);
    config.dem.nearest_max_distance = readOr<double>(dem, "nearest_max_distance", 0.0);
    config.dem.idw_radius = readOr<double>(dem, "idw_radius", 0.0);
    config.dem.idw_max_points = readOr<std::size_t>(dem, "idw_max_points", 12);
    config.dem.idw_min_points = readOr<std::size_t>(dem, "idw_min_points", 3);
    config.dem.idw_power = readOr<double>(dem, "idw_power", 2.0);
    config.dem.nodata = readOr<double>(dem, "nodata", -9999.0);
    config.dem.edge_shrink_cells = readOr<double>(dem, "edge_shrink_cells", 1.0);
    config.dem.enable_edge_mask = readOr<bool>(dem, "enable_edge_mask", true);
    config.dem.fill_holes = readOr<bool>(dem, "fill_holes", true);
    config.dem.fill_max_radius = readOr<int>(dem, "fill_max_radius", 2);
    config.dem.fill_min_neighbors = readOr<std::size_t>(dem, "fill_min_neighbors", 4);
  }

  /* Tile 分块段 */
  if (root.contains("tile")) {
    const auto& tile = root.at("tile");
    config.tile.mode = parseTileMode(readOr<std::string>(tile, "mode", "auto"));
    config.tile.tile_size_cells = readOr<std::size_t>(tile, "tile_size_cells", 1000);
    config.tile.tile_buffer = readOr<double>(tile, "tile_buffer", 0.0);
    config.tile.memory_limit_mb = readOr<std::size_t>(tile, "memory_limit_mb", 1024);
  }

  /* 输出段 */
  if (root.contains("output")) {
    const auto& output = root.at("output");
    if (output.contains("directory")) {
      config.output.directory = output.at("directory").get<std::string>();
    } else {
      config.output.directory = "output";
    }
    config.output.overwrite = readOr<bool>(output, "overwrite", true);
    config.output.write_png = readOr<bool>(output, "write_png", true);
    config.output.timestamp_subdir = readOr<bool>(output, "timestamp_subdir", true);
  } else {
    config.output.directory = "output";
    config.output.write_png = true;
    config.output.timestamp_subdir = true;
  }

  return config;
}

/**
 * @brief 归一化派生参数：将值为 0（表示"自动计算"）的字段替换为基于 cell_size 的合理默认值
 *
 * 自动计算的规则：
 * - seed_grid_size → cell_size × 3
 * - search_radius → max(cell_size×6, seed_grid_size×2)
 * - nearest_max_distance → max(cell_size×6, search_radius)
 * - idw_radius → max(cell_size×8, search_radius + cell_size×2)
 * - tile_buffer → max(search_radius, nearest_max_distance, idw_radius) + fill_max_radius×cell_size
 */
void ConfigManager::normalizeDerivedValues(DEMConfig& config) const {
  if (config.ground.seed_grid_size <= 0.0) {
    config.ground.seed_grid_size = 3.0 * config.dem.cell_size;
  }
  if (config.ground.search_radius <= 0.0) {
    config.ground.search_radius = std::max(6.0 * config.dem.cell_size, 2.0 * config.ground.seed_grid_size);
  }
  if (config.dem.nearest_max_distance <= 0.0) {
    config.dem.nearest_max_distance = std::max(6.0 * config.dem.cell_size, config.ground.search_radius);
  }
  if (config.dem.idw_radius <= 0.0) {
    config.dem.idw_radius = std::max(8.0 * config.dem.cell_size, config.ground.search_radius + 2.0 * config.dem.cell_size);
  }
  if (config.tile.tile_buffer <= 0.0) {
    config.tile.tile_buffer = std::max({config.ground.search_radius, config.dem.nearest_max_distance, config.dem.idw_radius}) +
                              static_cast<double>(std::max(0, config.dem.fill_max_radius)) * config.dem.cell_size;
  }
}

/**
 * @brief 对解析后的配置进行合法性校验，不满足条件时抛出异常
 *
 * 校验项包括：
 * - 必填非空字段（file_path, directory）
 * - 正数约束（cell_size, idw_power）
 * - 最小值约束（knn, max_iterations 等）
 * - CRS 可用性检查
 */
void ConfigManager::validate(const DEMConfig& config) const {
  if (config.input.file_path.empty()) {
    throw std::runtime_error("input.file_path must not be empty.");
  }
  if (config.dem.cell_size <= 0.0) {
    throw std::runtime_error("dem.cell_size must be > 0.");
  }
  if (config.dem.idw_power <= 0.0) {
    throw std::runtime_error("dem.idw_power must be > 0.");
  }
  if (config.outlier.knn < 1) {
    throw std::runtime_error("outlier.knn must be >= 1.");
  }
  if (config.ground.knn < 1) {
    throw std::runtime_error("ground.knn must be >= 1.");
  }
  if (config.ground.max_iterations < 1) {
    throw std::runtime_error("ground.max_iterations must be >= 1.");
  }
  if (config.dem.fill_max_radius < 0) {
    throw std::runtime_error("dem.fill_max_radius must be >= 0.");
  }
  if (config.tile.tile_size_cells < 1) {
    throw std::runtime_error("tile.tile_size_cells must be >= 1.");
  }
  if (config.tile.tile_buffer < 0.0) {
    throw std::runtime_error("tile.tile_buffer must be >= 0.");
  }
  if (config.output.directory.empty()) {
    throw std::runtime_error("output.directory must not be empty.");
  }
  const bool has_known_crs = !config.crs.crs_wkt.empty() || !config.crs.crs_wkt_file.empty() || config.crs.epsg_code > 0;
  if (!has_known_crs && !config.crs.allow_unknown_crs) {
    throw std::runtime_error("CRS is required unless crs.allow_unknown_crs is true.");
  }
}

}  // namespace dem
