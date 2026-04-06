/**
 * @file InputManager.cpp
 * @brief 输入管理器实现：PLY 文件头解析、点云扫描、流式读取和输入验证
 */

#include "dem/InputManager.hpp"

#include "dem/Utils.hpp"

#include <bit>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace dem {
namespace {

/**
 * @brief 用于快速检测重复点的位模式哈希键
 * 使用 std::bit_cast 将浮点坐标直接转换为整数位模式，
 * 实现精确坐标匹配的高效去重。
 */
struct SamplePointKey {
  std::uint64_t x_bits = 0;   /**< X 坐标的位模式 */
  std::uint64_t y_bits = 0;   /**< Y 坐标的位模式 */
  std::uint64_t z_bits = 0;   /**< Z 坐标的位模式 */

  bool operator==(const SamplePointKey& other) const noexcept {
    return x_bits == other.x_bits && y_bits == other.y_bits && z_bits == other.z_bits;
  }
};

/** SamplePointKey 的哈希函数，使用异或混合三个坐标的位模式 */
struct SamplePointKeyHash {
  std::size_t operator()(const SamplePointKey& key) const noexcept {
    return static_cast<std::size_t>(key.x_bits ^ (key.y_bits << 1U) ^ (key.z_bits << 7U));
  }
};

/**
 * @brief 判断输入点云的坐标轴顺序是否可能存在异常
 *
 * 检测两种可疑情况：
 * 1. X/Y 中心坐标量级差异过大（可能经纬度被颠倒）
 * 2. X/Y 范围比例极度悬殊（可能坐标系定义错误）
 */
bool shouldWarnCoordinateOrder(const PointCloudScanSummary& summary) {
  if (!summary.bounds.valid()) {
    return false;
  }
  const double x_center = (summary.bounds.xmin + summary.bounds.xmax) * 0.5;
  const double y_center = (summary.bounds.ymin + summary.bounds.ymax) * 0.5;
  const double width = std::max(summary.bounds.width(), 1e-9);
  const double height = std::max(summary.bounds.height(), 1e-9);
  const bool swapped_magnitude = std::abs(x_center) > 1000000.0 && std::abs(y_center) < 100000.0;
  const bool highly_skewed_extent = width > height * 100.0 || height > width * 100.0;
  return swapped_magnitude || highly_skewed_extent;
}

}  // namespace

/**
 * @brief 仅解析 PLY 文件头信息，不读取顶点数据
 * 用于快速获取属性列表和顶点数量等元数据。
 */
PlyHeader InputManager::inspectHeader(const std::filesystem::path& file_path) const {
  std::ifstream stream(file_path);
  if (!stream) {
    throw std::runtime_error("Unable to open PLY file: " + file_path.string());
  }
  return parseHeader(stream);
}

/**
 * @brief 执行完整的输入验证流程
 *
 * 包含以下步骤：
 * 1. 解析 PLY 文件头获取元数据
 * 2. 流式扫描全部点统计 NaN/Inf/重复点情况
 * 3. 估算内存占用并判断是否需要启用 Tile 分块模式
 * 4. 检测坐标轴顺序异常
 * 5. 验证输出目录可写性
 */
InputValidationReport InputManager::validateInput(const std::filesystem::path& file_path,
                                                  const DEMConfig& config,
                                                  const std::filesystem::path& output_directory,
                                                  Logger& logger,
                                                  ProcessStats& stats) const {
  ScopedTimer timer(stats, "input_validate_seconds");
  InputValidationReport report;
  report.header = inspectHeader(file_path);
  report.summary = scanPointCloud(file_path, config.preprocess.sample_spacing_count, logger, stats);
  report.output_directory_writable = canWriteToDirectory(output_directory);

  /* 根据内存估算和配置决定是否使用 Tile 分块模式 */
  report.will_use_tile_mode =
      config.tile.mode == TileMode::Forced ||
      (config.tile.mode == TileMode::Auto &&
       report.summary.estimated_memory_mb > static_cast<double>(config.tile.memory_limit_mb));

  /* 通过位哈希检测采样点中的重复点 */
  std::unordered_set<SamplePointKey, SamplePointKeyHash> seen;
  for (const auto& point : report.summary.sample_points) {
    const SamplePointKey key{
        .x_bits = std::bit_cast<std::uint64_t>(point.x),
        .y_bits = std::bit_cast<std::uint64_t>(point.y),
        .z_bits = std::bit_cast<std::uint64_t>(point.z),
    };
    if (!seen.insert(key).second) {
      report.summary.duplicate_hint_count += 1;
    }
  }

  /* 收集各类警告信息 */
  report.coordinate_order_suspect = shouldWarnCoordinateOrder(report.summary);
  if (report.coordinate_order_suspect) {
    report.warnings.push_back("Coordinate order may be suspicious; verify x/y axis orientation.");
  }
  if (report.summary.nan_points > 0 || report.summary.inf_points > 0) {
    report.warnings.push_back("Input contains NaN/Inf points and will be cleaned during preprocessing.");
  }
  if (report.summary.duplicate_hint_count > 0) {
    report.warnings.push_back("Sampled input contains duplicate points; duplicates will be removed during preprocessing.");
  }
  if (!report.output_directory_writable) {
    throw std::runtime_error("Output directory is not writable: " + output_directory.string());
  }

  /* 记录扫描结果到统计信息 */
  stats.setCount("scan_nan_points", report.summary.nan_points);
  stats.setCount("scan_inf_points", report.summary.inf_points);
  stats.setCount("scan_duplicate_hint_count", report.summary.duplicate_hint_count);
  stats.setValue("input_bounds_xmin", report.summary.bounds.xmin);
  stats.setValue("input_bounds_xmax", report.summary.bounds.xmax);
  stats.setValue("input_bounds_ymin", report.summary.bounds.ymin);
  stats.setValue("input_bounds_ymax", report.summary.bounds.ymax);
  stats.setValue("input_bounds_zmin", report.summary.bounds.zmin);
  stats.setValue("input_bounds_zmax", report.summary.bounds.zmax);
  stats.setValue("estimated_memory_mb", report.summary.estimated_memory_mb);

  for (const auto& warning : report.warnings) {
    logger.warn(warning);
  }
  logger.info("Input validation completed.");
  return report;
}

/**
 * @brief 完整读取 PLY 文件中的所有顶点到内存中的 PointCloud 结构
 *
 * 同时更新空间边界信息，并在完成后记录加载统计。
 */
PointCloud InputManager::readPointCloud(const std::filesystem::path& file_path,
                                        Logger& logger,
                                        ProcessStats& stats) const {
  ScopedTimer timer(stats, "input_read_seconds");
  std::ifstream stream(file_path);
  if (!stream) {
    throw std::runtime_error("Unable to open PLY file: " + file_path.string());
  }

  const auto header = parseHeader(stream);
  PointCloud cloud;
  cloud.raw_point_count = header.vertex_count;
  cloud.property_names = header.property_names;
  cloud.points.reserve(header.vertex_count);

  std::string line;
  std::uint32_t point_index = 0;
  while (point_index < header.vertex_count && std::getline(stream, line)) {
    const auto point = parsePoint(line, header, point_index);
    if (point.has_value()) {
      cloud.bounds.expand(point->x, point->y, point->z);
      cloud.points.push_back(*point);
    }
    ++point_index;
  }

  if (point_index != header.vertex_count) {
    throw std::runtime_error("PLY point count does not match header vertex count.");
  }

  cloud.stored_point_count = cloud.points.size();
  stats.setCount("raw_point_count", cloud.raw_point_count);
  stats.setCount("loaded_point_count", cloud.points.size());
  logger.info("Loaded " + std::to_string(cloud.points.size()) + " points from " + file_path.string());
  return cloud;
}

/**
 * @brief 流式扫描 PLY 文件用于快速统计，不将所有点载入内存
 *
 * 统计内容：
 * - 总点数、NaN 点数、Inf 点数、有限点数
 * - 空间边界范围
 * - 等间距采样点集合（用于后续分析）
 * - 内存占用估算
 */
PointCloudScanSummary InputManager::scanPointCloud(const std::filesystem::path& file_path,
                                                   std::size_t sample_limit,
                                                   Logger& logger,
                                                   ProcessStats& stats) const {
  ScopedTimer timer(stats, "input_scan_seconds");
  PointCloudScanSummary summary;
  summary.sample_points.reserve(sample_limit);

  std::ifstream stream(file_path);
  if (!stream) {
    throw std::runtime_error("Unable to open PLY file: " + file_path.string());
  }

  const auto header = parseHeader(stream);
  summary.total_points = header.vertex_count;

  /* 按固定步长采样以控制采样总数在限制内 */
  std::string line;
  std::uint32_t point_index = 0;
  const std::size_t sampling_step = std::max<std::size_t>(1, header.vertex_count / std::max<std::size_t>(1, sample_limit));
  while (point_index < header.vertex_count && std::getline(stream, line)) {
    const auto point = parsePoint(line, header, point_index);
    if (point.has_value()) {
      if (std::isnan(point->x) || std::isnan(point->y) || std::isnan(point->z)) {
        ++summary.nan_points;
      } else if (!std::isfinite(point->x) || !std::isfinite(point->y) || !std::isfinite(point->z)) {
        ++summary.inf_points;
      } else {
        ++summary.finite_points;
        summary.bounds.expand(point->x, point->y, point->z);
        if (summary.sample_points.size() < sample_limit && (point_index % sampling_step == 0)) {
          summary.sample_points.push_back(*point);
        }
      }
    }
    ++point_index;
  }

  summary.estimated_memory_mb =
      static_cast<double>(estimatePointMemoryBytes(summary.total_points)) / (1024.0 * 1024.0);
  logger.info("Scanned " + std::to_string(summary.total_points) + " points, estimated in-memory footprint " +
              std::to_string(summary.estimated_memory_mb) + " MB");
  return summary;
}

/**
 * @brief 以回调方式流式遍历 PLY 文件中的每个点
 * 用于 Tile 物化阶段的大规模点分配，避免全量内存驻留。
 */
void InputManager::streamPoints(const std::filesystem::path& file_path,
                                const std::function<void(const Point3D&)>& callback) const {
  std::ifstream stream(file_path);
  if (!stream) {
    throw std::runtime_error("Unable to open PLY file: " + file_path.string());
  }

  const auto header = parseHeader(stream);
  std::string line;
  std::uint32_t point_index = 0;
  while (point_index < header.vertex_count && std::getline(stream, line)) {
    const auto point = parsePoint(line, header, point_index);
    if (point.has_value()) {
      callback(*point);
    }
    ++point_index;
  }
}

/**
 * @brief 解析 PLY 文件头部，提取元数据和属性索引映射
 *
 * 支持 ASCII 格式的 PLY 文件，提取：
 * - 顶点数量
 * - 属性名称列表
 * - x/y/z/r/g/b/classification 各属性的列索引
 */
PlyHeader InputManager::parseHeader(std::istream& stream) const {
  PlyHeader header;
  std::string line;
  bool saw_ply = false;
  bool in_vertex_block = false;

  while (std::getline(stream, line)) {
    line = trim(line);
    if (line.empty()) {
      continue;
    }
    const auto tokens = splitWhitespace(line);
    if (tokens.empty()) {
      continue;
    }
    /* 必须以 ply 标识符开头 */
    if (!saw_ply) {
      if (tokens[0] != "ply") {
        throw std::runtime_error("File is not a PLY file.");
      }
      saw_ply = true;
      continue;
    }
    /* 格式声明：仅支持 ASCII */
    if (tokens[0] == "format") {
      if (tokens.size() < 3 || tokens[1] != "ascii") {
        throw std::runtime_error("Only ASCII PLY is supported in this version.");
      }
      header.ascii = true;
      continue;
    }
    /* 元素声明：记录是否进入 vertex 块及顶点数 */
    if (tokens[0] == "element") {
      in_vertex_block = tokens.size() >= 3 && tokens[1] == "vertex";
      if (in_vertex_block) {
        header.vertex_count = static_cast<std::size_t>(std::stoull(tokens[2]));
      }
      continue;
    }
    /* 属性声明：建立名称到列索引的映射 */
    if (tokens[0] == "property" && in_vertex_block) {
      if (tokens.size() < 3) {
        throw std::runtime_error("Malformed property declaration in PLY header.");
      }
      header.property_names.push_back(tokens.back());
      const auto property_index = static_cast<int>(header.property_names.size() - 1);
      const auto property_name = toLower(tokens.back());
      if (property_name == "x") {
        header.x_index = property_index;
      } else if (property_name == "y") {
        header.y_index = property_index;
      } else if (property_name == "z") {
        header.z_index = property_index;
      } else if (property_name == "red") {
        header.r_index = property_index;
      } else if (property_name == "green") {
        header.g_index = property_index;
      } else if (property_name == "blue") {
        header.b_index = property_index;
      } else if (property_name == "classification") {
        header.classification_index = property_index;
      }
      continue;
    }
    /* 头部结束标记 */
    if (tokens[0] == "end_header") {
      break;
    }
  }

  if (!header.ascii) {
    throw std::runtime_error("PLY header does not declare ASCII format.");
  }
  if (header.vertex_count == 0) {
    throw std::runtime_error("PLY vertex count must be greater than 0.");
  }
  if (header.x_index < 0 || header.y_index < 0 || header.z_index < 0) {
    throw std::runtime_error("PLY file must contain x, y, and z properties.");
  }
  return header;
}

/**
 * @brief 解析单行 PLY 顶点数据为 Point3D 结构
 * 根据 parseHeader 中建立的属性索引映射提取各字段值。
 */
std::optional<Point3D> InputManager::parsePoint(const std::string& line,
                                                const PlyHeader& header,
                                                std::uint32_t index) const {
  const auto tokens = splitWhitespace(line);
  if (tokens.size() < header.property_names.size()) {
    throw std::runtime_error("Malformed PLY vertex line.");
  }

  Point3D point;
  point.index = index;
  point.x = std::stod(tokens[header.x_index]);
  point.y = std::stod(tokens[header.y_index]);
  point.z = std::stod(tokens[header.z_index]);

  /* 提取可选的颜色和分类属性 */
  if (header.r_index >= 0) {
    point.red = static_cast<std::uint8_t>(std::stoi(tokens[header.r_index]));
  }
  if (header.g_index >= 0) {
    point.green = static_cast<std::uint8_t>(std::stoi(tokens[header.g_index]));
  }
  if (header.b_index >= 0) {
    point.blue = static_cast<std::uint8_t>(std::stoi(tokens[header.b_index]));
  }
  if (header.classification_index >= 0) {
    point.classification = std::stoi(tokens[header.classification_index]);
  }
  return point;
}

/**
 * @brief 测试输出目录是否具有写权限
 * 通过创建临时探测文件的方式验证目录的可写性。
 */
bool InputManager::canWriteToDirectory(const std::filesystem::path& directory) const {
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  if (ec) {
    return false;
  }
  const auto probe_path = directory / ".dem_write_probe";
  std::ofstream probe(probe_path);
  if (!probe) {
    return false;
  }
  probe << "probe";
  probe.close();
  std::filesystem::remove(probe_path, ec);
  return true;
}

}  // namespace dem
