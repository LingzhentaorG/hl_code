/**
 * @file Utils.cpp
 * @brief 通用工具函数实现：数学统计、字符串处理、文件 I/O 与平面拟合
 */

#include "dem/Utils.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <numeric>
#include <numbers>
#include <random>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dem {

/* ======================== 数学统计工具 ======================== */

/**
 * @brief 计算一组数值的中位数（排序后取中间值）
 * 对偶数长度取上下两值的平均。
 */
double median(std::span<const double> values) {
  if (values.empty()) { return 0.0; }
  std::vector<double> sorted(values.begin(), values.end());
  std::ranges::sort(sorted);
  const std::size_t n = sorted.size();
  return (n % 2 == 0) ? (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0 : sorted[n / 2];
}

/**
 * @brief 计算 MAD（中位绝对偏差）
 * MAD = median(|xi - median(x)|)，是标准差的鲁棒替代量。
 */
double mad(std::span<const double> values) {
  if (values.empty()) { return 0.0; }
  const double med = median(values);
  std::vector<double> deviations;
  deviations.reserve(values.size());
  for (const double value : values) {
    deviations.push_back(std::abs(value - med));
  }
  return median(deviations);
}

/**
 * @brief 计算指定分位数位置的值
 * 使用线性插值方法处理非整数位置的情况。
 */
double quantile(std::span<const double> values, double q) {
  if (values.empty()) { return 0.0; }
  std::vector<double> sorted(values.begin(), values.end());
  std::ranges::sort(sorted);
  const double pos = q * static_cast<double>(sorted.size() - 1);
  const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
  const std::size_t hi = std::min(lo + 1, sorted.size() - 1);
  const double fraction = pos - static_cast<double>(lo);
  return sorted[lo] * (1.0 - fraction) + sorted[hi] * fraction;
}

/**
 * @brief 最小二乘法拟合三维点到平面 z = ax + by + c
 *
 * 通过法方程求解超定线性方程组：
 * | Σx²   Σxy   Σx | |a|   | Σxz |
 * | Σxy   Σy²   Σy | |b| = | Σyz |
 * | Σx    Σy    n  | |c|   | Σz  |
 *
 * @return 平面系数 a, b, c 及坡度角度和条件指示器
 */
FittedPlane fitPlaneLeastSquares(const std::vector<Point3D>& points, const std::vector<std::size_t>& indices) {
  FittedPlane plane{};

  /* 至少需要 3 个点才能确定一个平面 */
  if (indices.size() < 3) {
    plane.success = false;
    return plane;
  }

  /* 累加法方程的各项系数 */
  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  double sum_xx = 0.0, sum_xy = 0.0, sum_xz = 0.0;
  double sum_yy = 0.0, sum_yz = 0.0;
  const double n = static_cast<double>(indices.size());

  for (const auto idx : indices) {
    const double x = points[idx].x;
    const double y = points[idx].y;
    const double z = points[idx].z;
    sum_x += x;
    sum_y += y;
    sum_z += z;
    sum_xx += x * x;
    sum_xy += x * y;
    sum_xz += x * z;
    sum_yy += y * y;
    sum_yz += y * z;
  }

  /* 构建正规方程组的系数矩阵和右侧向量 */
  /* A · [a, b, c]^T = B */
  const double det = sum_xx * (sum_yy * n - sum_y * sum_y) -
                     sum_xy * (sum_xy * n - sum_y * sum_x) +
                     sum_x * (sum_xy * sum_y - sum_yy * sum_x);

  /* 行列式接近零表示点共线或共面退化，无法唯一确定平面 */
  plane.condition_indicator = std::abs(det);
  if (std::abs(det) < 1e-12) {
    plane.success = false;
    return plane;
  }

  /* Cramer 法则求解三元一次方程组 */
  plane.a = (sum_xz * (sum_yy * n - sum_y * sum_y) -
             sum_xy * (sum_yz * n - sum_y * sum_z) +
             sum_x * (sum_yz * sum_y - sum_yy * sum_z)) / det;

  plane.b = (sum_xx * (sum_yz * n - sum_y * sum_z) -
             sum_xz * (sum_xy * n - sum_y * sum_x) +
             sum_x * (sum_xy * sum_z - sum_yz * sum_x)) / det;

  plane.c = (sum_xx * (sum_yy * sum_z - sum_y * sum_yz) -
             sum_xy * (sum_xy * sum_z - sum_y * sum_xz) +
             sum_x * (sum_xy * sum_yz - sum_yy * sum_xz)) / det;

  /* 计算法向量 (a, b, -1) 的坡角（弧度转角度） */
  const double normal_magnitude = std::sqrt(plane.a * plane.a + plane.b * plane.b + 1.0);
  plane.slope_deg = std::acos(1.0 / normal_magnitude) * 180.0 / std::numbers::pi;
  plane.success = true;
  return plane;
}

/* ======================== 字符串处理工具 ======================== */

/** @brief 去除字符串首尾空白字符 */
std::string trim(const std::string& str) {
  const auto start = str.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) { return ""; }
  const auto end = str.find_last_not_of(" \t\r\n");
  return str.substr(start, end - start + 1);
}

/** @brief 将字符串转换为小写形式 */
std::string toLower(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return str;
}

/* ======================== 空间分析工具 ======================== */

/**
 * @brief 统计中心点周围 8 个扇区中被占用的数量
 *
 * 扇区划分方式（从正 X 轴逆时针旋转）：
 *   2 | 3 | 4
 *   ---------
 *   1 | ● | 5
 *   ---------
 *   0 | 7 | 6
 *
 * 用于评估邻域的空间覆盖完整性，辅助判断是否位于数据边缘。
 */
int occupiedSectors8(double center_x,
                     double center_y,
                     const std::vector<Point3D>& points,
                     const std::vector<std::size_t>& indices) {
  constexpr int sector_count = 8;   /* 8 个等分扇区 */
  std::bitset<sector_count> occupied;

  for (const auto idx : indices) {
    const double dx = points[idx].x - center_x;
    const double dy = points[idx].y - center_y;
    /* 计算方位角并映射到 [0, 7] 扇区编号 */
    double angle = std::atan2(dy, dx) * 180.0 / std::numbers::pi;
    if (angle < 0.0) { angle += 360.0; }
    const int sector = static_cast<int>(angle / 45.0) % sector_count;
    occupied.set(sector);
  }

  return static_cast<int>(occupied.count());   /* 返回被占用扇区总数 */
}

/* ======================== 文件系统工具 ======================== */

/**
 * @brief 创建目录及其所有父目录（静默忽略已存在的情况）
 */
bool ensureDirectoryExists(const std::filesystem::path& dir_path) {
  try {
    std::filesystem::create_directories(dir_path);
    return true;
  } catch (...) {
    return false;
  }
}

/**
 * @brief 检查路径是否存在且为常规文件
 */
bool fileExists(const std::filesystem::path& path) {
  std::error_code ec;
  return std::filesystem::is_regular_file(path, ec);
}

/* ======================== JSON 报告导出工具 ======================== */

/**
 * @brief 将处理结果导出为结构化的 JSON 报告文件
 *
 * 包含内容：
 * - 输入/输出元数据（文件名、路径、时间戳）
 * - 配置快照（当前使用的完整配置参数）
 * - CRS 信息（权威名称、EPSG 编码等）
 * - 处理统计（各阶段耗时、点数变化、空洞填充进度等）
 * - DEM 栅格属性（尺寸、范围、NoData 值等）
 */
void exportJsonReport(const std::filesystem::path& report_path,
                      const DEMRaster& raster,
                      const DEMConfig& config,
                      const ProcessStats& stats,
                      const CRSDefinition& crs) {
  nlohmann::json root;

  /* 元数据部分 */
  root["metadata"]["generated_at"] = currentTimeISO8601();
  root["metadata"]["input_file"] = config.input_path.string();
  root["metadata"]["output_directory"] = config.output_directory.string();

  /* 配置快照 */
  root["config"] = config.toJson();

  /* CRS 信息 */
  root["crs"]["authority_name"] = crs.authority_name;
  root["crs"]["epsg_code"] = crs.epsg_code;
  root["crs"]["known"] = crs.known;

  /* 处理统计数据 */
  root["stats"] = stats.toJson();

  /* DEM 栅格属性 */
  root["raster"]["cols"] = raster.cols;
  root["raster"]["rows"] = raster.rows;
  root["raster"]["resolution"] = raster.resolution;
  root["raster"]["nodata_value"] = raster.nodata_value;
  root["raster"]["xmin"] = raster.xmin;
  root["raster"]["ymin"] = raster.ymin;
  root["raster"]["xmax"] = raster.xmax;
  root["raster"]["ymax"] = raster.ymax;

  /* 写入文件（带缩进的易读格式） */
  std::ofstream stream(report_path);
  stream << std::setw(4) << root << '\n';
}

/**
 * @brief 生成 ISO-8601 格式的当前 UTC 时间字符串
 * 格式示例: 2025-01-15T10:30:45Z
 */
std::string currentTimeISO8601() {
  const auto now = std::time(nullptr);
  std::tm buf{};
#ifdef _WIN32
  gmtime_s(&buf, &now);
#else
  gmtime_r(&now, &buf);
#endif
  std::ostringstream oss;
  oss << std::put_time(&buf, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

}  // namespace dem
