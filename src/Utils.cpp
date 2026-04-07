/**
 * @file Utils.cpp
 * @brief 通用工具函数实现：数学统计、字符串处理、文件系统与统计容器
 */

#include "dem/Utils.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace dem {
namespace {

template <typename StringT>
std::string utf8BytesFromPathString(const StringT& value) {
#if defined(__cpp_char8_t)
  if constexpr (std::is_same_v<typename StringT::value_type, char8_t>) {
    return std::string(reinterpret_cast<const char*>(value.data()), value.size());
  } else {
    return std::string(value.begin(), value.end());
  }
#else
  return std::string(value.begin(), value.end());
#endif
}

}  // namespace

void Bounds::expand(double x, double y, double z) noexcept {
  xmin = std::min(xmin, x);
  xmax = std::max(xmax, x);
  ymin = std::min(ymin, y);
  ymax = std::max(ymax, y);
  zmin = std::min(zmin, z);
  zmax = std::max(zmax, z);
}

void Bounds::merge(const Bounds& other) noexcept {
  if (!other.valid()) {
    return;
  }
  expand(other.xmin, other.ymin, other.zmin);
  expand(other.xmax, other.ymax, other.zmax);
}

bool Bounds::containsXY(double x, double y) const noexcept {
  return valid() && x >= xmin && x <= xmax && y >= ymin && y <= ymax;
}

bool Bounds::intersectsXY(const Bounds& other) const noexcept {
  if (!valid() || !other.valid()) {
    return false;
  }
  return !(other.xmax < xmin || other.xmin > xmax || other.ymax < ymin || other.ymin > ymax);
}

Bounds Bounds::expanded(double margin) const noexcept {
  Bounds result = *this;
  if (!valid()) {
    return result;
  }
  result.xmin -= margin;
  result.xmax += margin;
  result.ymin -= margin;
  result.ymax += margin;
  result.zmin -= margin;
  result.zmax += margin;
  return result;
}

void ProcessStats::addCount(const std::string& key, std::size_t value) { counts[key] += value; }

void ProcessStats::setCount(const std::string& key, std::size_t value) { counts[key] = value; }

std::size_t ProcessStats::getCount(const std::string& key) const {
  const auto it = counts.find(key);
  return it == counts.end() ? 0U : it->second;
}

void ProcessStats::setValue(const std::string& key, double value) { values[key] = value; }

double ProcessStats::getValue(const std::string& key) const {
  const auto it = values.find(key);
  return it == values.end() ? 0.0 : it->second;
}

void ProcessStats::addDuration(const std::string& key, double seconds) { durations_seconds[key] += seconds; }

ScopedTimer::ScopedTimer(ProcessStats& stats, std::string key)
    : stats_(stats), key_(std::move(key)), started_at_(std::chrono::steady_clock::now()) {}

ScopedTimer::~ScopedTimer() {
  const auto ended_at = std::chrono::steady_clock::now();
  stats_.addDuration(key_, std::chrono::duration<double>(ended_at - started_at_).count());
}

std::string trim(const std::string& value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::vector<std::string> splitWhitespace(const std::string& line) {
  std::istringstream stream(line);
  std::vector<std::string> tokens;
  for (std::string token; stream >> token;) {
    tokens.push_back(token);
  }
  return tokens;
}

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

std::string formatSeconds(double seconds) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << seconds << "s";
  return stream.str();
}

std::string pathToUtf8String(const std::filesystem::path& path) {
  return utf8BytesFromPathString(path.u8string());
}

std::string pathToGenericUtf8String(const std::filesystem::path& path) {
  return utf8BytesFromPathString(path.generic_u8string());
}

std::size_t estimatePointMemoryBytes(std::size_t point_count) {
  return point_count * sizeof(Point3D) + point_count * sizeof(std::size_t);
}

std::vector<double> sampleValues(const std::vector<Point3D>& points,
                                 std::size_t max_count,
                                 const std::function<double(const Point3D&)>& extractor) {
  std::vector<double> values;
  if (points.empty() || max_count == 0) {
    return values;
  }
  const std::size_t step = std::max<std::size_t>(1, points.size() / max_count);
  values.reserve(std::min(points.size(), max_count));
  for (std::size_t i = 0; i < points.size() && values.size() < max_count; i += step) {
    values.push_back(extractor(points[i]));
  }
  if (values.empty()) {
    values.push_back(extractor(points.front()));
  }
  return values;
}

double quantile(std::vector<double> values, double q) {
  if (values.empty()) {
    return 0.0;
  }
  q = std::clamp(q, 0.0, 1.0);
  std::sort(values.begin(), values.end());
  const double position = q * static_cast<double>(values.size() - 1);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  if (lower == upper) {
    return values[lower];
  }
  const double fraction = position - static_cast<double>(lower);
  return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

double median(std::vector<double> values) { return quantile(std::move(values), 0.5); }

double mad(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  const double med = median(values);
  for (double& value : values) {
    value = std::abs(value - med);
  }
  return median(std::move(values));
}

std::size_t occupiedSectors8(double center_x,
                             double center_y,
                             const std::vector<Point3D>& points,
                             const std::vector<std::size_t>& indices) {
  std::bitset<8> sectors;
  for (const auto idx : indices) {
    const double dx = points[idx].x - center_x;
    const double dy = points[idx].y - center_y;
    if (std::abs(dx) < 1e-9 && std::abs(dy) < 1e-9) {
      continue;
    }
    double angle = std::atan2(dy, dx) * 180.0 / std::numbers::pi;
    if (angle < 0.0) {
      angle += 360.0;
    }
    sectors.set(static_cast<std::size_t>(angle / 45.0) % 8U);
  }
  return sectors.count();
}

PlaneFitResult fitPlaneLeastSquares(const std::vector<Point3D>& points,
                                    const std::vector<std::size_t>& indices) {
  PlaneFitResult plane;
  if (indices.size() < 3) {
    return plane;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double sum_xz = 0.0;
  double sum_yy = 0.0;
  double sum_yz = 0.0;
  const double n = static_cast<double>(indices.size());

  for (const auto idx : indices) {
    const auto& point = points[idx];
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
    sum_xx += point.x * point.x;
    sum_xy += point.x * point.y;
    sum_xz += point.x * point.z;
    sum_yy += point.y * point.y;
    sum_yz += point.y * point.z;
  }

  const double det = sum_xx * (sum_yy * n - sum_y * sum_y) -
                     sum_xy * (sum_xy * n - sum_x * sum_y) +
                     sum_x * (sum_xy * sum_y - sum_x * sum_yy);
  plane.condition_indicator = std::abs(det);
  if (std::abs(det) < 1e-12) {
    return plane;
  }

  plane.a = (sum_xz * (sum_yy * n - sum_y * sum_y) -
             sum_xy * (sum_yz * n - sum_z * sum_y) +
             sum_x * (sum_yz * sum_y - sum_z * sum_yy)) / det;
  plane.b = (sum_xx * (sum_yz * n - sum_z * sum_y) -
             sum_xz * (sum_xy * n - sum_x * sum_y) +
             sum_x * (sum_xy * sum_z - sum_xz * sum_y)) / det;
  plane.c = (sum_xx * (sum_yy * sum_z - sum_y * sum_yz) -
             sum_xy * (sum_xy * sum_z - sum_x * sum_yz) +
             sum_x * (sum_xy * sum_yz - sum_xz * sum_yy)) / det;
  plane.slope_deg = std::atan(std::sqrt(plane.a * plane.a + plane.b * plane.b)) * 180.0 / std::numbers::pi;
  plane.success = true;
  return plane;
}

std::filesystem::path makeTimestampedOutputDir(const std::filesystem::path& base_dir) {
  const auto now = std::time(nullptr);
  std::tm local_tm{};
#ifdef _WIN32
  localtime_s(&local_tm, &now);
#else
  localtime_r(&now, &local_tm);
#endif

  std::ostringstream suffix;
  suffix << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
  return base_dir / suffix.str();
}

void ensureDirectory(const std::filesystem::path& dir) {
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    throw std::runtime_error("Unable to create directory: " + pathToUtf8String(dir));
  }
}

}  // namespace dem
