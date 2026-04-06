#pragma once

#include "dem/Types.hpp"

#include <chrono>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace dem {

/**
 * @brief 作用域计时器（RAII 模式）
 *
 * 在构造时记录起始时间，析构时自动将经过的时间累加到指定的统计键中。
 * 用于精确测量各处理阶段的耗时。
 */
class ScopedTimer {
 public:
  /**
   * @brief 构造计时器并记录起始时刻
   * @param stats 统计信息收集器引用
   * @param key 耗时记录的键名
   */
  ScopedTimer(ProcessStats& stats, std::string key);
  /** 析构时自动计算并记录耗时 */
  ~ScopedTimer();

 private:
  ProcessStats& stats_;            /**< 统计信息收集器引用 */
  std::string key_;                /**< 耗时记录键名 */
  std::chrono::steady_clock::time_point started_at_;  /**< 起始时间点 */
};

/**
 * @brief 去除字符串首尾空白字符
 * @param value 输入字符串
 * @return 去除空白后的字符串
 */
[[nodiscard]] std::string trim(const std::string& value);

/**
 * @brief 按空白字符分割字符串
 * @param line 输入字符串
 * @return 分割后的子字符串列表
 */
[[nodiscard]] std::vector<std::string> splitWhitespace(const std::string& line);

/**
 * @brief 将字符串转换为小写
 * @param value 输入字符串
 * @return 小写字符串
 */
[[nodiscard]] std::string toLower(std::string value);

/**
 * @brief 将秒数格式化为可读字符串（如 "1.234s"）
 * @param seconds 秒数
 * @return 格式化字符串
 */
[[nodiscard]] std::string formatSeconds(double seconds);

/**
 * @brief 估算指定数量点云占用的内存字节数
 * @param point_count 点的数量
 * @return 估算的字节数（含额外开销）
 */
[[nodiscard]] std::size_t estimatePointMemoryBytes(std::size_t point_count);

/**
 * @brief 从点集中采样提取指定属性值
 * 通过等间距采样控制返回数量上限
 * @param points 点集合
 * @param max_count 最大采样数
 * @param extractor 属性提取函数
 * @return 采样的属性值列表
 */
[[nodiscard]] std::vector<double> sampleValues(const std::vector<Point3D>& points,
                                               std::size_t max_count,
                                               const std::function<double(const Point3D&)>& extractor);

/**
 * @brief 计算数值列表的指定分位数
 * 使用线性插值方法处理非整数位置
 * @param values 数值列表（将被部分排序修改）
 * @param q 分位数 [0.0, 1.0]
 * @return 分位数值
 */
[[nodiscard]] double quantile(std::vector<double> values, double q);

/**
 * @brief 计算数值列表的中位数（q=0.5 的分位数）
 * @param values 数值列表
 * @return 中位数值
 */
[[nodiscard]] double median(std::vector<double> values);

/**
 * @brief 计算数值列表的中位绝对偏差（MAD）
 * MAD = median(|xi - median(x)|)
 * @param values 数值列表
 * @return MAD 值
 */
[[nodiscard]] double mad(std::vector<double> values);

/**
 * @brief 计算以 (center_x, center_y) 为中心的 8 扇区中被占据的扇区数
 * 将 360 度均分为 8 个 45 度扇区，检查每个扇区内是否有邻域点分布
 * 用于评估点云的空间分布完整性，辅助判断是否处于数据边缘
 * @param center_x 中心点 X 坐标
 * @param center_y 中心点 Y 坐标
 * @param points 全部点集合
 * @param indices 邻域点索引列表
 * @return 被占据的扇区数量（0~8）
 */
[[nodiscard]] std::size_t occupiedSectors8(double center_x,
                                           double center_y,
                                           const std::vector<Point3D>& points,
                                           const std::vector<std::size_t>& indices);

/**
 * @brief 使用最小二乘法对一组点进行平面拟合
 * 拟合方程: z = a*x + b*y + c
 * 使用带主元选择的高斯消元法求解正规方程组
 * @param points 全部点集合
 * @param indices 参与拟合的点索引列表
 * @return 平面拟合结果（包含系数、坡角和稳定性指标）
 */
[[nodiscard]] PlaneFitResult fitPlaneLeastSquares(const std::vector<Point3D>& points,
                                                  const std::vector<std::size_t>& indices);

/**
 * @brief 创建带时间戳的输出子目录
 * 目录命名格式: base_dir/YYYYMMDD_HHMMSS
 * @param base_dir 基础目录路径
 * @return 带时间戳的完整目录路径
 */
[[nodiscard]] std::filesystem::path makeTimestampedOutputDir(const std::filesystem::path& base_dir);

/**
 * @brief 确保目录存在（递归创建）
 * @param dir 目标目录路径
 * @throw std::runtime_error 创建失败时抛出异常
 */
void ensureDirectory(const std::filesystem::path& dir);

}  // namespace dem
