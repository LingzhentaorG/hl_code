#pragma once

#include "dem/Logger.hpp"
#include "dem/Types.hpp"

#include <filesystem>
#include <functional>
#include <istream>
#include <optional>

namespace dem {

/**
 * @brief PLY 文件头信息结构体
 *
 * 解析 PLY 文件头后得到的元数据，包含格式、顶点数和属性索引映射。
 */
struct PlyHeader {
  bool ascii = false;                          /**< 是否为 ASCII 格式 */
  std::size_t vertex_count = 0;                /**< 顶点（点）数量 */
  std::vector<std::string> property_names;     /**< 属性名称列表 */
  int x_index = -1;                            /**< x 属性在属性列表中的索引 */
  int y_index = -1;                            /**< y 属性在属性列表中的索引 */
  int z_index = -1;                            /**< z 属性在属性列表中的索引 */
  int r_index = -1;                            /**< red 属性索引（可选） */
  int g_index = -1;                            /**< green 属性索引（可选） */
  int b_index = -1;                            /**< blue 属性索引（可选） */
  int classification_index = -1;               /**< 分类属性索引（可选） */
};

/**
 * @brief 点云扫描摘要结构体
 *
 * 在完整读取点云之前通过采样扫描获取的统计摘要信息，
 * 用于快速评估数据质量、估算内存占用等。
 */
struct PointCloudScanSummary {
  std::size_t total_points = 0;                /**< 原始总点数（来自 PLY 头） */
  std::size_t finite_points = 0;               /**< 有效有限值点数 */
  std::size_t nan_points = 0;                  /**< 包含 NaN 的点数 */
  std::size_t inf_points = 0;                  /**< 包含 Inf 的点数 */
  std::size_t duplicate_hint_count = 0;        /**< 采样中发现的疑似重复点数 */
  std::size_t coordinate_warning_count = 0;    /**< 坐标异常警告计数 */
  Bounds bounds;                               /**< 有效点的空间范围 */
  double estimated_memory_mb = 0.0;            /**< 估算的内存占用（MB） */
  std::vector<Point3D> sample_points;          /**< 采样的样本点集 */
};

/**
 * @brief 输入验证报告结构体
 *
 * 对输入文件进行全面检查后生成的报告，包含头信息、扫描摘要、
 * 警告列表和处理建议。
 */
struct InputValidationReport {
  PlyHeader header;                            /**< PLY 文件头信息 */
  PointCloudScanSummary summary;               /**< 点云扫描摘要 */
  bool coordinate_order_suspect = false;       /**< 坐标轴顺序是否可疑 */
  bool output_directory_writable = false;      /**< 输出目录是否可写 */
  bool will_use_tile_mode = false;             /**< 是否将启用 Tile 分块模式 */
  std::vector<std::string> warnings;           /**< 警告消息列表 */
};

/**
 * @brief 输入管理器，负责 PLY 点云文件的读取、解析和验证
 *
 * 支持 ASCII PLY 格式的流式读取、头部解析、逐点回调处理，
 * 以及输入数据的完整性校验。
 */
class InputManager {
 public:
  /**
   * @brief 检查并解析 PLY 文件的头部信息
   * @param file_path PLY 文件路径
   * @return 解析后的 PLY 头部信息
   */
  PlyHeader inspectHeader(const std::filesystem::path& file_path) const;

  /**
   * @brief 对输入文件进行全面的验证检查
   * 包括头部解析、点云扫描、重复检测、坐标异常检测、输出目录写权限检查
   * @param file_path 输入 PLY 文件路径
   * @param config DEM 配置参数
   * @param output_directory 输出目录路径
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 完整的验证报告
   */
  InputValidationReport validateInput(const std::filesystem::path& file_path,
                                      const DEMConfig& config,
                                      const std::filesystem::path& output_directory,
                                      Logger& logger,
                                      ProcessStats& stats) const;

  /**
   * @brief 完整读取 PLY 文件中的所有点到内存中的点云结构
   * @param file_path PLY 文件路径
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 完整的点云对象
   */
  PointCloud readPointCloud(const std::filesystem::path& file_path, Logger& logger, ProcessStats& stats) const;

  /**
   * @brief 以采样方式扫描点云文件，获取统计摘要而不加载全部数据
   * @param file_path PLY 文件路径
   * @param sample_limit 最大采样点数
   * @param logger 日志记录器
   * @param stats 统计信息收集器
   * @return 扫描摘要信息
   */
  PointCloudScanSummary scanPointCloud(const std::filesystem::path& file_path,
                                       std::size_t sample_limit,
                                       Logger& logger,
                                       ProcessStats& stats) const;

  /**
   * @brief 流式逐点读取 PLY 文件，对每个有效点调用回调函数
   * 适用于内存敏感场景，避免一次性加载全部点云
   * @param file_path PLY 文件路径
   * @param callback 每个点的处理回调函数
   */
  void streamPoints(const std::filesystem::path& file_path,
                    const std::function<void(const Point3D&)>& callback) const;

 private:
  /**
   * @brief 从输入流中解析 PLY 文件头部
   * @param stream 输入流（位置将在解析后位于数据段起始处）
   * @return 解析后的头部信息
   */
  PlyHeader parseHeader(std::istream& stream) const;

  /**
   * @brief 将单行文本解析为一个三维点
   * @param line 原始文本行
   * @param header PLY 头部信息（用于确定属性位置）
   * @param index 点的全局索引号
   * @return 解析成功返回点对象，失败返回空
   */
  std::optional<Point3D> parsePoint(const std::string& line, const PlyHeader& header, std::uint32_t index) const;

  /**
   * @brief 检查指定目录是否可写入
   * 通过创建临时探测文件来验证写权限
   * @param directory 待检查的目录路径
   * @return true 表示目录可写
   */
  bool canWriteToDirectory(const std::filesystem::path& directory) const;
};

}  // namespace dem
