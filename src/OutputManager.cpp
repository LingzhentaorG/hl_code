/**
 * @file OutputManager.cpp
 * @brief 输出管理器实现：PLY 点云写入与 PNG 栅格导出
 */

#include "dem/OutputManager.hpp"
#include "dem/Utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>

namespace dem {

/**
 * @brief 将点云数据以 ASCII PLY 格式写入指定路径
 *
 * PLY 文件结构：
 * 1. 文件头（header）：声明格式、顶点数和属性列表
 * 2. 顶点数据：每行一个点，包含 x y z 及可选的分类属性
 *
 * 支持输出原始点云、种子点、地面点和非地面点四类。
 */
void OutputManager::writePly(const std::filesystem::path& output_path,
                             const PointCloud& cloud,
                             Logger& logger,
                             ProcessStats& stats) const {
  ScopedTimer timer(stats, "ply_write_seconds");
  std::ofstream stream(output_path);
  if (!stream) {
    throw std::runtime_error("Unable to open output file: " + output_path.string());
  }

  /* 写入 PLY 文件头 */
  stream << "ply\n";
  stream << "format ascii 1.0\n";
  stream << "element vertex " << cloud.points.size() << "\n";
  stream << "property float x\n";
  stream << "property float y\n";
  stream << "property float z\n";

  /* 根据点云类型决定是否写入分类属性列 */
  bool has_classification = false;
  if (!cloud.points.empty()) {
    has_classification = true;
  }
  if (has_classification) {
    stream << "property uchar classification\n";   /* 分类属性：0=非地面, 1=地面 */
  }
  stream << "end_header\n";

  /* 逐行写入顶点数据 */
  for (const auto& point : cloud.points) {
    stream << point.x << " " << point.y << " " << point.z;
    if (has_classification) {
      stream << " " << static_cast<int>(point.ground ? 1 : 0);
    }
    stream << "\n";
  }

  logger.info("Written PLY: " + output_path.string() +
              " (" + std::to_string(cloud.points.size()) + " points)");
}

/**
 * @brief 将 DEM 栅格以灰度 PNG 图像格式写入指定路径
 *
 * 像素映射策略：
 * 1. NoData 像素 → 黑色 (R=G=B=0)
 * 2. 有效高程 → 线性映射到 [0, 255] 灰度范围
 *    映射公式: gray = (z - min_z) / (max_z - min_z) × 255
 *
 * PNG 文件头遵循标准规范：
 * - 签名: \x89PNG\r\n\x1a\n
 * - IHDR: 宽度、高度、位深(8)、颜色类型(2=RGB)、压缩方式等
 * - IDAT: zlib 压缩的逐行像素数据
 * - IEND: 文件结束标记
 */
void OutputManager::writePng(const std::filesystem::path& output_path,
                             const DEMRaster& raster,
                             Logger& logger,
                             ProcessStats& stats) const {
  ScopedTimer timer(stats, "png_write_seconds");

  /* 扫描栅格数据获取有效高程的最小/最大值用于线性拉伸 */
  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  for (const auto value : raster.data) {
    if (value != raster.nodata_value) {
      min_z = std::min(min_z, value);
      max_z = std::max(max_z, value);
    }
  }

  /* 全部为 NoData 时使用默认范围避免除零 */
  if (!std::isfinite(min_z)) {
    min_z = 0.0;
    max_z = 255.0;
  }

  /* 将每行像素数据编码为 RGB 三元组并压缩存储 */
  const std::size_t row_bytes = raster.cols * 3;   /* 每像素 3 字节 (R+G+B) */
  std::vector<std::uint8_t> raw_data;
  raw_data.reserve((row_bytes + 1) * raster.rows);   /* +1 用于每行的 filter byte */

  for (std::size_t row = 0; row < raster.rows; ++row) {
    raw_data.push_back(0);   /* filter type: None (无过滤) */
    for (std::size_t col = 0; col < raster.cols; ++col) {
      const auto pixel_index = row * raster.cols + col;
      const auto value = raster.data[pixel_index];

      if (value == raster.nodata_value) {
        /* NoData → 黑色 */
        raw_data.push_back(0);
        raw_data.push_back(0);
        raw_data.push_back(0);
      } else {
        /* 有效高程 → 线性映射到 [0, 255] 灰度 */
        const double normalized = std::clamp(
            (value - min_z) / std::max(1e-6, max_z - min_z), 0.0, 1.0);
        const auto gray = static_cast<std::uint8_t>(std::round(normalized * 255.0));
        raw_data.push_back(gray);   /* R */
        raw_data.push_back(gray);   /* G */
        raw_data.push_back(gray);   /* B */
      }
    }
  }

  /* 使用 zlib 对原始数据进行压缩 */
  const std::size_t max_compressed_size = compressBound(raw_data.size());
  std::vector<std::uint8_t> compressed_data(max_compressed_size);
  uLongf compressed_length = static_cast<uLongf>(compressed_data.size());
  if (compress2(compressed_data.data(), &compressed_length,
                raw_data.data(), static_cast<uLong>(raw_data.size()), Z_DEFAULT_COMPRESSION) != Z_OK) {
    throw std::runtime_error("PNG compression failed.");
  }
  compressed_data.resize(compressed_length);

  /* 计算 CRC32 校验码辅助函数 */
  const auto crc32_for = [](const std::vector<std::uint8_t>& data) -> std::uint32_t {
    return crc32(0L, data.data(), static_cast<uInt>(data.size()));
  };

  /* 构建 IHDR 块（图像头信息） */
  const auto width = static_cast<std::uint32_t>(raster.cols);
  const auto height = static_cast<std::uint32_t>(raster.rows);
  std::vector<std::uint8_t> ihdr_data{static_cast<std::uint8_t>(width >> 24U),
                                       static_cast<std::uint8_t>(width >> 16U),
                                       static_cast<std::uint8_t>(width >> 8U),
                                       static_cast<std::uint8_t>(width),
                                       static_cast<std::uint8_t>(height >> 24U),
                                       static_cast<std::uint8_t>(height >> 16U),
                                       static_cast<std::uint8_t>(height >> 8U),
                                       static_cast<std::uint8_t>(height),
                                       8,   /* 位深：每通道 8 位 */
                                       2,   /* 颜色类型：RGB 彩色 */
                                       0,   /* 压缩方法：deflate */
                                       0,   /* 过滤方法：自适应 */
                                       0};  /* 隔行扫描：禁用 */

  /* 构建完整 PNG 文件并写入磁盘 */
  std::ofstream stream(output_path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("Unable to open output file: " + output_path.string());
  }

  /* PNG 文件签名 */
  const std::array<std::uint8_t, 8> png_signature{
      0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};
  stream.write(reinterpret_cast<const char*>(png_signature.data()),
               static_cast<std::streamsize>(png_signature.size()));

  /* 写入 IHDR 块（长度 + 类型 + 数据 + CRC32） */
  writeChunk(stream, "IHDR", ihdr_data, crc32_for);

  /* 写入 IDAT 块（压缩后的图像数据） */
  writeChunk(stream, "IDAT", compressed_data, crc32_for);

  /* 写入 IEND 块（文件结束标记，无数据负载） */
  writeChunk(stream, "IEND", {}, crc32_for);

  logger.info("Written PNG: " + output_path.string() +
              " (" + std::to_string(raster.cols) + "x" + std::to_string(raster.rows) + ")");
}

/**
 * @brief 辅助函数：将数据块按 PNG 规范格式写入流
 *
 * PNG 块结构: [4字节长度][4字节类型][N字节数据][4字节CRC32]
 * 其中长度字段不包含类型、数据和 CRC 本身的字节。
 */
void OutputManager::writeChunk(std::ofstream& stream,
                               const std::string& chunk_type,
                               const std::vector<std::uint8_t>& chunk_data,
                               const std::function<std::uint32_t(const std::vector<std::uint8_t>&)>& crc32_fn) const {
  /* 写入 4 字节大端序数据长度（不含类型和 CRC） */
  const auto length = static_cast<std::uint32_t>(chunk_data.size());
  const std::array<std::uint8_t, 4> length_bytes{static_cast<std::uint8_t>(length >> 24U),
                                                  static_cast<std::uint8_t>(length >> 16U),
                                                  static_cast<std::uint8_t>(length >> 8U),
                                                  static_cast<std::uint8_t>(length)};
  stream.write(reinterpret_cast<const char*>(length_bytes.data()),
               static_cast<std::streamsize>(length_bytes.size()));

  /* 写入 4 字节块类型标识符 */
  stream.write(chunk_type.c_str(), 4);

  /* 写入块数据负载 */
  if (!chunk_data.empty()) {
    stream.write(reinterpret_cast<const char*>(chunk_data.data()),
                 static_cast<std::streamsize>(chunk_data.size()));
  }

  /* 计算并写入 CRC32 校验码（覆盖类型 + 数据） */
  std::vector<std::uint8_t> crc_input(chunk_type.begin(), chunk_type.end());
  crc_input.insert(crc_input.end(), chunk_data.begin(), chunk_data.end());
  const auto checksum = crc32_fn(crc_input);
  const std::array<std::uint8_t, 4> crc_bytes{static_cast<std::uint8_t>(checksum >> 24U),
                                               static_cast<std::uint8_t>(checksum >> 16U),
                                               static_cast<std::uint8_t>(checksum >> 8U),
                                               static_cast<std::uint8_t>(checksum)};
  stream.write(reinterpret_cast<const char*>(crc_bytes.data()),
               static_cast<std::streamsize>(crc_bytes.size()));
}

}  // namespace dem
