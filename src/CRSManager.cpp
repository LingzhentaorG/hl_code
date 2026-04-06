/**
 * @file CRSManager.cpp
 * @brief 坐标参考系管理器实现：CRS 定义解析与 WKT 生成
 */

#include "dem/CRSManager.hpp"

#include "dem/Utils.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace dem {

/**
 * @brief 解析并确定最终的坐标参考系定义
 *
 * CRS 优先级（从高到低）：
 * 1. crs_wkt 字符串直接使用
 * 2. crs_wkt_file 指定的文件内容作为 WKT
 * 3. epsg_code 编码转换为权威名称
 * 4. allow_unknown_crs=true 时标记为未知但允许继续
 *
 * 解析结果写入 resolved_wkt、known 和 authority_name 字段。
 */
CRSDefinition CRSManager::resolve(const CRSDefinition& config_crs, Logger& logger, ProcessStats& stats) const {
  ScopedTimer timer(stats, "crs_resolve_seconds");
  CRSDefinition resolved = config_crs;

  /* 优先级1：直接提供 WKT 字符串 */
  if (!resolved.crs_wkt.empty()) {
    resolved.resolved_wkt = resolved.crs_wkt;
    resolved.known = true;
    resolved.authority_name = "WKT";
  }
  /* 优先级2：从外部文件加载 WKT */
  else if (!resolved.crs_wkt_file.empty()) {
    std::ifstream stream(resolved.crs_wkt_file);
    if (!stream) {
      throw std::runtime_error("Unable to open CRS WKT file: " + resolved.crs_wkt_file.string());
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    resolved.resolved_wkt = buffer.str();
    resolved.known = !trim(resolved.resolved_wkt).empty();
    resolved.authority_name = "WKT_FILE";
  }
  /* 优先级3：EPSG 编码 */
  else if (resolved.epsg_code > 0) {
    resolved.known = true;
    resolved.authority_name = "EPSG:" + std::to_string(resolved.epsg_code);
  }
  /* 优先级4：允许未知 CRS 继续处理 */
  else if (resolved.allow_unknown_crs) {
    resolved.known = false;
    resolved.authority_name = "UNKNOWN";
  }
  else {
    throw std::runtime_error("No CRS information available.");
  }

  logger.info("Resolved CRS: " + resolved.authority_name);
  return resolved;
}

}  // namespace dem
