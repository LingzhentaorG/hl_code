#pragma once

#include "dem/Logger.hpp"
#include "dem/Types.hpp"

namespace dem {

/**
 * @brief 坐标参考系（CRS）管理器，负责解析和确认 CRS 定义
 *
 * 支持通过 EPSG 编码、WKT 字符串或 WKT 文件三种方式定义坐标系，
 * 并将用户提供的配置统一转换为可用的 CRS 定义。
 */
class CRSManager {
 public:
  /**
   * @brief 解析并确认坐标参考系定义
   * @param config_crs 用户配置中的 CRS 定义
   * @param logger 日志记录器引用
   * @param stats 统计信息收集器引用
   * @return 解析后的完整 CRS 定义，包含 resolved_wkt 等派生字段
   */
  CRSDefinition resolve(const CRSDefinition& config_crs, Logger& logger, ProcessStats& stats) const;
};

}  // namespace dem
