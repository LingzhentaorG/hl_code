#pragma once

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

namespace dem {

/**
 * @brief 日志记录器，提供线程安全的日志输出功能
 *
 * 支持将日志同时输出到控制台和文件，
 * 提供 Info / Warning / Error 三种日志级别。
 */
class Logger {
 public:
  /** 日志级别枚举 */
  enum class Level { Info, Warning, Error };

  /**
   * @brief 打开日志文件，后续日志将同时写入该文件
   * @param log_path 日志文件路径
   */
  void open(const std::filesystem::path& log_path);

  /**
   * @brief 记录一条 Info 级别日志
   * @param message 日志消息内容
   */
  void info(const std::string& message);

  /**
   * @brief 记录一条 Warning 级别日志
   * @param message 警告消息内容
   */
  void warn(const std::string& message);

  /**
   * @brief 记录一条 Error 级别日志
   * @param message 错误消息内容
   */
  void error(const std::string& message);

 private:
  /**
   * @brief 内部写入方法，统一处理各级别日志的格式化和输出
   * @param level 日志级别
   * @param message 日志消息
   */
  void write(Level level, const std::string& message);

  std::mutex mutex_;       /**< 互斥锁，保证多线程安全 */
  std::ofstream stream_;   /**< 日志文件输出流 */
};

}  // namespace dem
