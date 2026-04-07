/**
 * @file Logger.cpp
 * @brief 线程安全日志记录器实现：支持多级别日志输出到控制台和文件
 */

#include "dem/Logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string_view>

#ifdef _WIN32
#include <windows.h>
#endif

namespace dem {
namespace {

#ifdef _WIN32
std::wstring utf8ToWide(const std::string& value) {
  const int wide_size = ::MultiByteToWideChar(CP_UTF8, 0, value.c_str(), -1, nullptr, 0);
  if (wide_size <= 0) {
    return {};
  }

  std::wstring wide_value(static_cast<std::size_t>(wide_size), L'\0');
  ::MultiByteToWideChar(CP_UTF8, 0, value.c_str(), -1, wide_value.data(), wide_size);
  if (!wide_value.empty() && wide_value.back() == L'\0') {
    wide_value.pop_back();
  }
  return wide_value;
}

void writeWideAsAnsi(std::wstring_view value) {
  const int ansi_size = ::WideCharToMultiByte(CP_ACP, 0, value.data(), static_cast<int>(value.size()), nullptr, 0, nullptr, nullptr);
  if (ansi_size <= 0) {
    return;
  }
  std::string ansi_value(static_cast<std::size_t>(ansi_size), '\0');
  ::WideCharToMultiByte(
      CP_ACP, 0, value.data(), static_cast<int>(value.size()), ansi_value.data(), ansi_size, nullptr, nullptr);
  std::cout << ansi_value << '\n';
}

void writeUtf8LineToConsole(const std::string& line) {
  HANDLE handle = ::GetStdHandle(STD_OUTPUT_HANDLE);
  DWORD mode = 0;
  const auto wide_line = utf8ToWide(line);
  if (wide_line.empty()) {
    std::cout << line << '\n';
    return;
  }

  if (handle == INVALID_HANDLE_VALUE || !::GetConsoleMode(handle, &mode)) {
    writeWideAsAnsi(wide_line);
    return;
  }

  DWORD written = 0;
  ::WriteConsoleW(handle, wide_line.c_str(), static_cast<DWORD>(wide_line.size()), &written, nullptr);
  ::WriteConsoleW(handle, L"\n", 1, &written, nullptr);
}
#endif

}  // namespace

/**
 * @brief 打开日志输出文件
 * 后续的 info/warn/error 调用将同时写入该文件。
 */
void Logger::open(const std::filesystem::path& log_path) {
  std::scoped_lock lock(mutex_);
  stream_.open(log_path);
}

/** @brief 输出信息级别日志 */
void Logger::info(const std::string& message) { write(Level::Info, message); }

/** @brief 输出警告级别日志 */
void Logger::warn(const std::string& message) { write(Level::Warning, message); }

/** @brief 输出错误级别日志 */
void Logger::error(const std::string& message) { write(Level::Error, message); }

/**
 * @brief 核心日志写入方法，带时间戳和级别前缀
 *
 * 日志格式: YYYY-MM-DD HH:MM:SS [LEVEL] message
 * 同时输出到标准控制台和已打开的日志文件。
 * 使用 mutex_ 保证多线程环境下的线程安全性。
 */
void Logger::write(Level level, const std::string& message) {
  /* 获取当前本地时间并格式化为字符串 */
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
#ifdef _WIN32
  localtime_s(&local_tm, &now_time);
#else
  localtime_r(&now_time, &local_tm);
#endif

  /* 构建完整日志行：时间戳 + 级别标签 + 消息内容 */
  std::ostringstream line;
  line << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S") << " ";
  switch (level) {
    case Level::Info:
      line << "[INFO] ";
      break;
    case Level::Warning:
      line << "[WARN] ";
      break;
    case Level::Error:
      line << "[ERROR] ";
      break;
  }
  line << message;

  /* 加锁后同时输出到控制台和文件 */
  std::scoped_lock lock(mutex_);
#ifdef _WIN32
  writeUtf8LineToConsole(line.str());
#else
  std::cout << line.str() << '\n';
#endif
  if (stream_.is_open()) {
    stream_ << line.str() << '\n';
    stream_.flush();   /* 实时刷新确保日志不丢失 */
  }
}

}  // namespace dem
