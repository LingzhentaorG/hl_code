/**
 * @file main.cpp
 * @brief 应用程序入口点：命令行解析与主控制器调度
 */

#include "dem/MainController.hpp"
#include "dem/Config.hpp"
#include "dem/Logger.hpp"
#include "dem/ProcessStats.hpp"
#include "dem/Utils.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

/**
 * @brief 打印程序使用帮助信息到标准输出
 * 列出所有支持的命令行选项及其简要说明。
 */
void printUsage(const char* program_name) {
  std::cout << "Usage:\n";
  std::cout << "  " << program_name << " --input <ply_file> [--config <json>] [options]\n\n";

  std::cout << "Required:\n";
  std::cout << "  --input <path>       Path to the input PLY point cloud file.\n\n";

  std::cout << "Optional:\n";
  std::cout << "  --config <path>      Path to the configuration JSON file.\n";
  std::cout << "                       Default: config.json in the working directory.\n";
  std::cout << "  --output-dir <path>  Override the output directory.\n";
  std::cout << "  --set key=value      Override a specific configuration parameter.\n";
  std::cout << "                       Supports dot notation, e.g., ground.knn=12.\n";
  std::cout << "  --help               Show this help message and exit.\n";
  std::cout << "  --version            Show version information and exit.\n";
}

/**
 * @brief 打印版本信息到标准输出
 */
void printVersion() {
  std::cout << "DEM Generator v1.0.0\n";
  std::cout << "Optical Surveying Satellite Photogrammetry Point Cloud -> DEM\n";
}

/**
 * @brief 简易命令行参数解析器
 *
 * 支持的参数格式：
 *   --input <value>     必需，输入 PLY 文件路径
 *   --config <value>    可选，JSON 配置文件路径（默认 config.json）
 *   --output-dir <value> 可选，覆盖输出目录
 *   --set key=value     可选，运行时配置覆盖（支持多组）
 *   --help              显示帮助信息
 *   --version           显示版本信息
 *
 * @param argc 参数计数
 * @param argv 参数数组
 * @return 包含解析结果的轻量级结构体
 */
CommandLineArgs parseCommandLine(int argc, char* argv[]) {
  CommandLineArgs args;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);

    if (arg == "--help" || arg == "-h") {
      args.show_help = true;
      return args;
    }

    if (arg == "--version" || arg == "-v") {
      args.show_version = true;
      return args;
    }

    /* 需要紧跟值的参数类型 */
    if ((arg == "--input" || arg == "-i") && i + 1 < argc) {
      args.input_file = argv[++i];
    } else if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
      args.config_path = argv[++i];
    } else if ((arg == "--output-dir" || arg == "-o") && i + 1 < argc) {
      args.output_dir = argv[++i];
    } else if (arg == "--set" && i + 1 < argc) {
      /* --set 参数支持多次使用，收集到 overrides 向量中 */
      args.overrides.push_back(argv[++i]);
    }
  }

  return args;
}

}  // namespace

/**
 * @brief 主入口函数
 *
 * 执行流程：
 * 1. 解析命令行参数
 * 2. 处理 --help / --version 特殊请求
 * 3. 加载并验证配置文件
 * 4. 初始化日志系统和性能统计器
 * 5. 调用 MainController 执行完整 DEM 处理流水线
 * 6. 捕获并友好显示异常信息
 *
 * @return 0 表示成功，非零表示错误
 */
int main(int argc, char* argv[]) {
  try {
    /* 第一步：解析命令行参数 */
    const auto args = parseCommandLine(argc, argv);

    /* 第二步：处理特殊请求 */
    if (args.show_help) {
      printUsage(argv[0]);
      return 0;
    }
    if (args.show_version) {
      printVersion();
      return 0;
    }

    /* 第三步：验证必需参数并提供默认值 */
    if (args.input_file.empty()) {
      std::cerr << "Error: --input is required.\n\n";
      printUsage(argv[0]);
      return 1;
    }

    /* 第四步：加载配置文件（默认使用当前目录下的 config.json） */
    dem::ConfigManager config_manager;
    const auto config_path = args.config_path.empty()
                                 ? std::filesystem::path("config.json")
                                 : std::filesystem::path(args.config_path);

    auto config = config_manager.load(config_path, args.overrides);

    /* 命令行参数覆盖配置中的对应字段（优先级高于配置文件） */
    if (!args.input_file.empty()) {
      config.input_path = args.input_file;
    }
    if (!args.output_dir.empty()) {
      config.output_directory = args.output_dir;
    }

    /* 第五步：初始化日志和统计系统 */
    dem::Logger logger;
    dem::ProcessStats stats;

    /* 在输出目录下创建 log 子目录并打开日志文件 */
    const auto log_path = config.output_directory / "log" / "processing.log";
    dem::ensureDirectoryExists(log_path.parent_path());
    logger.open(log_path);

    /* 第六步：执行 DEM 生成流水线 */
    dem::MainController controller;
    controller.run(config.input_path, config, config.output_directory, logger, stats);

    return 0;   /* 正常退出 */

  } catch (const std::exception& e) {
    /* 友好地显示异常信息并返回错误码 */
    std::cerr << "\nError: " << e.what() << '\n';
    return 1;
  }
}
