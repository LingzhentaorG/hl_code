/**
 * @file main.cpp
 * @brief 应用程序入口点：命令行解析与主控制器调度
 */

#include "dem/MainController.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct CommandLineArgs {
  std::filesystem::path config_path = "config.json";
  std::vector<std::string> overrides;
  bool show_help = false;
  bool show_version = false;
};

void printUsage(const char* program_name) {
  std::cout << "Usage:\n";
  std::cout << "  " << program_name << " [--config <json>] [--set key=value ...]\n\n";
  std::cout << "Options:\n";
  std::cout << "  --config <path>  Path to the configuration JSON file.\n";
  std::cout << "  --set key=value  Override a specific configuration parameter.\n";
  std::cout << "  --help           Show this help message and exit.\n";
  std::cout << "  --version        Show version information and exit.\n";
}

void printVersion() { std::cout << "DEM Generator v1.0.0\n"; }

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
    if ((arg == "--config" || arg == "-c") && i + 1 < argc) {
      args.config_path = argv[++i];
      continue;
    }
    if (arg == "--set" && i + 1 < argc) {
      args.overrides.emplace_back(argv[++i]);
      continue;
    }
    throw std::runtime_error("Unknown or incomplete argument: " + arg);
  }
  return args;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto args = parseCommandLine(argc, argv);
    if (args.show_help) {
      printUsage(argv[0]);
      return 0;
    }
    if (args.show_version) {
      printVersion();
      return 0;
    }

    dem::MainController controller;
    return controller.run(args.config_path, args.overrides);
  } catch (const std::exception& e) {
    std::cerr << "\nError: " << e.what() << '\n';
    return 1;
  }
}
