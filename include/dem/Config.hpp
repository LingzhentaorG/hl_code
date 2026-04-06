#pragma once

#include "dem/Types.hpp"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace dem {

/**
 * @brief 配置管理器，负责加载、解析、验证和写入 DEM 处理配置
 *
 * 该类从 JSON 配置文件中读取所有处理参数，支持运行时参数覆盖，
 * 并在加载后对配置进行归一化和合法性校验。
 */
class ConfigManager {
 public:
  /**
   * @brief 从指定路径加载配置文件，并应用运行时覆盖参数
   * @param path JSON 配置文件路径
   * @param overrides 运行时覆盖参数列表，格式为 "key.subkey=value"
   * @return 解析并校验后的完整配置结构体
   */
  DEMConfig load(const std::filesystem::path& path, const std::vector<std::string>& overrides) const;

  /**
   * @brief 将解析后的完整配置回写到文件，用于记录实际使用的参数
   * @param config 已解析的配置对象
   * @param output_path 输出文件路径
   */
  void writeResolvedConfig(const DEMConfig& config, const std::filesystem::path& output_path) const;

 private:
  /**
   * @brief 将命令行 --set 参数覆盖到 JSON 根节点上
   * @param root JSON 根节点（将被修改）
   * @param overrides 覆盖参数列表
   */
  void applyOverrides(nlohmann::json& root, const std::vector<std::string>& overrides) const;

  /**
   * @brief 从已解析的 JSON 对象中提取所有配置字段
   * @param root 已解析的 JSON 根节点
   * @return 填充好的配置结构体（尚未归一化）
   */
  DEMConfig parse(const nlohmann::json& root) const;

  /**
   * @brief 归一化派生值，将未显式设置的参数根据其他参数自动计算默认值
   * @param config 待归一化的配置（将被修改）
   */
  void normalizeDerivedValues(DEMConfig& config) const;

  /**
   * @brief 验证配置参数的合法性和约束条件
   * @param config 待验证的配置
   * @throw std::runtime_error 当配置不满足约束时抛出异常
   */
  void validate(const DEMConfig& config) const;
};

}  // namespace dem
