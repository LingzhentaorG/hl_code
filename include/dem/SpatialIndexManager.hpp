#pragma once

#include "dem/Types.hpp"

#include <memory>
#include <vector>

namespace dem {

/**
 * @brief 空间索引管理器，基于 nanoflann 库封装 KD-Tree 索引
 *
 * 提供高效的 2D 和 3D 空间查询能力，包括：
 * - K 近邻搜索（KNN）
 * - 半径搜索（Radius Search）
 *
 * 是地面滤波和 DEM 插值模块的核心依赖组件。
 */
class SpatialIndexManager {
 public:
  /** 默认构造函数 */
  SpatialIndexManager();
  /** 析构函数 */
  ~SpatialIndexManager();

  /**
   * @brief 基于 2D（X-Y 平面）坐标构建 KD-Tree 索引
   * @param points 全部点集合的引用
   * @param indices 需要纳入索引的点索引列表
   */
  void build2D(const std::vector<Point3D>& points, const std::vector<std::size_t>& indices);

  /**
   * @brief 基于 3D（X-Y-Z 空间）坐标构建 KD-Tree 索引
   * @param points 全部点集合的引用
   * @param indices 需要纳入索引的点索引列表
   */
  void build3D(const std::vector<Point3D>& points, const std::vector<std::size_t>& indices);

  /**
   * @brief 在 2D 索引上执行 K 近邻查询
   * @param x 查询点 X 坐标
   * @param y 查询点 Y 坐标
   * @param k 返回的近邻数量
   * @return 查询结果（包含点索引和欧氏距离）
   */
  QueryResult knn2D(double x, double y, std::size_t k) const;

  /**
   * @brief 在 3D 索引上执行 K 近邻查询
   * @param x 查询点 X 坐标
   * @param y 查询点 Y 坐标
   * @param z 查询点 Z 坐标
   * @param k 返回的近邻数量
   * @return 查询结果（包含点索引和欧氏距离）
   */
  QueryResult knn3D(double x, double y, double z, std::size_t k) const;

  /**
   * @brief 在 2D 索引上执行半径搜索
   * @param x 查询圆心 X 坐标
   * @param y 查询圆心 Y 坐标
   * @param radius 搜索半径
   * @param max_results 最大返回结果数限制
   * @return 查询结果（按距离升序排列）
   */
  QueryResult radius2D(double x, double y, double radius, std::size_t max_results) const;

  /**
   * @brief 在 3D 索引上执行半径搜索
   * @param x 查询球心 X 坐标
   * @param y 查询球心 Y 坐标
   * @param z 查询球心 Z 坐标
   * @param radius 搜索半径
   * @param max_results 最大返回结果数限制
   * @return 查询结果（按距离升序排列）
   */
  QueryResult radius3D(double x, double y, double z, double radius, std::size_t max_results) const;

  /** @return 当前是否已构建 2D 索引 */
  [[nodiscard]] bool has2D() const noexcept;
  /** @return 当前是否已构建 3D 索引 */
  [[nodiscard]] bool has3D() const noexcept;

 private:
  struct Impl;                          /**< 实现细节的前向声明（PImpl 模式） */
  std::unique_ptr<Impl> impl_;          /**< 实现对象的唯一指针 */
};

}  // namespace dem
