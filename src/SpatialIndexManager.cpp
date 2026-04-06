/**
 * @file SpatialIndexManager.cpp
 * @brief 空间索引管理器实现：基于 nanoflann 的 KD-Tree 2D/3D 索引构建与查询
 */

#include "dem/SpatialIndexManager.hpp"

#include <nanoflann.hpp>

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace dem {
namespace {

/**
 * @brief 点集适配器模板，将 std::vector<Point3D> 适配到 nanoflann 的 KDTree 接口
 *
 * @tparam Dim 维度参数（2 或 3），决定使用 X/Y 还是 X/Y/Z 坐标
 *
 * nanoflann 要求提供 kdtree_get_point_count、kdtree_get_pt 和 kdtree_get_bbox
 * 三个接口方法。本适配器通过 active_indices 支持对点集子集建立索引。
 */
template <std::size_t Dim>
struct PointSetAdaptor {
  const std::vector<Point3D>* points = nullptr;       /**< 原始点集的指针 */
  std::vector<std::size_t> active_indices;             /**< 需要纳入索引的点索引列表 */

  /** @return 当前纳入索引的有效点数量 */
  [[nodiscard]] std::size_t kdtree_get_point_count() const { return active_indices.size(); }

  /**
   * @brief 获取指定点的指定维度坐标值
   * @param idx 局部索引（在 active_indices 中的位置）
   * @param dim 维度编号（0=X, 1=Y, 2=Z）
   * @return 坐标值
   */
  [[nodiscard]] double kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
    const auto& point = (*points)[active_indices[idx]];
    if constexpr (Dim == 2) {
      return dim == 0 ? point.x : point.y;
    } else {
      if (dim == 0) { return point.x; }
      if (dim == 1) { return point.y; }
      return point.z;
    }
  }

  /** 不提供边界框信息（返回 false），让 KD-Tree 自行计算 */
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};

/** 基于 L2 距离的 KD-Tree 类型别名 */
template <std::size_t Dim>
using KDTree =
    nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointSetAdaptor<Dim>>, PointSetAdaptor<Dim>, Dim, std::size_t>;

}  // namespace

/** PImpl 实现结构体，持有 2D 和 3D 两套适配器和树实例 */
struct SpatialIndexManager::Impl {
  PointSetAdaptor<2> adaptor2d;                        /**< 2D 适配器（X-Y 平面） */
  PointSetAdaptor<3> adaptor3d;                        /**< 3D 适配器（X-Y-Z 空间） */
  std::unique_ptr<KDTree<2>> tree2d;                  /**< 2D KD-Tree 实例 */
  std::unique_ptr<KDTree<3>> tree3d;                  /**< 3D KD-Tree 实例 */
};

SpatialIndexManager::SpatialIndexManager() : impl_(std::make_unique<Impl>()) {}
SpatialIndexManager::~SpatialIndexManager() = default;

/**
 * @brief 使用全部点的 X-Y 坐标构建 2D KD-Tree 索引
 * 叶节点大小设为 16，在查询效率和内存占用之间取得平衡。
 */
void SpatialIndexManager::build2D(const std::vector<Point3D>& points, const std::vector<std::size_t>& indices) {
  impl_->adaptor2d.points = &points;
  impl_->adaptor2d.active_indices = indices;
  if (indices.empty()) {
    impl_->tree2d.reset();
    return;
  }
  impl_->tree2d = std::make_unique<KDTree<2>>(2, impl_->adaptor2d, nanoflann::KDTreeSingleIndexAdaptorParams(16));
  impl_->tree2d->buildIndex();
}

/**
 * @brief 使用全部点的 X-Y-Z 坐标构建 3D KD-Tree 索引
 */
void SpatialIndexManager::build3D(const std::vector<Point3D>& points, const std::vector<std::size_t>& indices) {
  impl_->adaptor3d.points = &points;
  impl_->adaptor3d.active_indices = indices;
  if (indices.empty()) {
    impl_->tree3d.reset();
    return;
  }
  impl_->tree3d = std::make_unique<KDTree<3>>(3, impl_->adaptor3d, nanoflann::KDTreeSingleIndexAdaptorParams(16));
  impl_->tree3d->buildIndex();
}

/**
 * @brief 在 2D 索引上执行 K 近邻查询
 * 返回结果中的距离已从平方距离转换为欧氏距离。
 */
QueryResult SpatialIndexManager::knn2D(double x, double y, std::size_t k) const {
  if (!impl_->tree2d || k == 0) {
    return {};
  }
  /* 限制 K 值不超过实际点数 */
  const std::size_t actual_k = std::min(k, impl_->adaptor2d.active_indices.size());
  std::vector<std::size_t> local_indices(actual_k);
  std::vector<double> squared_distances(actual_k);
  const double query[2] = {x, y};
  impl_->tree2d->knnSearch(query, actual_k, local_indices.data(), squared_distances.data());

  /* 将局部索引映射回原始点集中的全局索引，并开方得到真实距离 */
  QueryResult result;
  result.indices.reserve(actual_k);
  result.distances.reserve(actual_k);
  for (std::size_t i = 0; i < actual_k; ++i) {
    result.indices.push_back(impl_->adaptor2d.active_indices[local_indices[i]]);
    result.distances.push_back(std::sqrt(squared_distances[i]));
  }
  return result;
}

/**
 * @brief 在 3D 索引上执行 K 近邻查询
 */
QueryResult SpatialIndexManager::knn3D(double x, double y, double z, std::size_t k) const {
  if (!impl_->tree3d || k == 0) {
    return {};
  }
  const std::size_t actual_k = std::min(k, impl_->adaptor3d.active_indices.size());
  std::vector<std::size_t> local_indices(actual_k);
  std::vector<double> squared_distances(actual_k);
  const double query[3] = {x, y, z};
  impl_->tree3d->knnSearch(query, actual_k, local_indices.data(), squared_distances.data());

  QueryResult result;
  result.indices.reserve(actual_k);
  result.distances.reserve(actual_k);
  for (std::size_t i = 0; i < actual_k; ++i) {
    result.indices.push_back(impl_->adaptor3d.active_indices[local_indices[i]]);
    result.distances.push_back(std::sqrt(squared_distances[i]));
  }
  return result;
}

/**
 * @brief 在 2D 索引上执行半径搜索
 * 返回按距离升序排列的结果，可通过 max_results 限制返回数量。
 */
QueryResult SpatialIndexManager::radius2D(double x, double y, double radius, std::size_t max_results) const {
  if (!impl_->tree2d || radius <= 0.0) {
    return {};
  }
  const double query[2] = {x, y};
  std::vector<nanoflann::ResultItem<std::size_t, double>> matches;
  nanoflann::SearchParameters params;
  params.sorted = true;
  impl_->tree2d->radiusSearch(query, radius * radius, matches, params);

  /* 截断超出最大返回数的尾部结果 */
  if (max_results > 0 && matches.size() > max_results) {
    matches.resize(max_results);
  }

  QueryResult result;
  result.indices.reserve(matches.size());
  result.distances.reserve(matches.size());
  for (const auto& match : matches) {
    result.indices.push_back(impl_->adaptor2d.active_indices[match.first]);
    result.distances.push_back(std::sqrt(match.second));
  }
  return result;
}

/**
 * @brief 在 3D 索引上执行半径搜索
 */
QueryResult SpatialIndexManager::radius3D(double x, double y, double z, double radius, std::size_t max_results) const {
  if (!impl_->tree3d || radius <= 0.0) {
    return {};
  }
  const double query[3] = {x, y, z};
  std::vector<nanoflann::ResultItem<std::size_t, double>> matches;
  nanoflann::SearchParameters params;
  params.sorted = true;
  impl_->tree3d->radiusSearch(query, radius * radius, matches, params);

  if (max_results > 0 && matches.size() > max_results) {
    matches.resize(max_results);
  }

  QueryResult result;
  result.indices.reserve(matches.size());
  result.distances.reserve(matches.size());
  for (const auto& match : matches) {
    result.indices.push_back(impl_->adaptor3d.active_indices[match.first]);
    result.distances.push_back(std::sqrt(match.second));
  }
  return result;
}

bool SpatialIndexManager::has2D() const noexcept { return static_cast<bool>(impl_->tree2d); }
bool SpatialIndexManager::has3D() const noexcept { return static_cast<bool>(impl_->tree3d); }

}  // namespace dem
