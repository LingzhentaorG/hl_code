/**
 * @file CMakeLists.txt
 * @brief DEM 生成器项目构建系统配置文件
 *
 * 构建依赖：
 * - C++20 标准（强制要求）
 * - nlohmann/json: JSON 配置文件解析（通过 FetchContent 自动下载）
 * - nanoflann: 高性能 KD-Tree 空间索引库（通过 FetchContent 自动下载）
 * - doctest: 单元测试框架（通过 FetchContent 自动下载，可选构建）
 * - zlib: PNG 输出时的数据压缩（Windows 系统自带，Linux 需安装 libz-dev）
 * - Python3: GeoTIFF 后端写入支持（必需）
 * - TIFF/PROJ/libgeotiff: 原生 GeoTIFF 后端（可选，需 pkg-config 检测）
 *
 * 使用方法：
 *   mkdir build && cd build
 *   cmake .. -G "MinGW Makefiles"
 *   mingw32-make -j$(nproc)
 */

cmake_minimum_required(VERSION 3.24)

/* 兼容旧版 CMake 的链接目录行为策略 */
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

/* 项目基本信息：名称、版本号、开发语言 */
project(dem_cli VERSION 0.1.0 LANGUAGES CXX)

/* 强制使用 C++20 标准并禁用编译器扩展 */
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

/* ======================== 可选功能开关 ======================== */
option(DEM_ENABLE_OPENMP "Enable OpenMP acceleration when available" ON)     /* OpenMP 并行加速 */
option(DEM_ENABLE_TESTS "Build unit and integration tests" ON)               /* 单元测试构建 */
set(DEM_GTIFF_BACKEND "AUTO" CACHE STRING "GeoTIFF backend: AUTO, PYTHON, or NATIVE")  /* GeoTIFF 后端选择 */
set_property(CACHE DEM_GTIFF_BACKEND PROPERTY STRINGS AUTO PYTHON NATIVE)

/* ======================== 第三方依赖管理 ======================== */
include(FetchContent)

/* 禁用子项目的测试/示例/基准构建以加速下载和编译 */
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)

/* 声明并通过 Git 自动获取 nlohmann_json (v3.11.3) */
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)

/* 声明并通过 Git 自动获取 doctest 测试框架 (v2.4.12) */
FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.4.12
)

/* 声明并通过 Git 自动获取 nanoflann KD-Tree 库 (v1.5.5) */
FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG v1.5.5
)

/* 使 JSON 和 doctest 可用（nanoflann 需要手动处理 populate） */
FetchContent_MakeAvailable(nlohmann_json doctest)
FetchContent_GetProperties(nanoflann)
if(NOT nanoflann_POPULATED)
  FetchContent_Populate(nanoflann)
endif()

/* ======================== Python 解释器检测 ======================== */
find_package(Python3 COMPONENTS Interpreter REQUIRED)

/* ======================== GeoTIFF 后端选择逻辑 ======================== */
set(DEM_USE_NATIVE_GTIFF OFF)
if(DEM_GTIFF_BACKEND STREQUAL "NATIVE" OR DEM_GTIFF_BACKEND STREQUAL "AUTO")
  /* 尝试查找原生 GeoTIFF 所需的系统库 */
  find_package(TIFF QUIET)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(PROJ QUIET IMPORTED_TARGET proj)              /* PROJ 坐标投影库 */
    pkg_check_modules(GEOTIFF QUIET IMPORTED_TARGET libgeotiff)    /* GeoTIFF 地理标签库 */
  endif()
  /* 仅当所有原生库均可用时启用原生后端 */
  if(TIFF_FOUND AND TARGET PkgConfig::PROJ AND TARGET PkgConfig::GEOTIFF)
    set(DEM_USE_NATIVE_GTIFF ON)
  elseif(DEM_GTIFF_BACKEND STREQUAL "NATIVE")
    message(FATAL_ERROR "Native GeoTIFF backend requested, but TIFF/PROJ/libgeotiff were not found.")
  endif()
endif()

/* ======================== 源文件列表定义 ======================== */
set(DEM_HEADERS
  include/dem/Config.hpp
  include/dem/CRSManager.hpp
  include/dem/DEMEngine.hpp
  include/dem/GroundFilterEngine.hpp
  include/dem/InputManager.hpp
  include/dem/Logger.hpp
  include/dem/MainController.hpp
  include/dem/OutputManager.hpp
  include/dem/PreprocessEngine.hpp
  include/dem/SpatialIndexManager.hpp
  include/dem/TileManager.hpp
  include/dem/Types.hpp
  include/dem/Utils.hpp
)

set(DEM_SOURCES
  src/Config.cpp
  src/CRSManager.cpp
  src/DEMEngine.cpp
  src/GroundFilterEngine.cpp
  src/InputManager.cpp
  src/Logger.cpp
  src/MainController.cpp
  src/OutputManager.cpp
  src/PreprocessEngine.cpp
  src/SpatialIndexManager.cpp
  src/TileManager.cpp
  src/Utils.cpp
)

/* ======================== 核心库目标构建 ======================== */
add_library(dem_core ${DEM_HEADERS} ${DEM_SOURCES})

/* 设置头文件搜索路径：项目 include 目录 + nanoflann 头文件目录 */
target_include_directories(
  dem_core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${nanoflann_SOURCE_DIR}/include
)

/* 链接 JSON 解析库 */
target_link_libraries(dem_core PUBLIC nlohmann_json::nlohmann_json)

/* 向代码注入源码目录路径和 Python 可执行文件路径（供 main.cpp 使用） */
target_compile_definitions(
  dem_core
  PUBLIC
    DEM_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    DEM_PYTHON_EXECUTABLE="${Python3_EXECUTABLE}"
)

/* OpenMP 并行加速支持（可选） */
if(DEM_ENABLE_OPENMP)
  find_package(OpenMP QUIET)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(dem_core PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(dem_core PUBLIC DEM_HAS_OPENMP=1)
  endif()
endif()

/* 根据 GeoTIFF 后端选择结果设置编译宏和链接库 */
if(DEM_USE_NATIVE_GTIFF)
  target_link_libraries(dem_core PRIVATE TIFF::TIFF PkgConfig::PROJ PkgConfig::GEOTIFF)
  target_compile_definitions(dem_core PUBLIC DEM_USE_NATIVE_GTIFF=1)
else()
  target_compile_definitions(dem_core PUBLIC DEM_USE_PYTHON_GTIFF=1)
endif()

/* ======================== 可执行文件目标构建 ======================== */
add_executable(dem_cli src/main.cpp)
target_link_libraries(dem_cli PRIVATE dem_core)

/* ======================== 编译警告选项 ======================== */
if(MSVC)
  /* MSVC 编译器：Level 4 警告级别 */
  target_compile_options(dem_core PRIVATE /W4)
  target_compile_options(dem_cli PRIVATE /W4)
else()
  /* GCC/Clang 编译器：Wall + Wextra + Wpedantic 全量警告 */
  target_compile_options(dem_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(dem_cli PRIVATE -Wall -Wextra -Wpedantic)
endif()

/* ======================== 单元测试目标构建 ======================== */
if(DEM_ENABLE_TESTS)
  enable_testing()
  add_executable(
    dem_tests
    tests/test_main.cpp      /* 测试入口（含 doctest 主函数） */
    tests/test_config.cpp    /* 配置加载与归一化测试 */
    tests/test_pipeline.cpp  /* 端到端流水线集成测试 */
  )
  target_link_libraries(dem_tests PRIVATE dem_core doctest::doctest)
  add_test(NAME dem_tests COMMAND dem_tests)
endif()
