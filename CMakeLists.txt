cmake_minimum_required(VERSION 3.24)

if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

project(dem_cli VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(DEM_ENABLE_OPENMP "Enable OpenMP acceleration when available" ON)
option(DEM_ENABLE_TESTS "Build unit and integration tests" ON)
set(DEM_GTIFF_BACKEND "AUTO" CACHE STRING "GeoTIFF backend: AUTO, PYTHON, or NATIVE")
set_property(CACHE DEM_GTIFF_BACKEND PROPERTY STRINGS AUTO PYTHON NATIVE)

include(FetchContent)

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(NANOFLANN_BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3
)

FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.4.12
)

FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG v1.5.5
)

FetchContent_MakeAvailable(nlohmann_json doctest)
FetchContent_GetProperties(nanoflann)
if(NOT nanoflann_POPULATED)
  FetchContent_Populate(nanoflann)
endif()

find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(DEM_USE_NATIVE_GTIFF OFF)
if(DEM_GTIFF_BACKEND STREQUAL "NATIVE" OR DEM_GTIFF_BACKEND STREQUAL "AUTO")
  find_package(TIFF QUIET)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(PROJ QUIET IMPORTED_TARGET proj)
    pkg_check_modules(GEOTIFF QUIET IMPORTED_TARGET libgeotiff)
  endif()
  if(TIFF_FOUND AND TARGET PkgConfig::PROJ AND TARGET PkgConfig::GEOTIFF)
    set(DEM_USE_NATIVE_GTIFF ON)
  elseif(DEM_GTIFF_BACKEND STREQUAL "NATIVE")
    message(FATAL_ERROR "Native GeoTIFF backend requested, but TIFF/PROJ/libgeotiff were not found.")
  endif()
endif()

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

add_library(dem_core ${DEM_HEADERS} ${DEM_SOURCES})

target_include_directories(
  dem_core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${nanoflann_SOURCE_DIR}/include
)

target_link_libraries(dem_core PUBLIC nlohmann_json::nlohmann_json)

target_compile_definitions(
  dem_core
  PUBLIC
    DEM_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    DEM_PYTHON_EXECUTABLE="${Python3_EXECUTABLE}"
)

if(DEM_ENABLE_OPENMP)
  find_package(OpenMP QUIET)
  if(OpenMP_CXX_FOUND)
    target_link_libraries(dem_core PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(dem_core PUBLIC DEM_HAS_OPENMP=1)
  endif()
endif()

if(DEM_USE_NATIVE_GTIFF)
  target_link_libraries(dem_core PRIVATE TIFF::TIFF PkgConfig::PROJ PkgConfig::GEOTIFF)
  target_compile_definitions(dem_core PUBLIC DEM_USE_NATIVE_GTIFF=1)
else()
  target_compile_definitions(dem_core PUBLIC DEM_USE_PYTHON_GTIFF=1)
endif()

add_executable(dem_cli src/main.cpp)
target_link_libraries(dem_cli PRIVATE dem_core)

if(MSVC)
  target_compile_options(dem_core PRIVATE /W4)
  target_compile_options(dem_cli PRIVATE /W4)
else()
  target_compile_options(dem_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(dem_cli PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(DEM_ENABLE_TESTS)
  enable_testing()
  add_executable(
    dem_tests
    tests/test_main.cpp
    tests/test_config.cpp
    tests/test_pipeline.cpp
  )
  target_link_libraries(dem_tests PRIVATE dem_core doctest::doctest)
  add_test(NAME dem_tests COMMAND dem_tests)
endif()
