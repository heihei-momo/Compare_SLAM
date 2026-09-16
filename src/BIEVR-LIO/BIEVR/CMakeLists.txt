cmake_minimum_required(VERSION 3.16)
project(bievr_lio VERSION 0.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Build configuration
# ---------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Build a shared library by default (matches the previous catkin behaviour).
option(BUILD_SHARED_LIBS "Build bievr_lio as a shared library" ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Dependencies
#
# Ceres / glog / Eigen are currently provided by the *_catkin source packages
# in the workspace devel/install space, so they are found via CMAKE_PREFIX_PATH
# when the workspace is sourced. TBB is found system-wide.
# ---------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(TBB REQUIRED)
find_package(glog QUIET)  # usually re-exported transitively by Ceres

# ---------------------------------------------------------------------------
# Library target
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME}
  src/bievr_map.cpp
  src/common.cpp
  src/pipeline.cpp
  src/imu_integrator.cpp
  src/inertial_factor.cpp
  src/ls_optimizer.cpp
  src/preprocess.cpp
  src/prior_factor.cpp
  src/synchronizer.cpp
  src/timing.cpp
  src/undistort.cpp
  src/utils.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# 本方法源码目录(src/BIEVR-LIO/), 供 yaml 里的相对输出路径(如 Robotpose/xxx.csv)解析用
target_compile_definitions(${PROJECT_NAME} PRIVATE BIEVR_PKG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 上游用 C++20 + concepts; 但本机是 GCC 10(libstdc++ 10) + Boost 1.71, 在 -std=gnu++20 下
# boost/signals2、boost/format 会报 "no type named 'pointer' in class std::allocator<...>"
# (C++20 删掉了 std::allocator::pointer / 双参 allocate, 而 Boost 1.71 还在用)。
# GCC 10 的 -fconcepts 在 C++17 模式下也接受 C++20 的 concept 语法, 所以统一降到 C++17 + -fconcepts,
# 这也是上游 ROS1 侧(它们用 GCC 9 的 concepts-TS)实际走的路径。
set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:GNU>:-fconcepts>
  $<$<CONFIG:Release>:-O3>
)

# Route all logging through log++ (include/bievr_lio/log++.h) in LPP mode. PUBLIC
# so consumers that include our headers compile log++ in the same mode.
target_compile_definitions(${PROJECT_NAME} PUBLIC MODE_LPP)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  Ceres::ceres
  TBB::tbb
)
if(glog_FOUND)
  target_link_libraries(${PROJECT_NAME} PUBLIC glog::glog)
endif()

# ---------------------------------------------------------------------------
# Install & export
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

set(BIEVR_LIO_HAS_GLOG ${glog_FOUND})
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
