cmake_minimum_required(VERSION 3.16)
project(bievr_ros_common VERSION 0.0.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Header-only library with the ROS-agnostic glue shared by the ROS1 and ROS2
# wrappers, kept out of the core `bievr_lio` library so its include tree stays
# free of any wrapper/interface code:
#   - config_loader.h     : parses the shared YAML config into the core `Config`
#   - conversions_impl.h  : templated ROS msg <-> LIO type conversions
#
# yaml-cpp (needed by config_loader.h) and the ROS message types (needed by
# conversions_impl.h) are supplied by the consumer, exactly as before: this
# library only carries the headers and the dependency on the core library.
# ---------------------------------------------------------------------------
find_package(bievr_lio REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# bievr_lio publicly carries Eigen/Ceres/TBB, so consumers of this library get
# everything the headers need from the core (TBB for conversions_impl.h, etc.).
target_link_libraries(${PROJECT_NAME} INTERFACE bievr_lio::bievr_lio)

# 消费方(ROS1/ROS2 接口)统一用 C++17 + -fconcepts, 原因见 BIEVR/CMakeLists.txt
# (GCC 10 + Boost 1.71 在 C++20 模式下编不过 ROS 的 tf/rosbag 头)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)

# ---------------------------------------------------------------------------
# Install & export
# ---------------------------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

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
