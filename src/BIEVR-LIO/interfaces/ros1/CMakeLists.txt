cmake_minimum_required(VERSION 3.16)
project(bievr_lio_ros LANGUAGES CXX)

# 用 CMake 的标准机制(而不是裸 -std)声明 C++ 标准: Ceres 会传递 cxx_std_17 这个
# compile feature, 裸写 -std=c++2a 会被 CMake 为该 feature 追加的 -std=gnu++17 覆盖。
#
# 本机是 GCC 10 + Boost 1.71: -std=gnu++20 下 ROS 的 tf(signals2)/rosbag(boost::format)
# 会报 "no type named 'pointer' in class std::allocator<...>"(C++20 删掉了
# std::allocator::pointer 与双参 allocate, Boost 1.71 仍在用)。GCC 10 的 -fconcepts
# 在 C++17 模式下同样接受 C++20 的 concept 语法, 所以这里用 C++17 + -fconcepts,
# 与上游 ROS1 侧(GCC 9 的 concepts-TS)保持一致。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fconcepts -fdiagnostics-color=always")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf
  tf_conversions
  rosbag
)
find_package(TBB REQUIRED)

# yaml-cpp: 本机装了两份 —— 系统的 0.6.2(/usr/lib/x86_64-linux-gnu, ROS Noetic 的 rosbag 就是链接它)
# 和 /usr/local 的 0.8.0。默认 find_package 会命中 0.8.0, 于是头文件是 0.8 的(而且 /usr/local/include
# 是编译器的隐式搜索目录, 改 -I 顺序也压不住), 但 ${YAML_CPP_LIBRARIES} 只是裸名字 "yaml-cpp",
# 链接器最终仍解析到系统的 0.6.2 —— 链接时就会报
#   undefined reference to `YAML::detail::node_data::empty_scalar[abi:cxx11]()'
# (0.6.2 里 empty_scalar 是个变量, 0.8 里才是函数)。
# 这里显式指定系统那份 config, 让头文件和库都锁定 0.6.2(与 rosbag 一致)。
set(_bievr_system_yaml_cpp_dir "/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp")
if(EXISTS "${_bievr_system_yaml_cpp_dir}/yaml-cpp-config.cmake")
  set(yaml-cpp_DIR "${_bievr_system_yaml_cpp_dir}" CACHE PATH
      "system yaml-cpp 0.6.2 (same one ROS Noetic rosbag links)" FORCE)
endif()
find_package(yaml-cpp REQUIRED)  # shared config_loader.h parses plain YAML files
# bievr_lio (core) and bievr_ros_common (shared header-only glue) are plain-CMake
# packages (not catkin), so they are not pulled in via ${catkin_LIBRARIES}; find
# and link their exported targets explicitly.
find_package(bievr_lio REQUIRED)
find_package(bievr_ros_common REQUIRED)

# Livox support is optional: the Livox CustomMsg branches are only compiled when
# the corresponding driver is on the system. Users without the drivers can still
# build and run with sensor_msgs/PointCloud2 inputs.
#   livox_ros_driver  (gen1) -> BIEVR_WITH_LIVOX
#   livox_ros_driver2 (gen2) -> BIEVR_WITH_LIVOX2
find_package(livox_ros_driver QUIET)
find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver_FOUND)
  message(STATUS "bievr_lio_ros: livox_ros_driver found, enabling Livox (gen1) support.")
else()
  message(STATUS "bievr_lio_ros: livox_ros_driver NOT found, building without Livox (gen1) support.")
endif()
if(livox_ros_driver2_FOUND)
  message(STATUS "bievr_lio_ros: livox_ros_driver2 found, enabling Livox (gen2) support.")
else()
  message(STATUS "bievr_lio_ros: livox_ros_driver2 NOT found, building without Livox (gen2) support.")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    nav_msgs
    tf
    tf_conversions
    rosbag
)

# compat_include/yaml-cpp 是指向系统 0.6.2 头文件的软链接, 必须放在最前面:
# /usr/include 是编译器的标准系统目录, 显式写 -I/usr/include 会被 GCC 直接忽略,
# 于是 <yaml-cpp/yaml.h> 会命中隐式搜索目录 /usr/local/include 里的 0.8 版本,
# 而链接到的库是 ROS 的 0.6.2 -> 报 empty_scalar[abi:cxx11]() 未定义。
set(_bievr_compat_include "${CMAKE_CURRENT_SOURCE_DIR}/../../compat_include")
include_directories(
  ${_bievr_compat_include}
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# The wrapper is now header-only apart from the two executables (conversions,
# publisher and config loading all live in headers shared via bievr_ros_common),
# so there is no library to build -- the executables link the dependencies
# directly.
add_executable(process_topics src/process_topics.cpp)
add_executable(process_bag src/process_bag.cpp)

foreach(target process_topics process_bag)
  # Ensure message headers from catkin dependencies exist first.
  add_dependencies(${target} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${target}
    ${catkin_LIBRARIES}
    bievr_lio::bievr_lio
    bievr_ros_common::bievr_ros_common
    ${YAML_CPP_LIBRARIES}
    TBB::tbb
  )
  # Optional Livox drivers: define the guard macros and pull in headers/libs
  # only when found (livox_ros_driver = gen1, livox_ros_driver2 = gen2).
  if(livox_ros_driver_FOUND)
    target_compile_definitions(${target} PRIVATE BIEVR_WITH_LIVOX)
    target_include_directories(${target} PRIVATE ${livox_ros_driver_INCLUDE_DIRS})
    target_link_libraries(${target} ${livox_ros_driver_LIBRARIES})
  endif()
  if(livox_ros_driver2_FOUND)
    target_compile_definitions(${target} PRIVATE BIEVR_WITH_LIVOX2)
    target_include_directories(${target} PRIVATE ${livox_ros_driver2_INCLUDE_DIRS})
    target_link_libraries(${target} ${livox_ros_driver2_LIBRARIES})
  endif()
endforeach()

install(TARGETS process_bag process_topics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
