cmake_minimum_required(VERSION 2.8.3)
project(da_lio)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++17 )
ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions  
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED timer) #GTSAM require
# GTSAM: 必须用 /usr/local 的 4.0.x(原因同 MC_PGO: ROS 自带的 4.2a9 API 不兼容, 且 GTSAM_DIR
# 全局 cache 已指向 /opt/ros/noetic)。直接指定头/库, 不改全局 cache, 不影响其它包。
set(GTSAM_INCLUDE_DIR "/usr/local/include")
set(GTSAM_LIBRARIES "/usr/local/lib/libgtsam.so")
# 原因同 MC_PGO: /usr/local/include 是编译器隐式搜索目录会被 CMake 从 -I 里剔除, 必须移出隐式列表,
# 才能在 include_directories 里生成 -I/usr/local/include 并抢在 /opt/ros/noetic/include(gtsam 4.2a9) 之前
find_package(Ceres REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

# compat_include 里是 gtsam/ceres 软链接(指向 /usr/local 的 4.0.x/2.1.0), 必须放最前,
# 否则 /opt/ros/noetic/include 里的 gtsam 4.2a9 会抢先命中(原因见 MC_PGO/CMakeLists.txt)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../compat_include
  ${GTSAM_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_library(basic_odom include/registration.cpp include/common_pt_operations.cpp include/io.cpp)
target_link_libraries(basic_odom ${catkin_LIBRARIES} ${GTSAM_LIBRARIES} tbb ${PCL_LIBRARIES} -lstdc++fs ${CERES_LIBRARIES} )

add_executable(da_lio src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
# laserMapping.cpp 里是 Ceres 自动求导 + basalt 连续时间样条的模板重灾区, -O3 下 cc1plus 峰值内存可达数 GB
# (本机 16GB 内存会直接被 OOM 杀掉)。放宽 GCC 的 GC 阈值可大幅降低编译内存, 几乎不影响生成代码质量。
set_source_files_properties(src/laserMapping.cpp PROPERTIES COMPILE_FLAGS "--param ggc-min-expand=10 --param ggc-min-heapsize=32768")
target_link_libraries(da_lio ${PYTHON_LIBRARIES} basic_odom)
target_include_directories(da_lio PRIVATE ${PYTHON_INCLUDE_DIRS})

# ---- 运行时锁定 GTSAM 版本 ----
# 头文件来自 /usr/local/include(GTSAM 4.0.0), 但 LD_LIBRARY_PATH 里 GTSAM 4.2.0 排在 /usr/local/lib 前面,
# 运行时会加载 4.2.0 的 libgtsam.so.4 → libbasic_odom.so 报 undefined symbol:
#   gtsam::NoiseModelFactor::print(std::string const&, KeyFormatter const&) const
# 用旧式 DT_RPATH(优先级高于 LD_LIBRARY_PATH)把 /usr/local/lib 钉在最前, 只影响本工程进程。
foreach(tgt da_lio basic_odom)
    target_link_options(${tgt} PRIVATE "-Wl,--disable-new-dtags")
    set_target_properties(${tgt} PROPERTIES
            BUILD_RPATH "/usr/local/lib"
            INSTALL_RPATH "/usr/local/lib"
            BUILD_WITH_INSTALL_RPATH TRUE)
endforeach()
