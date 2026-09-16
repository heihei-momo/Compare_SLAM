cmake_minimum_required(VERSION 3.0.2)
project(backend_mapping)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

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

find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED) # ikd-tree require
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
find_package(Boost REQUIRED timer) #GTSAM require
# GTSAM: 必须用 /usr/local 的 4.0.x。DALI-SLAM 按 GTSAM 4.0.3 API 编写, 而 /opt/ros/noetic 里的
# 4.2a9 把 Pose3::transform_from 改名成 transformFrom、并去掉了带雅可比的 Point3::dot。
# 又因为 GTSAM_DIR 是全局 cache(已被设成 /opt/ros/noetic), find_package 必定命中 4.2a9,
# 所以这里直接指定 4.0.x 的头文件与库(不动全局 cache, 因此不影响 LIO-SAM 等其它包)。
set(GTSAM_INCLUDE_DIR "/usr/local/include")
set(GTSAM_LIBRARIES "/usr/local/lib/libgtsam.so")
find_package(Ceres REQUIRED)
find_package(PDAL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        rosbag
        roscpp
        livox_ros_driver
        pcl_ros
        )

catkin_package()

# compat_include 里是 gtsam / ceres 的软链接, 指向 /usr/local 的 4.0.x / 2.1.0。
# 必须放在最前面: /usr/local/include 是编译器的隐式搜索目录, CMake 不会为它生成 -I,
# 结果 /opt/ros/noetic/include 里的 gtsam 4.2a9(API 不兼容) 会先被找到。
include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../compat_include
        include
        ${GTSAM_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

add_library(basic include/io.cpp include/ikd-Tree/ikd_Tree.cpp
        include/imu_process.cpp include/registration.cpp include/common_pt_operations.cpp)
target_link_libraries(basic ${catkin_LIBRARIES} ${GTSAM_LIBRARIES} tbb ${PCL_LIBRARIES} -lstdc++fs ${CERES_LIBRARIES} pdalcpp)

add_library(fast_gicp
        include/fast_gicp/lsq_registration.cpp
        include/fast_gicp/fast_gicp.cpp
        include/fast_gicp/fast_vgicp.cpp
        )
target_link_libraries(fast_gicp
        ${PCL_LIBRARIES})

add_executable(mc_pgo src/pose_graph_opti.cpp)
target_link_libraries(mc_pgo basic fast_gicp)

add_executable(pgo_base src/pose_graph_opti_base.cpp)
target_link_libraries(pgo_base basic)

add_executable(map_mme src/mean_map_entropy.cpp)
target_link_libraries(map_mme basic)

add_executable(test_fastgicp src/test_fastgicp.cpp)
target_link_libraries(test_fastgicp fast_gicp)

# ---- 运行时锁定 GTSAM 版本 ----
# 本工程按 GTSAM 4.0.x 的 API 编写(上游 Dockerfile 装的是 4.0.3), 头文件取自 /usr/local/include(GTSAM 4.0.0)。
# 但 LD_LIBRARY_PATH 里 /opt/ros/noetic/lib/x86_64-linux-gnu 的 GTSAM 4.2.0 排在 /usr/local/lib 之前,
# 运行时会加载 4.2.0 的 libgtsam.so.4, 于是 libbasic.so 报
#   undefined symbol: gtsam::NoiseModelFactor::print(std::string const&, KeyFormatter const&) const
# DT_RPATH 的搜索优先级高于 LD_LIBRARY_PATH(同名新式 DT_RUNPATH 则低于), 所以用
# --disable-new-dtags + rpath=/usr/local/lib 把 4.0.0 钉死; 该设置只影响本工程的进程, 不污染其它节点。
foreach(tgt mc_pgo pgo_base map_mme test_fastgicp basic fast_gicp)
    target_link_options(${tgt} PRIVATE "-Wl,--disable-new-dtags")
    set_target_properties(${tgt} PROPERTIES
            BUILD_RPATH "/usr/local/lib"
            INSTALL_RPATH "/usr/local/lib"
            BUILD_WITH_INSTALL_RPATH TRUE)
endforeach()