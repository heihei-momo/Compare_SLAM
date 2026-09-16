/// Basic data type used in SLAM, point cloud processing
/// Author: Weitong Wu, wwtgeomatics@gmail.com
/// Version 1.2
/// Create Time: 2022/5/23
/// Update Time: 2023/6/1

#ifndef HELMET_CALIB_TYPES_H
#define HELMET_CALIB_TYPES_H

// ---- GCC 10 / ROS Noetic 兼容补丁 ----
// 原项目基于 Ubuntu 18.04 + GCC 7, 依赖其它头文件间接引入标准库并隐式 using namespace std。
// 新编译器不再传递这些包含(io.cpp/imu_process.cpp 等用了未限定的 cout/string/vector/ifstream),
// 这里在公共基础头里显式补上。
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

#include <gtsam/geometry/Pose3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

enum LiDAR_TYPE{LIVOX = 1, Velo16, Oust64, HESAI}; //{1, 2, 3, 4}

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloud;
typedef pcl::PointCloud<PointXYZ>::Ptr PointCloudPtr;

typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointCloud<PointXYZI> IPointCloud;
typedef pcl::PointCloud<PointXYZI>::Ptr IPointCloudPtr;

typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointCloud<PointXYZRGB> RGBPointCloud;
typedef pcl::PointCloud<PointXYZRGB>::Ptr RGBPointCloudPtr;

struct PointXYZT {
    PCL_ADD_POINT4D
    double relative_time_ms;// relative time with frame start time
};

typedef pcl::PointCloud<PointXYZT> TPointCloud;
typedef pcl::PointCloud<PointXYZT>::Ptr TPointCloudPtr;

struct PointXYZIT {
    PCL_ADD_POINT4D
    int reflectivity;       // or intensity
    double relative_time_ms;// relative time with frame start time
};

typedef pcl::PointCloud<PointXYZIT> ITPointCloud;
typedef pcl::PointCloud<PointXYZIT>::Ptr ITPointCloudPtr;

struct LidarData {
    double start_time_s;
    ITPointCloud pointcloud;
};

struct ImuData {

    double timestamp_s;//Timestamp of the reading
    Eigen::Vector3d wm;// Gyroscope reading, angular velocity (rad/s)
    Eigen::Vector3d am;// Accelerometer reading, linear acceleration (m/s^2)
};

struct PoseData {
    double timestamp;
    gtsam::Pose3 pose;///local to world
};

struct CalibData {
    double scan_start_time;
    double scan_end_time;
    gtsam::Pose3 scan_start_pose; //lidar pose calculated by LO
    gtsam::Pose3 scan_end_pose;
    gtsam::Pose3 scan_start_pose_fused; // lidar pose fused by lidar constraint and imu constraint
    gtsam::Pose3 scan_end_pose_fused;
    vector<ImuData> imu_data;
    ITPointCloud raw_scan;
};

struct Submap {
    double submap_start_time;
    double submap_end_time;
    gtsam::Pose3 submap_pose;
    ITPointCloud raw_pointcloud; //raw pt in submap_start_time-end_time
    std::vector<PoseData> odom_poses;
    std::vector<ImuData> imu_data;
    PointCloud pointcloud_dt; //pt in submap_start_time coordinate using odom_poses and imu integration
    PointCloud pointcloud_ct; //pt in submap_start_time coordinate using continuous-time optimization
};

/*
namespace hesai_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        double time;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace hesai_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (double, time, time)
                                          (uint16_t, ring, ring)
)
*/
#endif//HELMET_CALIB_TYPES_H
