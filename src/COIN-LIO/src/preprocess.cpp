#include "preprocess.h"

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (lidar_type)
  {
  case OUSTER:
    ouster_handler(msg, pcl_out);
    break;

  case VELO16:
    velodyne_handler(msg, pcl_out);
    break;

  default:
    printf("LiDAR type not supported");
    break;
  }
}

void Preprocess::ouster_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pcl_out->reserve(plsize);
  pcl::uint64_t max_time = 0;
  for (size_t i = 0; i < pl_orig.points.size(); i++)
  {
    if(isnan(pl_orig.points[i].x) || isnan(pl_orig.points[i].y) || isnan(pl_orig.points[i].z)) continue;
    double range = pl_orig.points[i].getVector3fMap().norm();

    if (range < blind) continue;
    if (max_range > 0.0 && range > max_range) continue;   // 距离截断: 去掉太远(max_range)的点
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z - lidar_sensor_z_offset;

    if (reflectivity) {
      added_pt.intensity = pl_orig.points[i].reflectivity;
    } else {
      added_pt.intensity = pl_orig.points[i].intensity;
    }
    
    // to keep track of original point index
    added_pt.normal_x = i;
    added_pt.normal_y = range;
    added_pt.normal_z = 0;
    added_pt.curvature = pl_orig.points[i].t / 1e6; // curvature unit: ms

    pcl_out->points.push_back(added_pt);
  
    if (pl_orig.points[i].t > max_time) max_time = pl_orig.points[i].t;
  }
  pcl_out->header.stamp = max_time;
}

// Velodyne VLP-16: 与 ouster_handler 保持同样的输出约定
//   normal_x = 原始点索引 (Projector 建强度/距离图时用它做 LUT 索引, 必须保留)
//   normal_y = 距离 (作为 img_range 的值)
//   curvature = 相对"帧首"的时间 (ms, 用于去畸变)
//   header.stamp = 帧时长 (ns)  <- COIN-LIO 的 sync_packages 就是按 ns 时长用的
void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pcl_out->reserve(plsize);

  // Velodyne 的 time 是"相对本帧时间戳"的偏移(秒)。不同驱动可能以帧首或帧尾为基准
  // (你这个包里是帧尾, 取值约 -0.1~0 秒)。COIN-LIO 要求 curvature >= 0 且以帧首为基准,
  // 所以先求本帧时间范围, 再统一平移到帧首, 并把偏移量交给调用者去平移帧时间戳。
  double t_min = 0.0, t_max = 0.0;
  if (pl_orig.points.size() > 0) {
    t_min = pl_orig.points[0].time;
    t_max = pl_orig.points[0].time;
    for (size_t i = 1; i < pl_orig.points.size(); i++) {
      if (pl_orig.points[i].time < t_min) t_min = pl_orig.points[i].time;
      if (pl_orig.points[i].time > t_max) t_max = pl_orig.points[i].time;
    }
  }
  time_base_offset = t_min;

  for (size_t i = 0; i < pl_orig.points.size(); i++)
  {
    if (std::isnan(pl_orig.points[i].x) || std::isnan(pl_orig.points[i].y) || std::isnan(pl_orig.points[i].z)) continue;
    double range = pl_orig.points[i].getVector3fMap().norm();

    if (range < blind) continue;
    if (max_range > 0.0 && range > max_range) continue;   // 距离截断: 去掉太远(max_range)的点

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z - lidar_sensor_z_offset;
    added_pt.intensity = pl_orig.points[i].intensity;

    // to keep track of original point index
    added_pt.normal_x = i;
    added_pt.normal_y = range;
    added_pt.normal_z = 0;
    added_pt.curvature = (pl_orig.points[i].time - t_min) * 1000.0; // 秒 -> ms, 相对帧首

    pcl_out->points.push_back(added_pt);
  }
  pcl_out->header.stamp = static_cast<uint64_t>((t_max - t_min) * 1e9);  // 帧时长(ns)
}
