// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill Stachniss.
// Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Eigen/Core>
#include <memory>
#include <utility>
#include <vector>

// GenZ-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// GenZ-ICP
#include "genz_icp/pipeline/GenZICP.hpp"

// ROS 1 headers
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace genz_icp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

OdometryServer::OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), tf2_listener_(tf2_ros::TransformListener(tf2_buffer_)) {
    pnh_.param("base_frame", base_frame_, base_frame_);
    pnh_.param("odom_frame", odom_frame_, odom_frame_);
    pnh_.param("publish_odom_tf", publish_odom_tf_, false);
    pnh_.param("visualize", publish_debug_clouds_, publish_debug_clouds_);
    pnh_.param("max_range", config_.max_range, config_.max_range);
    pnh_.param("min_range", config_.min_range, config_.min_range);
    pnh_.param("deskew", config_.deskew, config_.deskew);
    pnh_.param("voxel_size", config_.voxel_size, config_.max_range / 100.0);
    pnh_.param("map_cleanup_radius", config_.map_cleanup_radius, config_.max_range);
    pnh_.param("planarity_threshold", config_.planarity_threshold, config_.planarity_threshold);
    pnh_.param("max_points_per_voxel", config_.max_points_per_voxel, config_.max_points_per_voxel);
    pnh_.param("desired_num_voxelized_points", config_.desired_num_voxelized_points, config_.desired_num_voxelized_points);
    pnh_.param("initial_threshold", config_.initial_threshold, config_.initial_threshold);
    pnh_.param("min_motion_th", config_.min_motion_th, config_.min_motion_th);
    pnh_.param("max_num_iterations", config_.max_num_iterations, config_.max_num_iterations);
    pnh_.param("convergence_criterion", config_.convergence_criterion, config_.convergence_criterion);
    pnh_.param("robotpose/csv_en", robot_pose_csv_en_, true);
#ifdef ROOT_DIR
    // config 里没写 robotpose/csv_file 时的默认值: 包目录上一级的 Robotpose/robot_pose.csv
    const std::string default_csv_file = std::string(ROOT_DIR) + "../Robotpose/robot_pose.csv";
#else
    const std::string default_csv_file = "robot_pose.csv";
#endif
    pnh_.param<std::string>("robotpose/csv_file", robot_pose_csv_file_, default_csv_file);

    // 点云话题: 优先取私有命名空间的 topic(即 yaml 里的 topic), 其次全局同名参数; 都为空则保持默认
    std::string topic_param;
    if (!pnh_.getParam("topic", topic_param)) nh_.getParam("topic", topic_param);
    if (!topic_param.empty()) pointcloud_topic_ = topic_param;

    // 中文用 cout 打印(rosconsole 会把非 ASCII 变成 ?)
    std::cout << "[GenZ-ICP] 点云话题: " << pointcloud_topic_ << std::endl;
    std::cout << "[GenZ-ICP] 输入点云距离截断: min_range=" << config_.min_range
              << " m, max_range=" << config_.max_range
              << " m (核心 Preprocess 按点到原点距离裁剪)" << std::endl;
    if (config_.max_range < config_.min_range) {
        ROS_WARN("[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
        config_.min_range = 0.0;
    }

    // Construct the main GenZ-ICP odometry node
    odometry_ = genz_icp::pipeline::GenZICP(config_);

    // Initialize subscribers
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic_, queue_size_,
                                                              &OdometryServer::RegisterFrame, this);

    // Initialize publishers
    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("/genz/odometry", queue_size_);
    traj_publisher_ = pnh_.advertise<nav_msgs::Path>("/genz/trajectory", queue_size_);
    if (publish_debug_clouds_) {
        map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/local_map", queue_size_);
        planar_points_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/planar_points", queue_size_);
        non_planar_points_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("/genz/non_planar_points", queue_size_);
    }
    // Initialize the transform buffer
    tf2_buffer_.setUsingDedicatedThread(true);
    path_msg_.header.frame_id = odom_frame_;

    // publish odometry msg
    ROS_INFO("GenZ-ICP ROS 1 Odometry Node Initialized");
}

Sophus::SE3d OdometryServer::LookupTransform(const std::string &target_frame,
                                             const std::string &source_frame) const {
    std::string err_msg;
    if (tf2_buffer_._frameExists(source_frame) &&  //
        tf2_buffer_._frameExists(target_frame) &&  //
        tf2_buffer_.canTransform(target_frame, source_frame, ros::Time(0), &err_msg)) {
        try {
            auto tf = tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
            return tf2::transformToSophus(tf);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }
    ROS_WARN("Failed to find tf between %s and %s. Reason=%s", target_frame.c_str(),
             source_frame.c_str(), err_msg.c_str());
    return {};
}

void OdometryServer::RegisterFrame(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    const auto cloud_frame_id = msg->header.frame_id;
    const double t_frame_start = ros::WallTime::now().toSec();   // 本帧处理计时起点
    const auto points = PointCloud2ToEigen(msg);
    const auto timestamps = [&]() -> std::vector<double> {
        if (!config_.deskew) return {};
        return GetTimestamps(msg);
    }();
    const auto egocentric_estimation = (base_frame_.empty() || base_frame_ == cloud_frame_id);

    // Register frame, main entry point to GenZ-ICP pipeline
    const auto &[planar_points, non_planar_points] = odometry_.RegisterFrame(points, timestamps);

    // Compute the pose using GenZ, ego-centric to the LiDAR
    const Sophus::SE3d genz_pose = odometry_.poses().back();

    // If necessary, transform the ego-centric pose to the specified base_link/base_footprint frame
    const auto pose = [&]() -> Sophus::SE3d {
        if (egocentric_estimation) return genz_pose;
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        return cloud2base * genz_pose * cloud2base.inverse();
    }();

    // 统计与位姿记录(每帧)
    frame_time_sum_ += ros::WallTime::now().toSec() - t_frame_start;
    ++processed_frame_num_;
    planar_num_sum_     += static_cast<long long>(planar_points.size());      // 平面点 = 点-面 ICP 的有效观测
    non_planar_num_sum_ += static_cast<long long>(non_planar_points.size());

    {
        PoseRecord rec;
        const Eigen::Vector3d t = pose.translation();
        const Eigen::Quaterniond q = pose.unit_quaternion().normalized();
        rec.t  = msg->header.stamp.toSec();
        rec.px = t(0); rec.py = t(1); rec.pz = t(2);
        rec.qx = q.x(); rec.qy = q.y(); rec.qz = q.z(); rec.qw = q.w();
        const double sinr = 2.0 * (q.w() * q.x() + q.y() * q.z());
        const double cosr = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
        const double siny = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cosy = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        rec.roll  = std::atan2(sinr, cosr) * 180.0 / M_PI;
        rec.pitch = std::asin(sinp > 1.0 ? 1.0 : (sinp < -1.0 ? -1.0 : sinp)) * 180.0 / M_PI;
        rec.yaw   = std::atan2(siny, cosy) * 180.0 / M_PI;
        pose_records_.push_back(rec);
    }

    // Spit the current estimated pose to ROS msgs
    PublishOdometry(pose, msg->header.stamp, cloud_frame_id);

    // Publishing this clouds is a bit costly, so do it only if we are debugging
    if (publish_debug_clouds_) {
        PublishClouds(msg->header.stamp, cloud_frame_id, planar_points, non_planar_points);
    }
}

void OdometryServer::PublishOdometry(const Sophus::SE3d &pose,
                                     const ros::Time &stamp,
                                     const std::string &cloud_frame_id) {
    // Header for point clouds and stuff seen from desired odom_frame

    // Broadcast the tf
    if (publish_odom_tf_) {
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = odom_frame_;
        transform_msg.child_frame_id = base_frame_.empty() ? cloud_frame_id : base_frame_;
        transform_msg.transform = tf2::sophusToTransform(pose);
        tf_broadcaster_.sendTransform(transform_msg);
    }

    // publish trajectory msg
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = odom_frame_;
    pose_msg.pose = tf2::sophusToPose(pose);
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_.publish(path_msg_);

    // publish odometry msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.pose.pose = tf2::sophusToPose(pose);
    odom_publisher_.publish(odom_msg);
}

void OdometryServer::PublishClouds(const ros::Time &stamp,
                                   const std::string &cloud_frame_id,
                                   const std::vector<Eigen::Vector3d> &planar_points,
                                   const std::vector<Eigen::Vector3d> &non_planar_points) {
    std_msgs::Header odom_header;
    odom_header.stamp = stamp;
    odom_header.frame_id = odom_frame_;

    // Publish map
    const auto genz_map = odometry_.LocalMap();

    if (!publish_odom_tf_) {
        // debugging happens in an egocentric world
        std_msgs::Header cloud_header;
        cloud_header.stamp = stamp;
        cloud_header.frame_id = cloud_frame_id;

        map_publisher_.publish(*EigenToPointCloud2(genz_map, odom_header));
        planar_points_publisher_.publish(*EigenToPointCloud2(planar_points, cloud_header));
        non_planar_points_publisher_.publish(*EigenToPointCloud2(non_planar_points, cloud_header));

        return;
    }

    // If transmitting to tf tree we know where the clouds are exactly
    const auto cloud2odom = LookupTransform(odom_frame_, cloud_frame_id);
    planar_points_publisher_.publish(*EigenToPointCloud2(planar_points, odom_header));
    non_planar_points_publisher_.publish(*EigenToPointCloud2(non_planar_points, odom_header));

    if (!base_frame_.empty()) {
        const Sophus::SE3d cloud2base = LookupTransform(base_frame_, cloud_frame_id);
        map_publisher_.publish(*EigenToPointCloud2(genz_map, cloud2base, odom_header));
    } else {
        map_publisher_.publish(*EigenToPointCloud2(genz_map, odom_header));
    }
}

void OdometryServer::SaveResults() {
    /**************** 保存机器人位姿 csv ****************/
    if (robot_pose_csv_en_) {
        std::string csv_file = robot_pose_csv_file_;
        // yaml 里写完整 .csv 文件名即直接用; 写成目录则在该目录下生成 robot_pose.csv
        if (csv_file.size() < 4 || csv_file.compare(csv_file.size() - 4, 4, ".csv") != 0) {
            if (!csv_file.empty() && csv_file.back() != '/') csv_file += "/";
            csv_file += "robot_pose.csv";
        }
        // csv 路径支持相对写法: 相对路径以本方法源码目录(src/genz-icp/)为基准, 绝对路径原样使用
        if (!csv_file.empty() && csv_file[0] != '/') csv_file = std::string(ROOT_DIR) + csv_file;
        const std::string csv_directory = csv_file.substr(0, csv_file.find_last_of('/'));
        int unused = system((std::string("mkdir -p ") + csv_directory).c_str());
        unused = system((std::string("rm -f ") + csv_file).c_str());
        (void)unused;

        std::ofstream csv_out(csv_file.c_str());
        if (csv_out.is_open()) {
            csv_out << "timestamp_sec,x,y,z,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg" << std::endl;
            csv_out << std::fixed << std::setprecision(9);
            for (const auto &r : pose_records_) {
                csv_out << r.t << "," << r.px << "," << r.py << "," << r.pz << ","
                        << r.qx << "," << r.qy << "," << r.qz << "," << r.qw << ","
                        << r.roll << "," << r.pitch << "," << r.yaw << std::endl;
            }
            csv_out.close();
            std::cout << "[GenZ-ICP] robot pose csv saved to: " << csv_file
                      << " (" << pose_records_.size() << " poses)" << std::endl;
        } else {
            std::cout << "[GenZ-ICP] fail to create csv file: " << csv_file << std::endl;
        }
    }

    /**************** 终端打印统计(单帧耗时 + 平均有效观测点数) ****************/
    std::cout << "\n****************************************************" << std::endl;
    if (processed_frame_num_ > 0) {
        const double avg_frame_ms =
            frame_time_sum_ * 1000.0 / static_cast<double>(processed_frame_num_);
        std::cout << "[GenZ-ICP] 平均单帧处理耗时: " << std::fixed << std::setprecision(2) << avg_frame_ms
                  << " ms  |  处理帧率(FPS): " << 1000.0 / avg_frame_ms
                  << "  (总帧数 " << processed_frame_num_ << ")" << std::endl;
        std::cout << "[GenZ-ICP] 有效观测点(平面点): 平均 "
                  << static_cast<double>(planar_num_sum_) / static_cast<double>(processed_frame_num_)
                  << " 点/帧" << std::endl;
        std::cout << "[GenZ-ICP] 非平面点: 平均 "
                  << static_cast<double>(non_planar_num_sum_) / static_cast<double>(processed_frame_num_)
                  << " 点/帧" << std::endl;
    } else {
        std::cout << "[GenZ-ICP] 没有处理任何帧(未产生轨迹)" << std::endl;
    }
    std::cout << "****************************************************" << std::endl;
}

}  // namespace genz_icp_ros

int main(int argc, char **argv) {
    ros::init(argc, argv, "genz_icp");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    genz_icp_ros::OdometryServer node(nh, nh_private);

    ros::spin();

    node.SaveResults();   // 退出时写 csv + 打印统计

    return 0;
}
