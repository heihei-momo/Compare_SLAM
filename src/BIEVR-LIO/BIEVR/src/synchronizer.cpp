#include "bievr_lio/synchronizer.h"

#include "bievr_lio/log++.h"
#include "bievr_lio/utils.h"

namespace bievr {

bool Synchronizer::addImu(const ImuMeasurement& imu) {
  if (!imu_queue_.empty() && imu.stamp < imu_queue_.back().stamp) {
    LOG(W, "IMU measurement at " << imu.stamp << " out of order. Skipping.");
    return false;
  }
  imu_queue_.push_back(imu);
  synchronizeData();
  return true;
}

bool Synchronizer::addPointcloud(const StampedIntensityPointcloud& cloud) {
  if (!point_queue_.empty() && cloud.end_stamp < point_queue_.back().end_stamp) {
    LOG(W, "Pointcloud out of order. Time " << cloud.end_stamp << " smaller than "
                                            << point_queue_.back().end_stamp << ". Skipping.");
    return false;
  }
  point_queue_.push_back(cloud);
  synchronizeData();
  return true;
}

void Synchronizer::synchronizeData() {
  static const uint64_t kOverlapNs = sToNs(0.02);

  if (imu_queue_.size() < 2 || point_queue_.empty()) {
    return;
  }

  // Clean out point clouds that arrived before IMU
  while (!point_queue_.empty() &&
         (point_queue_.front().stamp + kOverlapNs) < imu_queue_.front().stamp) {
    LOG(W,
        "Removed pointcloud: " << point_queue_.front().stamp << " < " << imu_queue_.front().stamp);
    point_queue_.pop_front();
  }

  if (point_queue_.empty()) {
    return;
  }

  const StampedIntensityPointcloud& pointcloud = point_queue_.front();
  const uint64_t t_pointcloud = pointcloud.end_stamp;
  if (imu_queue_.back().stamp < t_pointcloud) {
    return;
  }

  std::vector<ImuMeasurement> imu_data;
  imu_data.reserve(imu_queue_.size());
  while (imu_queue_.size() > 1) {
    const ImuMeasurement& imu_curr = imu_queue_[0];
    const ImuMeasurement& imu_next = imu_queue_[1];
    imu_data.push_back(imu_curr);

    if (imu_next.stamp < t_pointcloud) {
      imu_queue_.pop_front();
    } else {
      // Linearly interpolate between the last IMU measurements
      const uint64_t t_curr = imu_curr.stamp;
      const uint64_t t_next = imu_next.stamp;
      const uint64_t t_diff = t_next - t_curr;
      const double t_ratio = static_cast<double>(t_pointcloud - t_curr) / t_diff;
      ImuMeasurement imu_interpolated;
      imu_interpolated.stamp = t_pointcloud;
      imu_interpolated.acc = imu_curr.acc + t_ratio * (imu_next.acc - imu_curr.acc);
      imu_interpolated.gyro = imu_curr.gyro + t_ratio * (imu_next.gyro - imu_curr.gyro);
      imu_data.push_back(imu_interpolated);
      // Add the interpolated measurement back to the front of the queue for the next point cloud
      imu_queue_.front() = imu_interpolated;
      break;
    }
  }

  pipeline_->processFrame(imu_data, pointcloud);
  point_queue_.pop_front();
}

}  // namespace bievr