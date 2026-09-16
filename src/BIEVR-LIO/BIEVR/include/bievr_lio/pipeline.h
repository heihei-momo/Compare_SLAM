#ifndef BIEVR_LIO_PIPELINE_H_
#define BIEVR_LIO_PIPELINE_H_

#include <typeindex>

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/imu_integrator.h"
#include "bievr_lio/log++.h"
#include "bievr_lio/ls_optimizer.h"
#include "bievr_lio/preprocess.h"
#include "bievr_lio/utils.h"

namespace bievr {

class Pipeline {
 public:
  struct Config {
    Config() = default;
    PreprocessConfig preprocess;
    ImuConfig imu;
    RegistrationConfig registration;
    BIEVRMap::Config map;
    bool print_timing = false;
    bool publish_all_clouds = false;
    bool print_debug = false;      // when true, lower the log level to show DEBUG messages
    bool print_dashboard = false;  // when true, print the fancy live status dashboard
    // Path to the ASCII art shown at the top of the dashboard (e.g. bievr_ascii.txt).
    std::string dashboard_ascii_path = "";
    Transform T_I_L = Transform::Identity();  // Transform from LiDAR to IMU frame
    std::string map_frame = "map";
    std::string body_frame = "body";
    std::string log_path = "";

    size_t min_points_for_map_init = 100;
    size_t map_size_running_threshold = 5;
    size_t informed_sample_count = 300;
    size_t min_map_size_for_imu_opt = 100;
    size_t min_imu_integrators_for_opt = 5;
    double gravity_prior_weight = 5.0;
  };

  explicit Pipeline(const Config& config);
  virtual ~Pipeline() = default;

  void processFrame(const std::vector<ImuMeasurement>& imu_data,
                    const StampedIntensityPointcloud& pointcloud);

  // ==================== 机器人位姿记录 + 运行统计 ====================
  // 每处理完一帧就把位姿累积到内存(与 Log/traj*.txt 用的是同一个时刻/同一个位姿),
  // 退出时才整文件写出 11 列 csv(不是追加), 路径由 yaml 的 robotpose/csv_file 指定。
  struct RobotPoseRecord {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double stamp_s = 0.0;  // 秒
    Transform T_W_B;       // map 系下的机体(IMU)位姿
  };

  // 一次运行结束后的汇总统计
  struct RunSummary {
    size_t frames = 0;                  // 处理帧数(单帧耗时采样数)
    double total_frame_time_s = 0.0;    // 单帧耗时之和
    size_t updates = 0;                 // 真正做了点云配准更新的次数
    double total_effective_points = 0;  // 每次更新的有效观测点数之和
    double meanFrameTimeMs() const {
      return frames > 0 ? 1000.0 * total_frame_time_s / static_cast<double>(frames) : 0.0;
    }
    double meanEffectivePoints() const {
      return updates > 0 ? total_effective_points / static_cast<double>(updates) : 0.0;
    }
  };

  const std::vector<RobotPoseRecord>& robotPoses() const { return robot_poses_; }
  RunSummary runSummary() const;

  // 把记录的位姿整文件写成 csv(先删旧文件再截断写, 目录不存在会自动创建)。
  bool saveRobotPoseCsv(const std::string& file_path) const;
  // 在终端打印统计: 平均单帧处理耗时 / 帧率 / 平均每次更新的有效观测点数。
  void printRunSummary() const;

  template <typename T>
  void registerPublisher(std::function<void(const T&, const Header&, const std::string& topic,
                                            const std::string& child_frame)>
                             func) {
    auto wrapper = [func](const void* val, const Header& header, const std::string& topic,
                          const std::string& child_frame) {
      func(*static_cast<const T*>(val), header, topic, child_frame);
    };
    publishers_[typeid(T)] = wrapper;
  }

 private:
  enum class Phase { NeedBias, NeedMap, Running };

  // Pipeline helpers
  bool initializeBias(const std::vector<ImuMeasurement>& imu_data, const Pointcloud& pointcloud);
  void tryInitMap(uint64_t stamp, const State& x_j_pred, const Transform& T_W_I_init,
                  const Pointcloud& undistorted, const IntensityView& intensities,
                  std::vector<double>& ranges, const Header& header);
  void sampleSource(const Pointcloud& undistorted, const Transform& T_W_I_init,
                    Pointcloud& filtered, Pointcloud& coarse, Pointcloud& fine) const;

  // State and optimization management
  bool addState(const uint64_t time, const Quaternion& quat, const V3& p, const V3& v);
  bool addImuIntegrator(ImuIntegratorPtr imu_integrator);
  bool optimizeInertialWindow();

  // Publishing
  template <typename T>
  void publish(const T& value, const Header& header, const std::string& topic,
               const std::string& child_frame = "") const {
    auto it = publishers_.find(typeid(T));
    if (it != publishers_.end()) {
      it->second(static_cast<const void*>(&value), header, topic, child_frame);
    } else {
      LOG(E, "No publisher registered in pipeline for type.");
    }
  }

  void publishFrame(const Header& header, const Transform& T_W_I, const Pointcloud& full_registered,
                    const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                    const Pointcloud& source_fine, const Pointcloud& undistorted,
                    const IntensityView& intensities);
  void publishLatestState(const Header& header);
  // 把当前位姿追加到轨迹里并发布 nav_msgs/Path(rviz 的 Path 显示用)
  void publishPath(const Transform& T_W_I, const Header& header);
  void publishDebugClouds(const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                          const Pointcloud& source_fine, const Pointcloud& undistorted_cloud,
                          const IntensityView& intensities, const Transform& T_W_I,
                          const Header& header);

  // Logging
  void logTUM(double timestamp, const Transform& pose);

  Config config_;
  std::map<uint64_t, State> states_;
  std::map<uint64_t, ImuIntegratorPtr> imu_integrators_;

  size_t seq_counter_ = 0;
  Phase phase_ = Phase::NeedBias;
  std::unique_ptr<BiasInitializer> bias_initializer_;
  std::shared_ptr<BIEVRMap> map_;
  V3 acc_bias_ = V3::Zero();
  V3 gyro_bias_ = V3::Zero();
  V3 gravity_dir_ = V3(0, 0, 1);
  // Latest gyro reading, used to report the angular velocity in the odometry twist.
  V3 latest_gyro_ = V3::Zero();
  // Accelerometer scale resolved during bias estimation (1 if raw, g if the IMU
  // reports gravity-normalized accelerations). Applied to all incoming IMU data.
  double imu_acc_scale_ = 1.0;

  using PublishFunction =
      std::function<void(const void*, const Header&, const std::string&, const std::string&)>;
  std::unordered_map<std::type_index, PublishFunction> publishers_;
  std::shared_ptr<std::ofstream> tum_log_;

  // 累积的轨迹与用于退出统计的计数器
  Path path_;
  std::vector<RobotPoseRecord> robot_poses_;
  size_t update_count_ = 0;
  double effective_points_sum_ = 0.0;

  // Accumulated state for the live status dashboard (printDashboard in utils).
  DashboardState dashboard_;
};
}  // namespace bievr

#endif  // BIEVR_LIO_PIPELINE_H_
