#ifndef BIEVR_LIO_IMU_INTEGRATOR_H_
#define BIEVR_LIO_IMU_INTEGRATOR_H_

#include <array>
#include <map>
#include <memory>

#include "bievr_lio/common.h"
#include "bievr_lio/utils.h"

namespace bievr {

static constexpr double kGMagnitude = 9.80665;
using GyroMap = std::map<uint64_t, V3>;

struct ImuConfig {
  double t_init = 0.5;  // duration of bias-init phase, used by Pipeline
  double gyro_oversample_freq = 500;
  double window_length_s = 5.0;  // length of optimization window, used by Pipeline
  // Accelerometer normalization mode, resolved during bias estimation:
  //   < 0  autodetect from the t_init window (closer to 1 -> normalized; closer to g -> raw)
  //   = 0  not normalized, use accelerations as-is
  //   > 0  normalized, multiply accelerations by g
  double normalized = 0.0;
};

struct IntegrationJacobians {
  M3 d_R_d_bg;
  M3 d_v_d_ba;
  M3 d_v_d_bg;
  M3 d_p_d_ba;
  M3 d_p_d_bg;
  double dt;
};

class BiasInitializer {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // normalized_mode follows ImuConfig::normalized (<0 autodetect, 0 raw, >0 normalized).
  BiasInitializer(double t_init_s, double normalized_mode);

  // Accumulates a sample. Returns true once the estimator has integrated more than t_init_s seconds
  // of measurements
  bool addMeasurement(const ImuMeasurement& imu);

  bool isReady() const { return ready_; }

  const V3& accBias() const { return acc_bias_; }
  const V3& gyroBias() const { return gyro_bias_; }
  const Rotation& initialOrientation() const { return initial_orientation_; }
  // Factor the accelerations must be multiplied by (1 if raw, g if normalized),
  // resolved from normalized_mode once the estimator is ready.
  double accScale() const { return acc_scale_; }

 private:
  const double t_init_s_;
  const double normalized_mode_;
  uint64_t last_time_ = 0;
  double accumulated_time_s_ = 0.0;
  V3 acc_sum_ = V3::Zero();
  V3 gyro_sum_ = V3::Zero();
  int num_samples_ = 0;
  bool ready_ = false;
  double acc_scale_ = 1.0;
  V3 acc_bias_ = V3::Zero();
  V3 gyro_bias_ = V3::Zero();
  Rotation initial_orientation_ = Rotation::Identity();
};

class ImuIntegrator {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuIntegrator(const ImuConfig& config = ImuConfig(), const V3& acc_bias = V3::Zero(),
                const V3& gyro_bias = V3::Zero());

  void integrate(const ImuMeasurement& imu_data, bool reintegrate = false);

  void integrate(const std::vector<ImuMeasurement>& imu_data) {
    for (const auto& imu : imu_data) {
      integrate(imu);
    }
  }

  void predict(const Quaternion& Qi, const V3& Pi, const V3& Vi, const V3& G, Quaternion& Qj,
               V3& Pj, V3& Vj);

  Transform predict(const uint64_t& time, const State& state, const V3& G) const;

  const IntegrationJacobians getLastJacobians() const;

  Eigen::Matrix<double, 9, 1> evaluate(const V3& Pi, const Quaternion& Qi, const V3& Vi,
                                       const V3& Pj, const Quaternion& Qj, const V3& Vj,
                                       const V3& Ba, const V3& Bg, const V3& Gravity);

  const Rotation getLastDeltaR() const { return delta_R_.rbegin()->second; }

  const uint64_t getLastTime() const { return last_time_; }

  const uint64_t getFirstTime() const { return delta_P_.begin()->first; }

  const V3& gyroBias() const { return gyro_bias_; }

  const V3 getLastDeltaP() const { return delta_P_.rbegin()->second; }

  const V3& getLastDeltaV() const { return delta_v_; }

  uint64_t getDeltaT() const { return last_time_ - first_time_; }

 private:
  GyroMap oversampleGyro(const V3& gyro, const uint64_t dt_ns);
  void rotateAcceleration(const V3& acc, const Rotation& R, const M3& d_delta_R_d_bg, V3& acc_rot,
                          M3& d_acc_rot_d_ba, M3& d_acc_rot_d_bg);

  void reintegrate(const V3& Ba, const V3& Bg);

  ImuConfig config_;
  bool initialized_ = false;
  V3 acc_bias_;
  V3 gyro_bias_;

  uint64_t first_time_ = 0;
  uint64_t last_time_ = 0;

  double inv_oversample_freq_;

  // Time-indexed: needed for oversampling / prediction at intermediate timestamps.
  std::map<uint64_t, Rotation> delta_R_;
  std::map<uint64_t, std::array<M3, 3>> d_R_d_bg_;
  std::map<uint64_t, V3> delta_P_;

  // Latest value only.
  V3 delta_v_ = V3::Zero();
  V3 last_acc_rot_;
  V3 last_gyro_;
  M3 d_delta_R_d_bg_ = M3::Zero();
  M3 d_a_d_ba_ = M3::Identity();
  M3 d_a_d_bg_ = M3::Identity();
  M3 d_v_d_ba_ = M3::Zero();
  M3 d_v_d_bg_ = M3::Zero();
  M3 d_p_d_ba_ = M3::Zero();
  M3 d_p_d_bg_ = M3::Zero();

  static constexpr double kDeltaGyro = 0.0001;
  std::vector<ImuMeasurement> imu_data_buffer_;
};

using ImuIntegratorPtr = std::shared_ptr<ImuIntegrator>;
}  // namespace bievr

#endif  // BIEVR_LIO_IMU_INTEGRATOR_H_
