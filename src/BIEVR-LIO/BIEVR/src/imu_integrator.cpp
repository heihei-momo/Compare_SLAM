// This IMU integrator implements the LPM integration presented in "Continuous latent state
// preintegration for inertial-aided systems", C Le Gentil, T Vidal-Calleja 
#include "bievr_lio/imu_integrator.h"

#include <cmath>

#include "bievr_lio/log++.h"
#include "bievr_lio/utils.h"

namespace bievr {

BiasInitializer::BiasInitializer(double t_init_s, double normalized_mode)
    : t_init_s_(t_init_s), normalized_mode_(normalized_mode) {}

bool BiasInitializer::addMeasurement(const ImuMeasurement& imu) {
  if (ready_) return true;

  if (num_samples_ > 0) {
    if (imu.stamp < last_time_) {
      LOG(W, "BiasInitializer: out-of-order sample " << imu.stamp << " < " << last_time_
                                                     << ". Skipping.");
      return false;
    }
    accumulated_time_s_ += nsToS(imu.stamp - last_time_);
  }
  last_time_ = imu.stamp;
  acc_sum_ += imu.acc;
  gyro_sum_ += imu.gyro;
  ++num_samples_;

  if (accumulated_time_s_ <= t_init_s_) return false;

  const V3 acc_mean = acc_sum_ / num_samples_;
  const V3 gyro_mean = gyro_sum_ / num_samples_;

  // Resolve the accelerometer normalization from the mean of the init window. With
  // gravity-normalized data the mean magnitude is ~1, otherwise it is ~g.
  if (normalized_mode_ > 0.0) {
    acc_scale_ = kGMagnitude;
  } else if (normalized_mode_ < 0.0) {
    const double mag = acc_mean.norm();
    const bool normalized = std::abs(mag - 1.0) < std::abs(mag - kGMagnitude);
    acc_scale_ = normalized ? kGMagnitude : 1.0;
    LOG(I, "BiasInitializer: autodetected IMU as " << (normalized ? "normalized" : "raw")
                                                   << " (mean |acc| = " << mag << ").");
  } else {
    acc_scale_ = 1.0;
  }

  const V3 acc_mean_scaled = acc_mean * acc_scale_;
  const V3 acc_scaled = acc_mean_scaled.normalized() * kGMagnitude;
  const Quaternion q_init = Quaternion::FromTwoVectors(acc_scaled, V3(0, 0, kGMagnitude));

  acc_bias_ = acc_mean_scaled - acc_scaled;
  gyro_bias_ = gyro_mean;
  initial_orientation_ = q_init.toRotationMatrix();
  ready_ = true;
  return true;
}

ImuIntegrator::ImuIntegrator(const ImuConfig& config, const V3& acc_bias, const V3& gyro_bias)
    : config_(config), acc_bias_(acc_bias), gyro_bias_(gyro_bias) {
  inv_oversample_freq_ = 1. / config_.gyro_oversample_freq;
  imu_data_buffer_.reserve(20);
}

void ImuIntegrator::integrate(const ImuMeasurement& imu_data, bool reintegrate) {
  if (!reintegrate) {
    imu_data_buffer_.push_back(imu_data);
  }
  const uint64_t& time = imu_data.stamp;
  const V3& acc = imu_data.acc;
  const V3& gyro = imu_data.gyro;
  if (!initialized_) {
    first_time_ = time;
    last_time_ = time;
    last_acc_rot_ = acc - acc_bias_;
    last_gyro_ = gyro - gyro_bias_;
    initialized_ = true;
    delta_R_[time] = Rotation::Identity();
    delta_P_[time] = V3::Zero();
    d_R_d_bg_[time] = std::array<M3, 3>{M3::Identity(), M3::Identity(), M3::Identity()};
    V3 acc_rot;
    rotateAcceleration(last_acc_rot_, delta_R_[time], d_delta_R_d_bg_, acc_rot, d_a_d_ba_,
                       d_a_d_bg_);
    return;
  }

  if (time < last_time_) {
    LOG(D, "ImuIntegrator: received time " << time << " is "
                                           << "before last time " << last_time_ << ". Skipping.");
    return;
  }

  const uint64_t dt_ns = time - last_time_;
  const double dt_s = nsToS(dt_ns);

  V3 gyro_unbiased = gyro - gyro_bias_;
  GyroMap oversampled_gyro;
  if (dt_s > inv_oversample_freq_) {
    oversampled_gyro = oversampleGyro(gyro_unbiased, dt_ns);
  } else {
    oversampled_gyro.emplace(last_time_, last_gyro_);
    oversampled_gyro.emplace(time, gyro_unbiased);
  }

  M3 gyro_delta = M3::Identity() * kDeltaGyro;
  for (auto it = oversampled_gyro.begin(); it != std::prev(oversampled_gyro.end()); ++it) {
    auto next_it = std::next(it);
    const double g_dt_s = nsToS(next_it->first - it->first);
    const V3 gyro_sample = it->second;

    delta_R_[next_it->first] = delta_R_[it->first] * expMap(gyro_sample * g_dt_s);
    for (int i = 0; i < 3; ++i) {
      d_R_d_bg_[next_it->first][i] =
          d_R_d_bg_[it->first][i] * expMap((gyro_sample + gyro_delta.col(i)) * g_dt_s);
    }
  }
  last_gyro_ = gyro_unbiased;

  d_delta_R_d_bg_ = M3::Zero();
  for (int i = 0; i < 3; i++) {
    // negative since numerical derivative is w.r.t. acceleration (bias is opposite direction)
    d_delta_R_d_bg_.col(i) = -logMap(delta_R_[time].transpose() * d_R_d_bg_[time][i]) / kDeltaGyro;
  }

  V3 acc_unbiased = acc - acc_bias_;
  V3 acc_rot;
  const M3 d_a_d_ba_0 = d_a_d_ba_;
  const M3 d_a_d_bg_0 = d_a_d_bg_;
  d_a_d_ba_ = M3::Identity();
  d_a_d_bg_ = M3::Identity();
  rotateAcceleration(acc_unbiased, delta_R_[time], d_delta_R_d_bg_, acc_rot, d_a_d_ba_, d_a_d_bg_);

  const M3 d_v_d_ba_0 = d_v_d_ba_;
  const M3 d_v_d_bg_0 = d_v_d_bg_;
  d_v_d_ba_ = d_v_d_ba_0 + (d_a_d_ba_0 + d_a_d_ba_) * dt_s / 2.;
  d_v_d_bg_ = d_v_d_bg_0 + (d_a_d_bg_0 + d_a_d_bg_) * dt_s / 2.;

  d_p_d_ba_ = d_p_d_ba_ + d_v_d_ba_0 * dt_s + (2 * d_a_d_ba_0 + d_a_d_ba_) * dt_s * dt_s / 6.;
  d_p_d_bg_ = d_p_d_bg_ + d_v_d_bg_0 * dt_s + (2 * d_a_d_bg_0 + d_a_d_bg_) * dt_s * dt_s / 6.;

  delta_P_[time] = delta_P_.rbegin()->second + delta_v_ * dt_s +
                   (2 * last_acc_rot_ + acc_rot) * dt_s * dt_s / 6.;
  delta_v_ += (acc_rot + last_acc_rot_) * dt_s / 2.;
  last_acc_rot_ = acc_rot;
  last_time_ = time;
}

void ImuIntegrator::rotateAcceleration(const V3& acc, const Rotation& R, const M3& d_delta_R_d_bg,
                                       V3& acc_rot, M3& d_acc_rot_d_ba, M3& d_acc_rot_d_bg) {
  acc_rot = R * acc;
  d_acc_rot_d_ba = -R;
  auto temp_d_R_d_bw = jacobianExpMapZeroM(d_delta_R_d_bg);
  Eigen::Matrix<double, 1, 9> temp_1;
  temp_1 << R(0, 0) * acc(0), R(0, 1) * acc(0), R(0, 2) * acc(0), R(0, 0) * acc(1),
      R(0, 1) * acc(1), R(0, 2) * acc(1), R(0, 0) * acc(2), R(0, 1) * acc(2), R(0, 2) * acc(2);
  Eigen::Matrix<double, 1, 9> temp_2;
  temp_2 << R(1, 0) * acc(0), R(1, 1) * acc(0), R(1, 2) * acc(0), R(1, 0) * acc(1),
      R(1, 1) * acc(1), R(1, 2) * acc(1), R(1, 0) * acc(2), R(1, 1) * acc(2), R(1, 2) * acc(2);
  Eigen::Matrix<double, 1, 9> temp_3;
  temp_3 << R(2, 0) * acc(0), R(2, 1) * acc(0), R(2, 2) * acc(0), R(2, 0) * acc(1),
      R(2, 1) * acc(1), R(2, 2) * acc(1), R(2, 0) * acc(2), R(2, 1) * acc(2), R(2, 2) * acc(2);
  d_acc_rot_d_bg.row(0) = temp_1 * temp_d_R_d_bw;
  d_acc_rot_d_bg.row(1) = temp_2 * temp_d_R_d_bw;
  d_acc_rot_d_bg.row(2) = temp_3 * temp_d_R_d_bw;
}

GyroMap ImuIntegrator::oversampleGyro(const V3& gyro, const uint64_t dt_ns) {
  const int num_samples = std::floor(nsToS(dt_ns) * config_.gyro_oversample_freq);
  const uint64_t quantum = dt_ns / num_samples;
  V3 gyro_quantum = (gyro - last_gyro_) / num_samples;
  GyroMap gyro_map;
  for (int i = 0; i < num_samples; ++i) {
    gyro_map.emplace(last_time_ + i * quantum, last_gyro_ + gyro_quantum * i);
  }
  gyro_map.emplace(last_time_ + dt_ns, gyro);
  return gyro_map;
}

const IntegrationJacobians ImuIntegrator::getLastJacobians() const {
  IntegrationJacobians jacobians;
  jacobians.d_R_d_bg = d_delta_R_d_bg_;
  jacobians.d_v_d_ba = d_v_d_ba_;
  jacobians.d_v_d_bg = d_v_d_bg_;
  jacobians.d_p_d_ba = d_p_d_ba_;
  jacobians.d_p_d_bg = d_p_d_bg_;
  jacobians.dt = nsToS(last_time_ - first_time_);
  return jacobians;
}

void ImuIntegrator::reintegrate(const V3& Ba, const V3& Bg) {
  if (imu_data_buffer_.empty()) {
    LOG(D, "ImuIntegrator: no IMU data to reintegrate. Skipping.");
    return;
  }

  initialized_ = false;
  delta_R_.clear();
  delta_P_.clear();
  d_R_d_bg_.clear();
  delta_v_ = V3::Zero();
  d_delta_R_d_bg_ = M3::Zero();
  d_a_d_ba_ = M3::Identity();
  d_a_d_bg_ = M3::Identity();
  d_v_d_ba_ = M3::Zero();
  d_v_d_bg_ = M3::Zero();
  d_p_d_ba_ = M3::Zero();
  d_p_d_bg_ = M3::Zero();

  acc_bias_ = Ba;
  gyro_bias_ = Bg;

  for (const auto& imu_data : imu_data_buffer_) {
    integrate(imu_data, true);
  }
}

Eigen::Matrix<double, 9, 1> ImuIntegrator::evaluate(const V3& Pi, const Quaternion& Qi,
                                                    const V3& Vi, const V3& Pj,
                                                    const Quaternion& Qj, const V3& Vj,
                                                    const V3& Ba, const V3& Bg, const V3& Gravity) {
  Eigen::Matrix<double, 9, 1> residuals;

  V3 dba = Ba - acc_bias_;
  V3 dbg = Bg - gyro_bias_;

  if (dba.norm() > 5e-2) {
    reintegrate(Ba, Bg);
    dba = Ba - acc_bias_;
    dbg = Bg - gyro_bias_;
  }

  const V3& gravity_vec = Gravity;

  const M3& delta_R = delta_R_.rbegin()->second;
  const V3& delta_p = delta_P_.rbegin()->second;

  const Rotation corrected_delta_R = delta_R * expMap(d_delta_R_d_bg_ * dbg);
  const Quaternion corrected_delta_q = Quaternion(corrected_delta_R);
  const V3 corrected_delta_v = delta_v_ + d_v_d_ba_ * dba + d_v_d_bg_ * dbg;
  const V3 corrected_delta_p = delta_p + d_p_d_ba_ * dba + d_p_d_bg_ * dbg;

  const double dt = nsToS(last_time_ - first_time_);
  residuals.block<3, 1>(3, 0) =
      Qi.inverse() * (0.5 * gravity_vec * dt * dt + Pj - Pi - Vi * dt) - corrected_delta_p;

  residuals.block<3, 1>(0, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
  residuals.block<3, 1>(6, 0) = Qi.inverse() * (gravity_vec * dt + Vj - Vi) - corrected_delta_v;
  return residuals;
}

void ImuIntegrator::predict(const Quaternion& Qi, const V3& Pi, const V3& Vi, const V3& G,
                            Quaternion& Qj, V3& Pj, V3& Vj) {
  const double dt_s = nsToS(getDeltaT());
  Pj = Pi + Qi * getLastDeltaP() + Vi * dt_s - 0.5 * G * dt_s * dt_s;
  Vj = Vi + Qi * getLastDeltaV() - G * dt_s;
  Qj = Qi * getLastDeltaR();
  Qj.normalize();
}

Transform ImuIntegrator::predict(const uint64_t& time, const State& state, const V3& G) const {
  Transform T;
  auto it = delta_P_.lower_bound(time);
  if (it == delta_P_.end()) {
    LOG(D, "ImuIntegrator: requested time " << time << " is "
                                            << "after last time " << delta_P_.rbegin()->first
                                            << ". Skipping.");
    T.translation() = state.p;
    T.linear() = state.quat.toRotationMatrix();
    return T;
  }

  if (it == delta_P_.begin()) {
    LOG(D, "ImuIntegrator: requested time " << time << " is "
                                            << "before first time " << delta_P_.begin()->first
                                            << ". Skipping.");
    T.translation() = state.p;
    T.linear() = state.quat.toRotationMatrix();
    return T;
  }

  double dt_s = nsToS(time - delta_P_.begin()->first);
  auto it_prev = std::prev(it);
  double alpha = nsToS(time - it_prev->first) / nsToS(it->first - it_prev->first);

  V3 delta_P_int = it_prev->second + alpha * (it->second - it_prev->second);
  T.translation() = state.p + state.quat * delta_P_int + state.v * dt_s - 0.5 * G * dt_s * dt_s;

  auto rot_it = delta_R_.lower_bound(time);
  auto rot_it_prev = std::prev(rot_it);
  double alpha_rot = nsToS(time - rot_it_prev->first) / nsToS(rot_it->first - rot_it_prev->first);
  V3 R_diff = logMap(rot_it_prev->second.transpose() * rot_it->second);
  Rotation delta_R_int = rot_it_prev->second * expMap(R_diff * alpha_rot);
  T.linear() = state.quat * delta_R_int;
  return T;
}

}  // namespace bievr
