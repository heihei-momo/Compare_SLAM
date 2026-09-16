#include "bievr_lio/inertial_factor.h"

#include "bievr_lio/log++.h"
#include "bievr_lio/utils.h"

namespace bievr {

bool InertialFactor::Evaluate(double const* const* parameters, double* residuals,
                              double** jacobians) const {
  // Map parameters to local variables we can work with
  QuatMap rot_0(parameters[0]);     // Rotation at first time
  V3Map pos_0(parameters[1]);       // Position at first time
  V3Map vel_0(parameters[2]);       // Velocity at first time
  QuatMap rot_1(parameters[3]);     // Rotation at second time
  V3Map pos_1(parameters[4]);       // Position at second time
  V3Map vel_1(parameters[5]);       // Velocity at second time
  V3Map bias_acc(parameters[6]);    // Accel bias (constant between the two times)
  V3Map bias_gyr(parameters[7]);    // Gyro bias (constant between the two times)
  V3Map grav_param(parameters[8]);  // Gravity direction (constant between the two times)

  const Eigen::Vector3d grav_vec = grav_param * kGMagnitude;

  // Calculate residual from preintegration
  Eigen::Map<Eigen::Matrix<double, 9, 1>> res_out(residuals);
  res_out =
      integrator_->evaluate(pos_0, rot_0, vel_0, pos_1, rot_1, vel_1, bias_acc, bias_gyr, grav_vec);

  // Calculate Jacobians
  if (jacobians) {
    const IntegrationJacobians jacs_data = integrator_->getLastJacobians();
    const double dt = jacs_data.dt;

    // Cache mathematically redundant computations
    const Rotation R0_inv = rot_0.inverse().toRotationMatrix();
    const Quaternion delta_q(integrator_->getLastDeltaR());
    const Quaternion corr_dq =
        delta_q * smallAnglePertubation(jacs_data.d_R_d_bg * (bias_gyr - integrator_->gyroBias()));

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 9, 4, Eigen::RowMajor>> jac_rot_0(jacobians[0]);
      jac_rot_0.setZero();

      // Rotation error wrt rotation at first time
      jac_rot_0.block<3, 3>(0, 0) =
          -(leftMultiply(rot_1.inverse() * rot_0) * rightMultiply(corr_dq))
               .bottomRightCorner<3, 3>();

      // Position error wrt rotation at first time
      jac_rot_0.block<3, 3>(3, 0) =
          skew(rot_0.inverse() * (0.5 * grav_vec * dt * dt + pos_1 - pos_0 - vel_0 * dt));

      // Velocity error wrt rotation at first time
      jac_rot_0.block<3, 3>(6, 0) = skew(rot_0.inverse() * (grav_vec * dt + vel_1 - vel_0));

      if (jac_rot_0.maxCoeff() > 1e8 || jac_rot_0.minCoeff() < -1e8) {
        LOG(W, "Preintegration is unstable");
      }
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jac_pos_0(jacobians[1]);
      jac_pos_0.setZero();

      // Position error wrt position at first time
      jac_pos_0.block<3, 3>(3, 0) = -R0_inv;
    }

    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jac_vel_0(jacobians[2]);
      jac_vel_0.setZero();

      // Position error wrt velocity at first time
      jac_vel_0.block<3, 3>(3, 0) = -R0_inv * dt;

      // Velocity error wrt velocity at first time
      jac_vel_0.block<3, 3>(6, 0) = -R0_inv;
    }

    if (jacobians[3]) {
      Eigen::Map<Eigen::Matrix<double, 9, 4, Eigen::RowMajor>> jac_rot_1(jacobians[3]);
      jac_rot_1.setZero();

      // Rotation error wrt rotation at first time
      jac_rot_1.block<3, 3>(0, 0) =
          leftMultiply(corr_dq.inverse() * rot_0.inverse() * rot_1).bottomRightCorner<3, 3>();
    }

    if (jacobians[4]) {
      Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jac_pos_1(jacobians[4]);
      jac_pos_1.setZero();

      // Position error wrt position at second time
      jac_pos_1.block<3, 3>(3, 0) = R0_inv;
    }

    if (jacobians[5]) {
      Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jac_vel_1(jacobians[5]);
      jac_vel_1.setZero();

      // Velocity error wrt velocity at second time
      jac_vel_1.block<3, 3>(6, 0) = R0_inv;
    }

    if (jacobians[6]) {
      Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jac_bias_acc(jacobians[6]);
      jac_bias_acc.setZero();

      // Position error wrt accel bias
      jac_bias_acc.block<3, 3>(3, 0) = -jacs_data.d_p_d_ba;

      // Velocity error wrt accel bias
      jac_bias_acc.block<3, 3>(6, 0) = -jacs_data.d_v_d_ba;
    }

    if (jacobians[7]) {
      Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jac_bias_gyro(jacobians[7]);
      jac_bias_gyro.setZero();

      // Rotation error wrt gyro bias
      jac_bias_gyro.block<3, 3>(0, 0) =
          -leftMultiply(rot_1.inverse() * rot_0 * delta_q).bottomRightCorner<3, 3>() *
          jacs_data.d_R_d_bg;

      // Position error wrt gyro bias
      jac_bias_gyro.block<3, 3>(3, 0) = -jacs_data.d_p_d_bg;

      // Velocity error wrt gyro bias
      jac_bias_gyro.block<3, 3>(6, 0) = -jacs_data.d_v_d_bg;
    }

    if (jacobians[8]) {
      Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jac_gravity(jacobians[8]);
      jac_gravity.setZero();

      // Position error wrt gravity
      jac_gravity.block<3, 3>(3, 0) = 0.5 * R0_inv * kGMagnitude * dt * dt;

      // Velocity error wrt gravity
      jac_gravity.block<3, 3>(6, 0) = R0_inv * kGMagnitude * dt;
    }
  }

  return true;
}
}  // namespace bievr
