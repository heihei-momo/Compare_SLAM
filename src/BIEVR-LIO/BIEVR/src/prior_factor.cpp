#include "bievr_lio/prior_factor.h"

namespace bievr {

bool PriorFactor::Evaluate(double const* const* parameters, double* residuals,
                           double** jacobians) const {
  Eigen::Map<const V3> value(parameters[0]);

  Eigen::Map<V3> residual(residuals);
  residual = value - prior_;
  residual = weight_ * residual;
  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian(jacobians[0]);
      jacobian.setZero();

      jacobian = Eigen::Matrix<double, 3, 3>::Identity();
      jacobian = weight_ * jacobian;
    }
  }
  return true;
}

}  // namespace bievr