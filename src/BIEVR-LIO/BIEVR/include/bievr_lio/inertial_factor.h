#ifndef BIEVR_LIO_INERTIAL_FACTOR_H_
#define BIEVR_LIO_INERTIAL_FACTOR_H_

#include <ceres/ceres.h>

#include "bievr_lio/common.h"
#include "bievr_lio/imu_integrator.h"

namespace bievr {

class InertialFactor : public ceres::SizedCostFunction<9, 4, 3, 3, 4, 3, 3, 3, 3, 3> {
 public:
  InertialFactor() = delete;
  explicit InertialFactor(ImuIntegratorPtr integrator) : integrator_(integrator) {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const;

 private:
  ImuIntegratorPtr integrator_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_INERTIAL_FACTOR_H_
