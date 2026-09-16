#ifndef BIEVR_LIO_PRIOR_FACTOR_H_
#define BIEVR_LIO_PRIOR_FACTOR_H_

#include <ceres/ceres.h>

#include "bievr_lio/common.h"

namespace bievr {

class PriorFactor : public ceres::SizedCostFunction<3, 3> {
 public:
  PriorFactor() = delete;
  PriorFactor(const V3& value, const double weight) : prior_(value), weight_(weight) {}

  virtual bool Evaluate(double const* const* parameters, double* residuals,
                        double** jacobians) const;

 private:
  V3 prior_;
  double weight_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_PRIOR_FACTOR_H_