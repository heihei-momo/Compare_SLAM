#ifndef BIEVR_LIO_SYNCHRONIZER_H_
#define BIEVR_LIO_SYNCHRONIZER_H_

#include <deque>

#include "bievr_lio/pipeline.h"

namespace bievr {

class Synchronizer {
 public:
  explicit Synchronizer(std::shared_ptr<Pipeline> pipeline) : pipeline_(pipeline) {}
  virtual ~Synchronizer() = default;
  bool addImu(const ImuMeasurement& imu);
  bool addPointcloud(const StampedIntensityPointcloud& cloud);

 private:
  void synchronizeData();
  std::deque<ImuMeasurement> imu_queue_;
  std::deque<StampedIntensityPointcloud> point_queue_;
  std::shared_ptr<Pipeline> pipeline_;
};
}  // namespace bievr

#endif  // BIEVR_LIO_SYNCHRONIZER_H_