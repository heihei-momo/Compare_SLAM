#include "bievr_lio/common.h"

#include <tbb/parallel_for.h>

namespace bievr {
void calculateRanges(const Pointcloud& cloud, std::vector<double>& ranges) {
  ranges.resize(cloud.size(), 0.f);
  tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        ranges[i] = cloud[i].norm();
                      }
                    });
}
}  // namespace bievr