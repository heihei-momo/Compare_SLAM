#include "bievr_lio/undistort.h"

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include "bievr_lio/utils.h"

namespace bievr {

void undistortCloud(const ImuIntegratorPtr& imu_integrator, const State& x,
                    const Pointcloud& pointcloud, const TimeView& times, uint64_t stamp,
                    const V3& G, Pointcloud& undistorted_pc) {
  const size_t N = static_cast<size_t>(times.cols());

  // Build (timestamp, index) pairs
  std::vector<std::pair<size_t, size_t>> time_idx(N);
  tbb::parallel_for(tbb::blocked_range<size_t>(0, N), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t i = r.begin(); i != r.end(); ++i) {
      time_idx[i] = {sToNs(times(i)), i};
    }
  });

  // Sort by timestamp
  tbb::parallel_sort(time_idx.begin(), time_idx.end());

  // Extract unique timestamps
  std::vector<size_t> timestamps;
  std::vector<std::pair<size_t, size_t>> ranges;  // [start, end) indices in time_idx

  size_t run_start = 0;
  while (run_start < time_idx.size()) {
    const size_t ts = time_idx[run_start].first;
    size_t run_end = run_start + 1;
    while (run_end < time_idx.size() && time_idx[run_end].first == ts) {
      ++run_end;
    }
    timestamps.push_back(ts);
    ranges.emplace_back(run_start, run_end);
    run_start = run_end;
  }

  // Predict transforms per timestamp, expressed at the last-point's time
  const uint64_t t_last = timestamps.back() + stamp;
  const Transform T_L1_G = imu_integrator->predict(t_last, x, G).inverse();
  std::vector<Transform> T_L1_Li(timestamps.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, timestamps.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        const uint64_t t_int = timestamps[i] + stamp;
                        T_L1_Li[i] = T_L1_G * imu_integrator->predict(t_int, x, G);
                      }
                    });

  // Apply transforms
  undistorted_pc.resize(pointcloud.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, ranges.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        const auto& range = ranges[i];
                        const Transform& T = T_L1_Li[i];
                        const auto& R = T.linear();
                        const auto& t = T.translation();
                        for (size_t k = range.first; k < range.second; ++k) {
                          const size_t idx = time_idx[k].second;
                          undistorted_pc[idx].head(3) = R * pointcloud[idx].head(3) + t;
                        }
                      }
                    });
}

}  // namespace bievr