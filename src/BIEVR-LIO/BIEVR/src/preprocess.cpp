#include "bievr_lio/preprocess.h"

#include "unordered_dense/unordered_dense.h"

namespace bievr {

namespace {

struct VoxelDownsampleEntry {
  size_t hash;
  size_t idx;
  double dist;
};

struct VoxelHashIdx {
  size_t hash;
  size_t idx;
};

struct VoxelScore {
  double score;
  size_t hash;
  size_t idx;
};

}  // namespace

void voxelDownsample(const Pointcloud& points_raw, Pointcloud& points_down, double voxel_size) {
  std::vector<VoxelDownsampleEntry> voxel_entries(points_raw.size());

  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, points_raw.size()), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t idx = r.begin(); idx != r.end(); ++idx) {
          const Eigen::Vector3i voxel = (points_raw[idx] / voxel_size).array().floor().cast<int>();
          Eigen::Vector3d voxel_center =
              (voxel.cast<double>() + Eigen::Vector3d::Constant(0.5)) * voxel_size;
          double dist = (points_raw[idx] - voxel_center).squaredNorm();
          voxel_entries[idx] = {hashIndexVoxel(voxel.matrix()), idx, dist};
        }
      });

  // Sort by voxel hash, then by distance to voxel center (closest first).
  tbb::parallel_sort(voxel_entries.begin(), voxel_entries.end(),
                     [](const VoxelDownsampleEntry& a, const VoxelDownsampleEntry& b) {
                       return std::tie(a.hash, a.dist) < std::tie(b.hash, b.dist);
                     });

  std::vector<size_t> selected_indices;
  selected_indices.reserve(points_raw.size());

  size_t curr_hash = std::numeric_limits<size_t>::max();
  for (const auto& entry : voxel_entries) {
    if (entry.hash != curr_hash) {
      selected_indices.push_back(entry.idx);
      curr_hash = entry.hash;
    }
  }

  points_down.resize(selected_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, selected_indices.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        points_down[i] = points_raw[selected_indices[i]];
                      }
                    });
}

void sampleInformed(const BIEVRMap& map, const Transform& T_W_L, const Pointcloud& points_raw,
                    Pointcloud& points_coarse, Pointcloud& points_fine, double voxel_size,
                    size_t n_samples) {
  if (points_raw.size() <= n_samples) {
    points_coarse.resize(points_raw.size());
    points_coarse.data().topRows(3) = points_raw.data().topRows(3);
    return;
  }

  // Lookup hash for each point based on its transformed position from the registration prior.
  std::vector<VoxelHashIdx> voxel_entries(points_raw.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points_raw.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        Point p_w = T_W_L.linear() * points_raw[idx] + T_W_L.translation();
                        voxel_entries[idx] = {map.hashIndex(p_w), idx};
                      }
                    });

  tbb::parallel_sort(voxel_entries.begin(), voxel_entries.end(),
                     [](const VoxelHashIdx& a, const VoxelHashIdx& b) {
                       // Add tie to ensure deterministic order for points in the same voxel, even
                       // if they have the same hash
                       return std::tie(a.hash, a.idx) < std::tie(b.hash, b.idx);
                     });

  // Extract unique hashes of observed voxels
  std::vector<VoxelHashIdx> unique_voxels;
  unique_voxels.reserve(points_raw.size());
  std::vector<bool> voxel_has_extras;
  voxel_has_extras.reserve(points_raw.size());

  for (size_t i = 0; i < voxel_entries.size(); ++i) {
    if (i == 0 || voxel_entries[i].hash != voxel_entries[i - 1].hash) {
      unique_voxels.push_back(voxel_entries[i]);
      voxel_has_extras.push_back(false);
    } else {
      voxel_has_extras.back() = true;
    }
  }

  std::vector<VoxelScore> voxel_scores(unique_voxels.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, unique_voxels.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        const auto& entry = unique_voxels[idx];
                        auto voxel = map.getVoxel(entry.hash);
                        double score =
                            (!voxel || !voxel_has_extras[idx]) ? 0.0 : voxel->mean_img_dist_;
                        voxel_scores[idx] = {score, entry.hash, entry.idx};
                      }
                    });

  // Sort by score, descending.
  tbb::parallel_sort(voxel_scores.begin(), voxel_scores.end(),
                     [](const VoxelScore& a, const VoxelScore& b) { return a.score > b.score; });

  ankerl::unordered_dense::set<size_t> informed_voxels;
  size_t n_select = std::min(n_samples, voxel_scores.size());
  informed_voxels.reserve(n_select);

  for (size_t idx = 0; idx < n_select; ++idx) {
    informed_voxels.insert(voxel_scores[idx].hash);
  }

  // Keep all points in informed voxels
  std::vector<size_t> informed_indices;
  informed_indices.reserve(points_raw.size());
  size_t curr_hash = std::numeric_limits<size_t>::max();
  bool curr_informed = false;
  for (const auto& entry : voxel_entries) {
    if (entry.hash != curr_hash) {
      curr_hash = entry.hash;
      curr_informed = informed_voxels.contains(entry.hash);
    }
    if (curr_informed) {
      informed_indices.push_back(entry.idx);
    }
  }

  points_fine.resize(informed_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points_fine.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        points_fine[idx] = points_raw[informed_indices[idx]];
                      }
                    });

  // Keep one point per coarse voxel for the rest
  size_t n_informed = informed_voxels.size();
  size_t n_coarse = voxel_scores.size() - n_informed;
  points_coarse.resize(n_coarse);
  tbb::parallel_for(tbb::blocked_range<size_t>(n_informed, voxel_scores.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        points_coarse[idx - n_informed] = points_raw[voxel_scores[idx].idx];
                      }
                    });
}

}  // namespace bievr
