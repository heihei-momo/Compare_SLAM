#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import open3d as o3d
import numpy as np
import os
import re

#1-2=a  a+3=b .....拼合后再和下一帧进行配准方法
def load_scan_sequence(folder):
    """
    scans_1.pcd, scans_2.pcd, ..., scans_n.pcd
    """
    files = os.listdir(folder)
    scan_files = []

    for f in files:
        m = re.match(r"scans_(\d+)\.pcd", f)
        if m:
            scan_files.append((int(m.group(1)), f))

    scan_files.sort(key=lambda x: x[0])

    if len(scan_files) == 0:
        raise RuntimeError("No scans_*.pcd found")

    return [os.path.join(folder, f[1]) for f in scan_files]


def preprocess(pcd, voxel_size):
    pcd = pcd.voxel_down_sample(voxel_size)
    return pcd


def height_filter(pcd, z_min=-2.0, z_max=3.0):
    pts = np.asarray(pcd.points)
    mask = (pts[:, 2] > z_min) & (pts[:, 2] < z_max)
    pcd.points = o3d.utility.Vector3dVector(pts[mask])
    return pcd


def icp(source, target, voxel_size):
    threshold = 0.1
    init = np.eye(4)

    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        threshold,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
    )

    return result.transformation


def incremental_registration(folder, voxel_size=0.5):
    scan_paths = load_scan_sequence(folder)
    print(f"Found {len(scan_paths)} scans")

    # === 初始参考帧 ===
    ref_cloud = o3d.io.read_point_cloud(scan_paths[0])
    ref_cloud = height_filter(ref_cloud)
    ref_cloud = preprocess(ref_cloud, voxel_size)

    for i in range(1, len(scan_paths)):
        print(f"Registering {os.path.basename(scan_paths[i])} "
              f"to accumulated map")

        curr = o3d.io.read_point_cloud(scan_paths[i])
        curr = height_filter(curr)
        curr = preprocess(curr, voxel_size)

        # 当前帧 → 累积点云
        T = icp(curr, ref_cloud, voxel_size)
        curr.transform(T)

        # 融合
        ref_cloud += curr

        # 防止点云无限增长
        ref_cloud = ref_cloud.voxel_down_sample(voxel_size)

    return ref_cloud


def main():
    folder = "/home/yyh/study_files/fast_lio2_ws/src/FAST_LIO/PCD"
    voxel_size = 0.1

    final_map = incremental_registration(folder, voxel_size)

    o3d.io.write_point_cloud("final_map_1.pcd", final_map)
    print("Saved final_map.pcd")
    print(f"Final map has {len(final_map.points)} points")

    o3d.visualization.draw_geometries([final_map])


if __name__ == "__main__":
    main()
