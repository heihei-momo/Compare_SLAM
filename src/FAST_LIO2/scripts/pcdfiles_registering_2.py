#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import open3d as o3d
import numpy as np
import os
import re

# --- 读取点云序列 ---
def load_scan_sequence(folder):
    files = os.listdir(folder)
    scan_files = []

    for f in files:
        m = re.match(r"scans_(\d+)\.pcd", f)
        if m:
            scan_files.append((int(m.group(1)), f))#scan_files = [(1, 'scans_1.pcd'), (2, 'scans_2.pcd')......(10, 'scans_10.pcd')]

    scan_files.sort(key=lambda x: x[0])#排序

    if len(scan_files) == 0:
        raise RuntimeError("No scans_*.pcd found")

    return [os.path.join(folder, f[1]) for f in scan_files]#返回一个 按帧号排序的完整路径列表


# --- 下采样 ---
def preprocess(pcd, voxel_size):
    return pcd.voxel_down_sample(voxel_size)


# --- 高度滤波，可选 ---
def height_filter(pcd, z_min=-2.0, z_max=4.0):
    pts = np.asarray(pcd.points)
    mask = (pts[:, 2] > z_min) & (pts[:, 2] < z_max)
    pcd.points = o3d.utility.Vector3dVector(pts[mask])
    return pcd


# --- scan-to-map ICP ---
def icp(source, target, voxel_size):#当前帧点云，目标对齐点云，匹配阈值
    threshold = voxel_size * 2.0
    init = np.eye(4)#设置初始变换矩阵是单位矩阵

    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        threshold,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)#最大迭代次数50
    )
    return result.transformation


# --- 主函数：分批累积 + 保存 partial map ---
def incremental_registration_batch(folder, voxel_size=0.1, batch_size=50):
    scan_paths = load_scan_sequence(folder)#返回路径的列表
    print(f"Found {len(scan_paths)} scans")

    ref_cloud = o3d.io.read_point_cloud(scan_paths[0])#第一个pcd完整路径
    ref_cloud = height_filter(ref_cloud)#高度滤波（-2到4）
    ref_cloud = preprocess(ref_cloud, voxel_size)#下采样

    batch_idx = 0
    all_partial_files = []

    for i in range(1, len(scan_paths)):
        print(f"Registering {os.path.basename(scan_paths[i])} to accumulated map")

        curr = o3d.io.read_point_cloud(scan_paths[i])
        curr = height_filter(curr)
        curr = preprocess(curr, voxel_size)

        T = icp(curr, ref_cloud, voxel_size)
        curr.transform(T)

        ref_cloud += curr

        # 每 batch_size 帧保存一次 partial map
        if i % batch_size == 0 or i == len(scan_paths) - 1:
            partial_file = f"partial_map_{batch_idx}.pcd"
            print(f"Saving {partial_file}")
            o3d.io.write_point_cloud(partial_file, ref_cloud)
            all_partial_files.append(partial_file)
            batch_idx += 1

            # 释放内存，只保留当前累积的点云
            ref_cloud = preprocess(ref_cloud, voxel_size)

    # --- 合并所有 partial map ---
    print("Merging all partial maps...")
    final_map = o3d.geometry.PointCloud()
    for f in all_partial_files:
        pc = o3d.io.read_point_cloud(f)
        final_map += pc

    # 下采样防止点数过多
    final_map = final_map.voxel_down_sample(voxel_size)
    o3d.io.write_point_cloud("final_map_2.pcd", final_map)
    print("Saved final_map.pcd")

    return final_map


def main():
    folder = "/home/yyh/study_files/fast_lio2_ws/src/FAST_LIO/PCD"
    voxel_size = 0.1
    batch_size = 50

    final_map = incremental_registration_batch(folder, voxel_size, batch_size)
    o3d.visualization.draw_geometries([final_map])


if __name__ == "__main__":
    main()
