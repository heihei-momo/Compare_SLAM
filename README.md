<p align="center">
  <img src="Picture/大狗.png" width="520" alt="大狗">
</p>


<h1 align="center">一些SLAM开源算法对比工作空间</h1>

<p align="center">
  在统一数据集下运行 8 个开源 LiDAR-Inertial SLAM / LIO 方法，并输出统一格式的轨迹 CSV
</p>

---

## 1. 概述

本工作空间集成了 8 个公开的 LiDAR-Inertial SLAM / LIO 方法，并为每个方法补齐了一套统一的实验接口，使其能够在同一数据集下按统一格式输出结果。

| 统一接口 | 说明 |
|:--|:--|
| 输入点云距离截断 | 点云进入处理流程时，按点到雷达原点的距离进行过滤；参数位于各方法自身的 yaml 配置中 |
| 位姿 CSV 导出 | 退出时将该方法估计的位姿以整文件覆盖方式写出为 11 列 CSV（格式见第 6 节） |
| 终端运行统计 | 退出时打印平均单帧处理耗时、处理帧率与平均每次更新的有效观测点数 |
| 轨迹可视化 | 启动 rviz 后可直接观察估计轨迹，以及配准点云与地图 |

各方法默认均为 **Velodyne 雷达配置**（话题 `/velodyne_points` 与 `/imu/data`）。所有方法使用默认配置即可运行，无需额外传参。

## 2. 环境与构建

| 项目 | 版本 / 路径 |
|:--|:--|
| 操作系统 | Ubuntu 20.04 |
| ROS 版本 | Noetic |
| 构建系统 | catkin_make |
| 工作空间 | `Compare_SLAM` |

在 catkin 工作空间根目录下执行：

```bash
catkin_make
source devel/setup.bash
```

> 说明：BIEVR-LIO 的核心库为 plain-CMake 包。修改 `src/BIEVR-LIO/BIEVR/include/` 下的头文件后，需先执行
> `bash src/BIEVR-LIO/build_core_cmake.sh`，再执行 `catkin_make`。

## 3. 数据

各方法默认订阅 Velodyne 雷达话题 `/velodyne_points` 与 IMU 话题 `/imu/data`。任意包含这两个话题的 rosbag 均可直接运行，无需修改配置。

## 4. 方法列表

| 方法 | 核心思想 | 启动命令 |
|:--|:--|:--|
| LIO-SAM | 激光惯性里程计 + GTSAM 因子图优化 | `roslaunch lio_sam run.launch` |
| FAST-LIO2 | 迭代扩展卡尔曼滤波 + ikd-Tree 增量地图 | `roslaunch fast_lio mapping_velodyne.launch` |
| PV-LIO | 点-面与体素联合，自适应平面/体素地图 | `roslaunch pv_lio mapping_velodyne.launch` |
| COIN-LIO | 点云与图像亮度联合约束 | `roslaunch coin_lio mapping_velodyne.launch` |
| VoxelMap | 自适应体素地图表示 | `roslaunch voxel_map mapping_velodyne.launch` |
| GenZ-ICP | 平面/非平面点自适应匹配 | `roslaunch genz_icp odometry.launch` |
| DALI-SLAM | 双样条运动畸变校正 + 退化感知 ESKF + 多约束位姿图（DA-LIO + MC-PGO） | `roslaunch da_lio run_dalio_velodyne.launch` |
| BIEVR-LIO | 体素级有向高度图（bump-image）地图表示 | `roslaunch bievr_lio_ros process_topics.launch` |

## 5. 运行方式

除 BIEVR-LIO 的离线模式外，所有方法均采用相同的两终端流程：

- **终端 1**：启动算法（使用默认配置，rviz 会自动打开）
- **终端 2**：播放数据包

以 FAST-LIO2 为例：

```bash
# 终端 1
roslaunch fast_lio mapping_velodyne.launch

# 终端 2
rosbag play <bag路径>
```

在终端 1 按 `Ctrl-C` 结束运行后，程序会写出位姿 CSV 并打印运行统计。

各方法的启动命令与结果输出位置如下：

| 方法 | 启动命令 | 位姿 CSV 输出目录 | rviz 轨迹话题 |
|:--|:--|:--|:--|
| LIO-SAM | `roslaunch lio_sam run.launch` | `src/LIO-SAM/Robotpose/` | 由自带 rviz 配置显示 |
| FAST-LIO2 | `roslaunch fast_lio mapping_velodyne.launch` | `src/FAST_LIO2/Robotpose/` | 由自带 rviz 配置显示 |
| PV-LIO | `roslaunch pv_lio mapping_velodyne.launch` | `src/PV-LIO/Robotpose/` | 由自带 rviz 配置显示 |
| COIN-LIO | `roslaunch coin_lio mapping_velodyne.launch` | `src/COIN-LIO/Robotpose/` | 由自带 rviz 配置显示 |
| VoxelMap | `roslaunch voxel_map mapping_velodyne.launch` | `src/VoxelMap/Robotpose/` | 由自带 rviz 配置显示 |
| GenZ-ICP | `roslaunch genz_icp odometry.launch` | `src/genz-icp/Robotpose/` | 由自带 rviz 配置显示 |
| DALI-SLAM | `roslaunch da_lio run_dalio_velodyne.launch` | `src/DALI_SLAM/Robotpose/` | `/path` |
| BIEVR-LIO | `roslaunch bievr_lio_ros process_topics.launch` | `src/BIEVR-LIO/Robotpose/` | `/bievr_lio/path` |

补充说明：

- LIO-SAM 导出的是关键帧位姿，其余方法导出的是逐帧位姿。
- DALI-SLAM 另在 `src/DALI_SLAM/DA_LIO/Log/` 下输出一份 TUM 格式轨迹，作为后端位姿图优化 MC-PGO 的输入。
- BIEVR-LIO 提供离线整包处理模式（不播放数据包，速度更快）：
  ```bash
  roslaunch bievr_lio_ros process_bag.launch rosbag:=<bag路径>
  ```

<details>
<summary>附：DALI-SLAM 后端位姿图优化（MC-PGO）</summary>

```bash
./devel/lib/backend_mapping/mc_pgo \
  <bag路径> src/DALI_SLAM/DA_LIO/Log/traj<时刻>.txt <外参.txt> \
  2 /velodyne_points 0 209 30 0.6 0
```

## 6. 输出格式

### 6.1 位姿 CSV

文件以整文件覆盖方式写出（非追加），列顺序固定为 11 列：

| 列名 | 含义 | 单位 |
|:--|:--|:--|
| `timestamp_sec` | 帧时间戳（雷达扫描结束时刻） | s |
| `x`, `y`, `z` | 机体（IMU）在参考系下的位置 | m |
| `qx`, `qy`, `qz`, `qw` | 姿态四元数 | — |
| `roll_deg`, `pitch_deg`, `yaw_deg` | 姿态的 ZYX 欧拉角 | deg |

### 6.2 终端运行统计

| 统计项 | 定义 |
|:--|:--|
| 平均单帧处理耗时 | 所有成功处理帧的耗时之和 / 帧数 |
| 处理帧率 | 1000 / 平均单帧处理耗时 |
| 平均有效观测点数 | 每次更新中实际参与残差构建的有效观测点数之平均值 |

## 7. 使用说明

- **关闭 rviz**：在启动命令后追加 `rviz:=false`，可减少计算开销。
- **更换序列**：更换 `rosbag play` 的数据包，并将对应方法 yaml 中的 CSV 文件名改为新序列名（仅此一处）。
- **更换雷达或话题**：各方法默认使用 `/velodyne_points` 与 `/imu/data`；BIEVR-LIO、COIN-LIO 支持在命令行直接覆盖，例如
  `roslaunch bievr_lio_ros process_topics.launch pointcloud_topic:=/my/points imu_topic:=/my/imu`
- **距离截断**：各方法 yaml 中均提供输入点云的最小/最大距离参数（本工作空间统一为 15 m，便于横向对比）。
- **位姿 CSV 路径**：各方法 yaml 中写的是相对路径，基准为该方法自身的源码目录（`src/<方法>/`），因此工作空间改名或移动后无需修改；写成绝对路径也同样支持。

## 8. 目录结构

```
Compare_SLAM/
├── src/
│   ├── LIO-SAM/  FAST_LIO2/  PV-LIO/  COIN-LIO/  VoxelMap/  genz-icp/
│   ├── DALI_SLAM/      # DA-LIO（前端）+ MC-PGO（后端）
│   ├── BIEVR-LIO/      # bievr_lio_ros（ROS1 接口）
│   ├── livox_ros_driver/  livox_ros_driver2/   # Livox 驱动（非 SLAM 算法）
│   └── ...
└── README.md           # 本文件
```

<details>
<summary>附：功能需求（原始说明）</summary>


</details>
