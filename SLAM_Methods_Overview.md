# 对比的 LiDAR-Inertial SLAM 方法简介

本工作空间收集了 8 个开源的 LiDAR 里程计 / LiDAR-惯性 SLAM 方法。选型的出发点是覆盖当前主流但技术路线不同的几类做法：以**效率与通用性**为主的直接法里程计、面向**退化与信息稀疏环境**的鲁棒方法、面向**测绘级建图**的系统，以及利用**强度信息**与**几何退化分析**的补充方案。因此它们各自"针对什么场景"并不相同，横向比较时需要把这些设计目标差异考虑进去。

---

## 1. 论文与发表信息

| 方法 | 论文全称 | 发表期刊 / 会议（年份） | 影响因子（JCR，近年） | 本工作空间包名 |
|:--|:--|:--|:--|:--|
| **LIO-SAM** | LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping | IEEE/RSJ International Conference on Intelligent Robots and Systems (**IROS**), 2020, pp. 5135–5142 | 会议，无 IF（机器人领域顶级会议，CCF-A） | `lio_sam` |
| **FAST-LIO2** | FAST-LIO2: Fast Direct LiDAR-Inertial Odometry | IEEE Transactions on Robotics (**T-RO**), 2022, 38(4) | **6~7**（Q1，Robotics） | `fast_lio` |
| **PV-LIO** | —（未见正式论文，开源实现） | 开源项目（基于 VoxelMap + FAST-LIO / IKFoM） | — | `pv_lio` |
| **COIN-LIO** | COIN-LIO: Complementary Intensity-Augmented LiDAR Inertial Odometry | IEEE International Conference on Robotics and Automation (**ICRA**), 2024 | 会议，无 IF（机器人领域顶级会议，CCF-A） | `coin_lio` |
| **VoxelMap** | Efficient and Probabilistic Adaptive Voxel Mapping for Accurate Online LiDAR Odometry | IEEE Robotics and Automation Letters (**RA-L**), 2022（arXiv:2109.07082） | **4.5~5**（Q1，Robotics） | `voxel_map` |
| **GenZ-ICP** | GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting | IEEE Robotics and Automation Letters (**RA-L**), 2024（DOI: 10.1109/LRA.2024.3498779） | **4.5~5**（Q1，Robotics） | `genz_icp` |
| **DALI-SLAM** | DALI-SLAM: Degeneracy-Aware LiDAR-inertial SLAM with novel distortion correction and accurate multi-constraint pose graph optimization | ISPRS Journal of Photogrammetry and Remote Sensing, 2025, 221: 92–108 | **10~12**（Q1，遥感 / 摄影测量） | `da_lio` + `backend_mapping` |
| **BIEVR-LIO** | BIEVR-LIO: Robust LiDAR-Inertial Odometry through Bump-Image-Enhanced Voxel Maps | Robotics: Science and Systems (**RSS**), 2026（arXiv:2604.14421） | 会议，无 IF（机器人领域顶级会议） | `bievr_lio_ros` |

> 说明：
> - **影响因子只对期刊有意义**，且逐年浮动；表中给的是近年 JCR 区间与分区（Q1），具体数值请以最新 JCR 为准。
> - ICRA / IROS / RSS 均为**机器人领域顶级会议**（ICRA、IROS 属 CCF-A），不参与影响因子排名，故标注为"会议，无 IF"。
> - PV-LIO 未检索到正式论文，按开源实现列出，其"出处"信息来自项目源码与 README。

---

## 2. 各方法核心思想与针对场景

| 方法 | 核心思想 | 主要针对场景 | 传感器需求 |
|:--|:--|:--|:--|
| **LIO-SAM** | 因子图优化（GTSAM）：IMU 预积分 + 激光里程计 + GPS + 回环四类因子紧耦合 | 大尺度、多传感器、实时建图；车载/地面机器人；有 GPS 与回环条件的场景 | 3D LiDAR + IMU（推荐 9 轴）+ 可选 GPS |
| **FAST-LIO2** | 迭代扩展卡尔曼滤波 + ikd-Tree 增量地图，**直接法**（无需提取特征）扫描匹配 | 通用型：快速运动、杂乱或含退化场景；无人机/手持/车载多平台；多种雷达即插即用 | 3D LiDAR（需逐点时间）+ 6 轴 IMU |
| **PV-LIO** | 以 VoxelMap 为局部地图，按雷达测距模型推导点-面匹配协方差作为置信度引导 KF 更新，并支持在线 LiDAR-IMU 外参标定 | 退化场景（如**狭窄楼梯间**）、外参不确定或需在线标定的平台 | 3D LiDAR + IMU |
| **COIN-LIO** | 在几何残差之外引入**强度图像的光度误差**，与点云几何互补 | 几何信息不足但**强度纹理丰富**的环境（长走廊/隧道内壁等）；需要反射率数据的雷达 | 带强度/反射率的 LiDAR + IMU |
| **VoxelMap** | 以**平面**为地图表示单元的自适应（粗到细）概率体素地图，每个平面带参数与不确定性 | 复杂/非结构化环境的高精度在线里程计；作为其它系统的地图后端 | 3D LiDAR + IMU |
| **GenZ-ICP** | 在点到面与点到点（ICP/GICP）之间**自适应加权**的广义 ICP | 泛化性与退化鲁棒性：开阔地、隧道等平面约束不足场景；换雷达无需重新调参 | 仅 3D LiDAR（可选逐点时间去畸变，**不使用 IMU**） |
| **DALI-SLAM** | 双样条运动畸变校正 + 退化感知 ESKF（分析雅可比最小特征值并重映射更新）+ 多约束位姿图优化（DA-LIO + MC-PGO） | 大尺度 GNSS 拒止环境的**头盔/手持测绘**；退化频繁的室内外场景 | 3D LiDAR + IMU |
| **BIEVR-LIO** | 体素级**有向高度图（bump image）**地图表示，直接在表示上配准；地图引导的点采样聚焦几何信息量大的区域 | **信息稀疏**环境（隧道等）的鲁棒里程计；兼顾下游高程建图 | 3D LiDAR（需逐点时间）+ IMU |

---

## 3. 按场景归类

### 3.1 通用型：效率优先、多平台多雷达
**FAST-LIO2、VoxelMap、PV-LIO**
三者同源（HKU MARS 的 VoxelMap / FAST-LIO / IKFoM 技术栈），都以"高效 + 通用"为目标：FAST-LIO2 用 ikd-Tree 增量地图与直接法匹配保证实时性；VoxelMap 把地图换成带不确定性的平面体素；PV-LIO 再把 VoxelMap 与 IMU 紧耦合并加入在线外参标定。适合作为大多数场景的**基线**。

### 3.2 退化与信息稀疏环境（隧道、长走廊、楼梯、开阔地）
**DALI-SLAM、BIEVR-LIO、GenZ-ICP、PV-LIO**，以及用强度补充几何的 **COIN-LIO**
- DALI-SLAM：显式做**退化检测**（雅可比最小特征值）并在 ESKF 更新中做重映射；
- BIEVR-LIO：用高分辨率有向高度图挖掘**细微几何变化**，并靠地图引导采样聚焦有效区域；
- GenZ-ICP：用平面/非平面自适应权重避免在平面约束不足时被错误约束主导；
- PV-LIO：用测距协方差作为置信度，在窄楼梯等场景抑制不可靠约束；
- COIN-LIO：几何退化时靠**强度纹理**提供额外约束。

### 3.3 面向测绘与大场景建图
**LIO-SAM**（GPS + 回环 + 地图保存）、**DALI-SLAM**（头盔扫描、后端多约束位姿图）、**BIEVR-LIO**（可输出高程图）。这类方法更关心**全局一致性**与成果质量，而非单纯里程计精度。

### 3.4 强度（反射率）信息的使用
**COIN-LIO** 是唯一把强度当作**观测残差**使用的方法；其余方法（FAST-LIO2、VoxelMap、PV-LIO、DALI-SLAM、BIEVR-LIO）把强度仅作为点云的一个通道输出，不参与优化。

### 3.5 无 IMU 的纯激光基线
**GenZ-ICP** 仅用 LiDAR（去畸变靠逐点时间字段），在对比中可作为"**不依赖 IMU** 的下界参考"，也适合 IMU 质量差或时间同步困难的平台。

---

## 4. 各方法简介

### 4.1 LIO-SAM
**论文**：LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping，IROS 2020（pp. 5135–5142，会议论文，无 IF）。
把 LiDAR 里程计、IMU 预积分、GPS 与回环检测统一到 GTSAM 因子图中做平滑与建图，支持 9 轴 IMU 提供 roll/pitch 初值，可在有 GPS 时消除漂移。**场景定位**：大尺度、多传感器、需要保存全局点云地图的地面平台。缺 GPS 或回环条件时退化为激光惯性里程计，长期漂移仍存在。

### 4.2 FAST-LIO2
**论文**：FAST-LIO2: Fast Direct LiDAR-Inertial Odometry，IEEE Transactions on Robotics (T-RO) 2022, 38(4)（期刊 IF 近年 6~7，Q1）。
用 ikd-Tree 维护增量式地图，采用**直接法**把原始点云配准到地图（省去特征提取），以紧耦合迭代 EKF 估计状态。**场景定位**：快速运动、杂乱环境、含退化的场景，以及无人机/手持/车载等多平台；支持旋转式与固态雷达。是本工作空间里最"通用"的基线。

### 4.3 PV-LIO
**论文**：暂无正式论文（开源实现，基于 VoxelMap 与 FAST-LIO / IKFoM）。
以 VoxelMap 作为局部地图管理器：依据雷达测距模型推导每个"点-面"对应的协方差，作为置信度参与 KF 更新；推导包含 LiDAR-IMU 外参的协方差传播，从而支持在线外参标定；地图更新做了并行优化。**场景定位**：几何约束较弱的退化场景（示例为**狭窄楼梯**），以及外参不精确、需要在线估计的平台。

### 4.4 COIN-LIO
**论文**：COIN-LIO: Complementary Intensity-Augmented LiDAR Inertial Odometry，ICRA 2024（会议论文，无 IF；arXiv:2310.01235）。
把 LiDAR 的强度/反射率按方位-俯仰投影成图像，引入**光度误差**与几何残差联合优化，使几何退化但纹理可辨的环境（走廊、隧道内壁）仍可获得约束。**场景定位**：几何信息不足、但强度纹理丰富的环境。**前提**：雷达需提供强度，且图像化投影依赖雷达的行列参数。

### 4.5 VoxelMap
**论文**：Efficient and Probabilistic Adaptive Voxel Mapping for Accurate Online LiDAR Odometry，IEEE Robotics and Automation Letters (RA-L) 2022（期刊 IF 近年 4.5~5，Q1；arXiv:2109.07082）。
地图不再存点，而是自适应地在体素内拟合**平面**并估计其不确定性；配准时使用面参数及其协方差。**场景定位**：复杂/非结构化环境的高精度在线里程计；也可作为其它系统的地图后端。地图更紧凑、精度较高，但对体素划分与平面假设有一定依赖。

### 4.6 GenZ-ICP
**论文**：GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting，IEEE Robotics and Automation Letters (RA-L) 2024（期刊 IF 近年 4.5~5，Q1；DOI: 10.1109/LRA.2024.3498779）。
在点到面与点到点残差之间**自适应加权**，相当于让广义 ICP 自己决定"更相信平面约束还是原始点约束"。**场景定位**：跨传感器泛化与退化鲁棒（开阔地、隧道）；本工作空间中作为**纯 LiDAR（无 IMU）**的对照方法。

### 4.7 DALI-SLAM
**论文**：DALI-SLAM: Degeneracy-Aware LiDAR-inertial SLAM with novel distortion correction and accurate multi-constraint pose graph optimization，ISPRS Journal of Photogrammetry and Remote Sensing 2025, 221: 92–108（期刊 IF 近年 10~12，Q1）。
前端 DA-LIO：用双样条拟合连续时间轨迹做运动畸变校正（DS-MDC），并根据优化系统雅可比的最小特征值判断**退化**、在 ESKF 更新时做重映射；后端 MC-PGO：用三类子图约束做鲁棒位姿图优化。**场景定位**：头盔/手持平台在大尺度 GNSS 拒止环境下的高精度测绘，退化频繁的室内外场景。

### 4.8 BIEVR-LIO
**论文**：BIEVR-LIO: Robust LiDAR-Inertial Odometry through Bump-Image-Enhanced Voxel Maps，Robotics: Science and Systems (RSS) 2026（会议论文，无 IF；arXiv:2604.14421）。
地图以体素为单位存储**有向高度图像（bump image）**，直接在高度图上配准而无需先计算平面等中间几何基元；并提出地图引导的点采样策略，把配准算力集中在几何信息量大的区域。**场景定位**：信息稀疏/退化环境（隧道等）的鲁棒里程计；同时支持下游高程建图（用于足式机器人地形感知）。

---

## 5. 在本工作空间中对比时的注意事项

1. **已统一的接口**：所有方法都补上了"输入点云距离截断（yaml 参数）→ 退出时输出 11 列位姿 CSV（相对路径，基准为该方法的源码目录）→ 终端打印平均单帧耗时与平均有效观测点数 → rviz 显示轨迹"这套一致的实验接口，便于同一套脚本处理。
2. **输出粒度不同**：LIO-SAM 输出的是**关键帧**位姿，其余方法输出**逐帧**位姿，比较轨迹点数时需注意。
3. **世界坐标系命名不同**：各方法分别使用 `map` / `odom` / `camera_init` 等名称，rviz 里查看或做可视化对比时要按各自配置设置固定坐标系。
4. **退化处理思路不同**：DALI-SLAM 是显式检测 + 更新重映射，BIEVR-LIO 是靠地图表示与采样，GenZ-ICP 是靠残差加权，PV-LIO 是靠协方差置信度，COIN-LIO 是靠强度信息 —— 结果相近但机理不同，是这份对比的主要看点之一。
5. **DALI-SLAM 的后端**：`da_lio` 只是前端里程计，完整精度需再离线运行 `backend_mapping`（MC-PGO）做多约束位姿图优化。
