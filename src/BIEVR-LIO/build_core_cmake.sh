#!/usr/bin/env bash
# ============================================================================
# BIEVR-LIO 构建脚本(本机 ROS Noetic + GCC 10 + catkin_make 工作区专用)
#
# 背景:
#   BIEVR-LIO 由三个包组成 ——
#     BIEVR/                     -> bievr_lio        (核心库, plain-CMake)
#     interfaces/ros_common/     -> bievr_ros_common (头文件库, plain-CMake)
#     interfaces/ros1/           -> bievr_lio_ros    (ROS1 接口, catkin 包)
#   前两个是 <build_type>cmake</build_type> 的“非 catkin 包”, catkin_make 不能和
#   catkin 包混在同一个非隔离工作区里(会报 non-homogeneous workspace), 上游因此改用
#   catkin_tools。本工作区是 catkin_make 的, 为了不破坏现有布局, 这里:
#     1) 给这三个非 catkin 目录放 CATKIN_IGNORE, 让 catkin_make 跳过它们
#     2) 用 CMake 直接把核心两个包编译安装到工作区的 devel/ 里(头文件/库/cmake 配置)
#     3) 再用 catkin_make 正常编译 ROS1 接口包
#
#   devel/ 被清掉(rm -rf devel / catkin_make clean)之后, 重新跑本脚本即可。
#
# 用法:  bash src/BIEVR-LIO/build_core_cmake.sh
# ============================================================================
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="$WS/src/BIEVR-LIO"
BUILD="$WS/build/bievr_manual"
DEVEL="$WS/devel"
JOBS="$(nproc)"

echo "[1/3] 编译并安装 bievr_lio (核心库) -> $DEVEL"
cmake -S "$SRC/BIEVR" -B "$BUILD/core" \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DCMAKE_INSTALL_PREFIX="$DEVEL"
cmake --build "$BUILD/core" -j"$JOBS"
cmake --install "$BUILD/core"

echo "[2/3] 安装 bievr_ros_common (头文件库) -> $DEVEL"
cmake -S "$SRC/interfaces/ros_common" -B "$BUILD/ros_common" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$DEVEL" \
      -DCMAKE_PREFIX_PATH="$DEVEL"
cmake --install "$BUILD/ros_common"

echo "[3/3] catkin_make 编译 ROS1 接口 bievr_lio_ros"
cd "$WS"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
catkin_make --pkg bievr_lio_ros -j"$JOBS"

echo
echo "完成. 可执行文件:"
ls -l "$DEVEL/lib/bievr_lio_ros/"
