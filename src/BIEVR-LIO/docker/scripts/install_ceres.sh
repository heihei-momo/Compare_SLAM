#!/usr/bin/env bash
set -euo pipefail

# Commit/branch/tag to build; defaults to your commit.
CERES_REF="2.2.0"

echo "Installing Ceres @ ${CERES_REF}"

# Install into /usr/local, which is on CMake's default search path
# (CMAKE_SYSTEM_PREFIX_PATH) and the default linker path, so find_package(Ceres)
# works out of the box without Ceres_DIR / CMAKE_PREFIX_PATH.
install_prefix=/usr/local

# Fetch the exact ref robustly (works for commits, tags, or branches)
src_root=/opt/ceres
rm -rf "${src_root}"
git init "${src_root}"
git -C "${src_root}" remote add origin https://github.com/ceres-solver/ceres-solver.git
# Try shallow first; fall back to full fetch if needed
if ! git -C "${src_root}" fetch --no-tags --depth 1 origin "${CERES_REF}" 2>/dev/null; then
  git -C "${src_root}" fetch --no-tags origin "${CERES_REF}"
fi
git -C "${src_root}" checkout -q FETCH_HEAD
# Pull submodules (Abseil, GTest, etc.)
git -C "${src_root}" submodule update --init --recursive

# Configure & build
mkdir -p "${src_root}/build"
gen=""
command -v ninja >/dev/null 2>&1 && gen="-G Ninja"

cmake -S "${src_root}" -B "${src_root}/build" ${gen} \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_INSTALL_PREFIX="${install_prefix}" \
  \
  -DBUILD_TESTING=OFF \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_BENCHMARKS=OFF \
  -DBUILD_DOCUMENTATION=OFF \
  -DSUITESPARSE=ON \
  -DEIGENSPARSE=ON \
  -DEIGENMETIS=ON \
  -DUSE_CUDA=OFF \
  -DLAPACK=ON \
  -DSCHUR_SPECIALIZATIONS=ON \
  -DCUSTOM_BLAS=ON

cmake --build "${src_root}/build" --parallel "$(nproc)"
cmake --install "${src_root}/build"

# Refresh the linker cache so the freshly installed libs are found at runtime.
# (/usr/local/lib is already on the default linker path, so no extra config.)
ldconfig

echo "Ceres installed to ${install_prefix} @ $(git -C "${src_root}" rev-parse --short HEAD)"

# Clean up source and build tree
rm -rf "${src_root}"
