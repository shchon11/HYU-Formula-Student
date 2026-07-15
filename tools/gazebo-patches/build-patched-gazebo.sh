#!/usr/bin/env bash
# Build a patched Gazebo Classic 11.10.2 into a sudo-free prefix.
#
#   bash tools/gazebo-patches/build-patched-gazebo.sh
#
# Env overrides: GZ_SRC (default ~/gazebo-classic-fsk), GZ_PREFIX (default
# ~/opt/gazebo11-fsk), GZ_TAG (default gazebo11_11.10.2 — keep it equal to the
# installed gzserver version so the ros-humble-gazebo-* plugins stay
# ABI-compatible), JOBS (default nproc).
#
# What it does: clone (if needed) -> patch_gazebo.py -> cmake Release build
# -> install. See README.md for what the patches change and how to verify.
set -euo pipefail

SRC="${GZ_SRC:-$HOME/gazebo-classic-fsk}"
PREFIX="${GZ_PREFIX:-$HOME/opt/gazebo11-fsk}"
TAG="${GZ_TAG:-gazebo11_11.10.2}"
JOBS="${JOBS:-$(nproc)}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ ! -d "$SRC/.git" ]; then
  git clone --depth 1 --branch "$TAG" \
    https://github.com/gazebosim/gazebo-classic "$SRC"
fi

python3 "$HERE/patch_gazebo.py" "$SRC"

# Tests are not compiled by default on 11.10.2 (needs an explicit 'make tests';
# the old ENABLE_TESTS_COMPILATION flag is deprecated and errors out).
# DART is disabled: jammy's libdart-dev ships headers that #include the
# missing dart/external/ikfast package, so gazebo_physics fails to compile
# against it — and this workspace only ever runs the (default) ODE engine.
cmake -S "$SRC" -B "$SRC/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_DISABLE_FIND_PACKAGE_DART:BOOL=TRUE
cmake --build "$SRC/build" -- -j"$JOBS"

# Without ruby/ronn the gz man page is never generated but the install
# manifest still lists it; give install the (empty) file it wants.
grep -rhoE '"[^"]+\.1\.gz"' "$SRC/build" --include=cmake_install.cmake \
  | tr -d '"' | sort -u | while read -r f; do
    [ -f "$f" ] || touch "$f"
  done

cmake --install "$SRC/build" >/dev/null

cat <<EOF

Patched Gazebo installed to: $PREFIX

Activate for a sim run (prepend to the launch environment):

  source $PREFIX/share/gazebo/setup.sh
  export PATH="$PREFIX/bin:\$PATH"
  export LD_LIBRARY_PATH="$PREFIX/lib:\${LD_LIBRARY_PATH:-}"
  # optional accuracy knob (default 2048 = stock geometry):
  export GAZEBO_GPU_LASER_TEX_MIN=4096

A patched gzserver identifies itself once per gpu_ray sensor:

  [Msg] GpuRaySensor [...]: cameras=3 first-pass tex=4096x1268 (GAZEBO_GPU_LASER_TEX_MIN=4096)
EOF
