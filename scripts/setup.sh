#!/usr/bin/env bash
# ============================================================================
# fsk one-shot bootstrap — set up the whole workspace on a FRESH machine.
#
#   bash <workspace>/src/scripts/setup.sh            # full setup
#   bash .../setup.sh --skip-gazebo                  # skip the ~30 min patched
#                                                    #   gazebo source build
#   FSK_JOBS=2 bash .../setup.sh                     # cap build parallelism more
#
# Idempotent: re-running skips anything already done. Safe on a machine that is
# already partly set up. Needs sudo (apt / swap); it will prompt as usual.
#
# This exists because following the README by hand hits avoidable traps on a new
# box — a missing `tmux`, ultralytics' opencv-python metadata dep crashing
# perception, OOM-freezes during the build, a loopback with no MULTICAST flag
# hanging discovery. Every one of those is handled below. See README + the
# per-step comments for the "why".
# ============================================================================
set -uo pipefail

# --- locate workspace (this file lives in <ws>/src/scripts) ------------------
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd -- "$HERE/.." && pwd)"          # .../src
WS="$(cd -- "$SRC_DIR/.." && pwd)"            # workspace root
JOBS="${FSK_JOBS:-4}"                         # per-package make jobs (OOM guard)
SKIP_GAZEBO=0
for a in "$@"; do [ "$a" = "--skip-gazebo" ] && SKIP_GAZEBO=1; done

say()  { printf '\n\033[1;36m== %s ==\033[0m\n' "$*"; }
warn() { printf '\033[1;33m!! %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m   %s\033[0m\n' "$*"; }

export EUFS_MASTER="$WS"
export COMMONROAD_CLCS_DIR="${COMMONROAD_CLCS_DIR:-$HOME/commonroad-clcs}"
PY=/usr/bin/python3                            # ROS/system interpreter (never conda)

# --- 0. sanity ---------------------------------------------------------------
say "workspace: $WS   (jobs=$JOBS, skip_gazebo=$SKIP_GAZEBO)"
[ -f /opt/ros/humble/setup.bash ] || { warn "ROS 2 Humble not found at /opt/ros/humble — install it first"; exit 1; }
if [ "$(id -u)" = 0 ]; then warn "run as your normal user, not root"; exit 1; fi

# --- 1. swap (OOM guard for the build on ≤16 GB RAM, no swap) -----------------
# g2o + CLCS/Eigen templates + patched-gazebo source all compile in parallel and
# have frozen low-RAM machines hard. An 8 GB swapfile is cheap insurance.
say "swap"
MEM_GB=$(awk '/MemTotal/{printf "%d", $2/1024/1024}' /proc/meminfo)
if [ "$(swapon --show=NAME --noheadings | wc -l)" -gt 0 ]; then
  ok "swap already present"
elif [ "$MEM_GB" -le 24 ]; then
  warn "no swap and only ${MEM_GB} GB RAM — adding an 8 GB /swapfile"
  sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile && sudo mkswap /swapfile >/dev/null && sudo swapon /swapfile
  grep -q '/swapfile' /etc/fstab || echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab >/dev/null
  ok "8 GB swap added (persists via /etc/fstab)"
else
  ok "${MEM_GB} GB RAM — swap not needed"
fi

# --- 2. apt system deps ------------------------------------------------------
# NOTE: includes tmux (race.sh needs it — the README table omitted it) and
# gazebo-ros-pkgs. rosdep (step 3) fills anything package.xml-specific.
say "apt system dependencies"
sudo apt-get update -y
sudo apt-get install -y \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  gazebo ros-humble-gazebo-dev ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-gazebo-ros-pkgs \
  ros-humble-sbg-driver ros-humble-libg2o ros-humble-rviz-2d-overlay-plugins \
  ros-humble-ackermann-msgs python3-sklearn python3-pandas python3-opencv \
  libeigen3-dev libboost-dev libspdlog-dev libomp-dev tmux
ok "apt deps installed"

# --- 3. rosdep ---------------------------------------------------------------
say "rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init 2>/dev/null || true
rosdep update
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
rosdep install --from-paths "$SRC_DIR" --ignore-src -r -y || warn "rosdep reported issues (often the harmless ament_python key) — continuing"
ok "rosdep done"

# --- 4. external source: CommonRoad-CLCS (hyu_frenet_conversion compiles it) --
say "commonroad-clcs"
if [ -d "$COMMONROAD_CLCS_DIR/.git" ]; then ok "already cloned at $COMMONROAD_CLCS_DIR"
else git clone --depth 1 https://github.com/CommonRoad/commonroad-clcs.git "$COMMONROAD_CLCS_DIR" && ok "cloned"; fi

# --- 5. Python (system interpreter — race strips conda/.venv) -----------------
say "python packages (system $PY)"
"$PY" -m pip install --user quadprog "numpy<2" >/dev/null && ok "quadprog + numpy<2"
if "$PY" -c 'import torch, torchvision' 2>/dev/null; then
  ok "torch/torchvision already present ($("$PY" -c 'import torch;print(torch.__version__)')) — not touching it"
else
  warn "installing torch+torchvision (CUDA 12.4 wheels; pick your CUDA if different)"
  "$PY" -m pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cu124
fi
"$PY" -m pip install --user ultralytics "numpy<2" >/dev/null && ok "ultralytics"
# ultralytics drags opencv-python back in; the workspace uses the SYSTEM cv2 for
# cv_bridge ABI. Remove the pip one...
"$PY" -m pip uninstall -y opencv-python >/dev/null 2>&1 || true
# ...but ultralytics DECLARES opencv-python, and hyu_perception's console-script
# runs pkg_resources.require() at launch → DistributionNotFound crash. perception
# never uses cv2 (np.frombuffer), so a stub dist-info satisfies the check without
# shadowing system cv2. (See README Setup 2c / Troubleshooting.)
STUB="$("$PY" -c 'import site;print(site.getusersitepackages())')/opencv_python-4.11.0.86.dist-info"
if [ -f "$STUB/METADATA" ]; then ok "opencv-python stub already present"
else
  mkdir -p "$STUB"
  printf 'Metadata-Version: 2.1\nName: opencv-python\nVersion: 4.11.0.86\n' > "$STUB/METADATA"
  : > "$STUB/RECORD"
  ok "opencv-python stub created (system cv2 stays the real one)"
fi
"$PY" -c 'import cv2,ultralytics,numpy,pkg_resources as p; p.require("ultralytics"); print("   verify: cv2",cv2.__version__,"ultralytics",ultralytics.__version__,"numpy",numpy.__version__)' \
  || warn "python verify failed — check the messages above"

# --- 6. shell rc: source fsk-shellrc -----------------------------------------
# fsk-shellrc sets EUFS_MASTER/CLCS, sources the workspace, auto-activates patched
# gazebo, caps OpenMP threads, and picks ROS_LOCALHOST_ONLY by whether lo has
# MULTICAST — the machine-adaptive bits that stop CPU melt / discovery hangs.
say "shell rc"
LINE="source $SRC_DIR/scripts/fsk-shellrc"
for RC in "$HOME/.zshrc" "$HOME/.bashrc"; do
  [ -f "$RC" ] || continue
  if grep -qF "fsk-shellrc" "$RC"; then ok "$(basename "$RC") already sources it"
  else printf '\n# HYU Formula Student autonomous stack\n%s\n' "$LINE" >> "$RC"; ok "added to $(basename "$RC")"; fi
done

# --- 7. build ----------------------------------------------------------------
# Capped parallelism: full-core parallel builds freeze low-RAM machines (step 1).
say "colcon build (this takes a while)"
( cd "$WS" && MAKEFLAGS="-j$JOBS" colcon build --symlink-install --base-paths src --parallel-workers 2 ) \
  && ok "workspace built" || { warn "build failed — see log above. Re-run this script; the build resumes incrementally."; exit 1; }

# --- 8. patched gazebo (optional, long: GPU LiDAR without sky-dome phantoms) --
if [ "$SKIP_GAZEBO" = 1 ]; then
  warn "skipping patched gazebo (--skip-gazebo). Sim still runs on stock gazebo (with gpu_ray sky-dome phantoms). Build later: bash $SRC_DIR/tools/gazebo-patches/build-patched-gazebo.sh"
elif [ -f "$HOME/opt/gazebo11-fsk/bin/gzserver" ]; then
  say "patched gazebo"; ok "already built at ~/opt/gazebo11-fsk"
else
  say "patched gazebo (source build — can take ~30 min; --skip-gazebo to skip)"
  # source build needs gazebo's build-deps → enable deb-src, then build-dep
  if ! grep -rqE '^deb-src .*ubuntu.* (main|universe)' /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
    . /etc/os-release 2>/dev/null || true
    echo "deb-src http://archive.ubuntu.com/ubuntu ${VERSION_CODENAME:-jammy} main universe" | sudo tee /etc/apt/sources.list.d/fsk-gazebo-src.list >/dev/null
    sudo apt-get update -y
  fi
  sudo apt-get build-dep -y gazebo || warn "apt build-dep gazebo failed — patched build may be missing headers"
  JOBS="$JOBS" MAKEFLAGS="-j$JOBS" nice -n 10 bash "$SRC_DIR/tools/gazebo-patches/build-patched-gazebo.sh" \
    && ok "patched gazebo installed to ~/opt/gazebo11-fsk" \
    || warn "patched gazebo build failed — re-run resumes incrementally, or use --skip-gazebo"
fi

say "DONE"
cat <<EOF
   Open a NEW terminal (or: source ~/.zshrc) so fsk-shellrc is loaded, then:
     race sim     # Gazebo simulated cones — light, no YOLO
     race         # full CUDA YOLO + LiDAR perception
   Verify a clean run: the monitor pane shows 'path_source: LOCAL' and the car drives.
EOF
