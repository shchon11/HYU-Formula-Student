#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────
#  캘리브레이션:  캡처 → 솔브 → 검증 → 적용까지 한 번에.
#
#   calib              전체 (센서 없으면 알아서 띄우고 끝나면 내림)
#   calib -c           캡처만
#   calib -s [세션]     기존 캡처로 솔브만 (생략 시 최신 세션)
#   calib -v [이미지]   투영 검증만 (생략 시 최신 이미지)
#   calib -l           extrinsic 목록 (active 는 * 표시)
#   calib -e <이름>     active 를 그 값으로 교체
#
#  fsk-shellrc 가 걸어주는 명령이라 어느 디렉토리에서든 그냥 'calib' 이면 된다.
#
#  캡처 창:  [SPACE]/[C] = 동기 쌍 저장,  [Q]/[ESC] = 종료
#  결과는 <워크스페이스>/extrinsics/<날짜_시각>.yaml 로 저장되고 active 가
#  그쪽을 가리킨다.
#
#  실행은 3-스텝 흐름:  fsk → stack → mission
#  (센서 브링업이 extrinsics/active 를 읽는다. 한 번만 다른 값으로 띄우려면
#   calib -e 로 바꾸지 말고  fsk extrinsic:=<경로>.yaml)
# ─────────────────────────────────────────────────────────────
set -o pipefail        # 주의: ROS setup.bash 가 미정의 변수를 참조하므로 -u 는 쓰지 않음

# 워크스페이스 루트: fsk-shellrc 가 export 한 EUFS_MASTER 를 우선 쓰고, 직접
# 실행됐으면 이 파일 위치(src/scripts)에서 두 단계 올라간다.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="${EUFS_MASTER:-$(cd -- "$SCRIPT_DIR/../.." && pwd)}"
EXT_DIR="$WS/extrinsics"
DEBUG_DIR="$WS/calib_debug"
K_YAML="$DEBUG_DIR/zed_K.yaml"
SOLVED="$DEBUG_DIR/extrinsic_zed_rslidar.yaml"    # calibrate 툴의 출력 경로 (고정)
LID_TOPIC=/sensors/lidar/points
CAM_RAW=/sensors/zed/left/color/rect/image         # 캡처는 raw Image 를 구독

usage() { sed -n '2,21p' "$0" | sed 's/^# \?//'; }

MODE=all
case "${1:-}" in
  -c|--capture) MODE=capture; shift ;;
  -s|--solve)   MODE=solve;   shift ;;
  -v|--verify)  MODE=verify;  shift ;;
  -l|--list)    MODE=list;    shift ;;
  -e|--use)     MODE=use;     shift ;;
  -h|--help)    usage; exit 0 ;;
  -*) echo "✗ 모르는 옵션: $1"; echo; usage; exit 1 ;;
esac

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
else
  echo "✗ $WS/install/setup.bash 없음 — colcon build 먼저"; exit 1
fi
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
mkdir -p "$DEBUG_DIR" "$EXT_DIR"
cd "$WS"                       # calibrate 의 상대 출력 경로를 워크스페이스에 고정

have_topic() { ros2 topic list --include-hidden-topics 2>/dev/null | grep -qx "$1"; }

resolve() {                    # 상대/절대 어디서 실행해도 되게
  if   [ -e "$1" ];      then realpath "$1"
  elif [ -e "$WS/$1" ];  then realpath "$WS/$1"
  else echo "$1"; fi
}

# sync_capture.py 는 자기 OUT_ROOT 에 저장한다(워크스페이스와 다를 수 있음) —
# 경로를 가정하지 말고 후보 루트에서 최신 세션을 찾는다.
export CAPTURE_OUT_ROOT="$WS/captures"
CAPTURE_ROOTS=("$WS/captures" "$HOME/ros2_ws/captures")

latest_session() {             # latest_session [이 epoch 이후에 생긴 것만]
  local newer="${1:-}" roots=() found
  for r in "${CAPTURE_ROOTS[@]}"; do [ -d "$r" ] && roots+=("$r"); done
  [ ${#roots[@]} -gt 0 ] || return 1
  if [ -n "$newer" ]; then
    found="$(find "${roots[@]}" -mindepth 1 -maxdepth 1 -type d -newermt "@$newer" \
             -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  else
    found="$(find "${roots[@]}" -mindepth 1 -maxdepth 1 -type d \
             -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -1 | cut -d' ' -f2-)"
  fi
  [ -n "$found" ] && { echo "$found"; return 0; }
  return 1
}

# ── 센서: 없으면 우리가 띄우고, 우리가 띄운 것만 내린다 ────
OWN_SENSORS=0; SENSOR_PID=""
cleanup() {
  trap - INT TERM EXIT
  if [ "$OWN_SENSORS" = 1 ] && [ -n "$SENSOR_PID" ]; then
    echo "▶ (우리가 띄운) 센서 종료 중…"
    kill -INT "$SENSOR_PID" 2>/dev/null
    for _ in $(seq 100); do kill -0 "$SENSOR_PID" 2>/dev/null || break; sleep 0.1; done
    kill -TERM "$SENSOR_PID" 2>/dev/null
    if pgrep -f 'component_container_isolated' >/dev/null 2>&1; then
      sleep 1; pkill -f 'component_container_isolated' 2>/dev/null || true
    fi
    wait 2>/dev/null
  fi
}
trap cleanup INT TERM EXIT

ensure_sensors() {
  if have_topic "$LID_TOPIC" && have_topic "$CAM_RAW"; then
    echo "▶ 센서 이미 실행 중 — 그대로 사용합니다."
    return 0
  fi
  if pgrep -f 'sensors.launch.py|zed_camera.launch|component_container_isolated' >/dev/null 2>&1; then
    echo "✗ 센서 프로세스는 떠 있는데 토픽이 안 보입니다 — 기동이 끝날 때까지 기다리거나"
    echo "  해당 터미널에서 Ctrl+C 후 다시 실행하세요."
    exit 1
  fi
  echo "▶ 센서 실행 (tf:=false — TF 없이 센서만)"
  ros2 launch hyu_sensor_bringup sensors.launch.py tf:=false gnss:=off &
  SENSOR_PID=$!; OWN_SENSORS=1
  for _ in $(seq 60); do
    have_topic "$LID_TOPIC" && have_topic "$CAM_RAW" && { echo "▶ 센서 준비 완료."; return 0; }
    kill -0 "$SENSOR_PID" 2>/dev/null || { echo "✗ 센서가 죽었습니다 — 위 로그 확인"; exit 1; }
    sleep 1
  done
  echo "✗ 60초 안에 센서 토픽이 안 떴습니다."
  echo "  라이다: $LID_TOPIC / 카메라: $CAM_RAW"
  exit 1
}

# ── 캡처 ───────────────────────────────────────────────────
do_capture() {                 # 성공 시 세션 경로를 SESSION 에 담는다
  local script="" c t0 n
  for c in "$WS/src/sensors/hyu_sensor_bringup/scripts/sync_capture.py" \
           "$WS/sync_capture.py" "$HOME/ros2_ws/sync_capture.py"; do
    [ -f "$c" ] && { script="$c"; break; }
  done
  [ -n "$script" ] || {
    echo "✗ sync_capture.py 를 못 찾았습니다. 다음 중 한 곳에 두세요:"
    echo "    $WS/src/sensors/hyu_sensor_bringup/scripts/sync_capture.py"; exit 1; }

  t0="$(date +%s)"
  echo "▶ 캡처 창 실행:  [SPACE]=저장   [Q]=종료   ($script)"
  python3 "$script" || { echo "✗ 캡처 실패"; exit 1; }

  SESSION="$(latest_session "$t0")" || {
    echo "✗ 새 캡처 세션을 찾지 못했습니다 (한 장도 저장 안 됨?)"; exit 1; }
  n="$(ls "$SESSION"/*.pcd 2>/dev/null | wc -l)"
  [ "$n" -gt 0 ] || { echo "✗ $SESSION 에 저장된 쌍이 없습니다."; exit 1; }
  echo "▶ 캡처 완료: $SESSION  ($n 쌍)"
}

# ── 솔브 ───────────────────────────────────────────────────
do_solve() {                   # do_solve <세션…>
  local tag
  echo "▶ 카메라 인트린식 확보 중…"
  if ros2 run charuco_lidar_calib grab_camera_info --out "$K_YAML" --timeout 6 >/dev/null 2>&1; then
    echo "  ✓ 라이브 grab 성공: $K_YAML"
  elif [ -f "$K_YAML" ]; then
    echo "  ⚠ 센서가 안 떠 있어 기존 파일 사용: $K_YAML"
    echo "    (카메라를 교체/재보정했다면 센서 켜고 다시 실행할 것)"
  else
    echo "✗ 인트린식 없음: 센서를 켜고 다시 실행하세요."; exit 1
  fi

  echo "▶ calibrate 실행: $*"
  ros2 run charuco_lidar_calib calibrate "$@" --camera-info "$K_YAML" || {
    echo "✗ 캘리브레이션 실패 — 위 로그의 WEAK/skip 항목을 확인하세요."; exit 1; }
  [ -f "$SOLVED" ] || { echo "✗ 결과 파일이 없습니다: $SOLVED"; exit 1; }

  # 결과를 extrinsics/ 로 들여놓고 active 를 그쪽으로 — 이게 곧 적용이다
  tag="$(date +%Y-%m-%d_%H%M)"
  cp "$SOLVED" "$EXT_DIR/$tag.yaml"
  ln -sfn "$tag.yaml" "$EXT_DIR/active"
  echo
  echo "▶ 저장: $EXT_DIR/$tag.yaml   (active → $tag.yaml)"
  python3 - "$EXT_DIR/$tag.yaml" <<'PY'
import sys, yaml
e = yaml.safe_load(open(sys.argv[1])) or {}
lc = e.get('lidar_to_camera') or {}
t = lc.get('t') or [0, 0, 0]
m = e.get('metrics') or {}
print(f"  TF  {lc.get('parent_frame','?')} -> {lc.get('child_frame','?')}")
print(f"  t = ({t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}) m")
print(f"  metrics: plane={m.get('rmse_plane_mm','-')}mm  normal={m.get('normal_deg_rms','-')}deg"
      f"  poses={m.get('n_poses','-')}  cond={m.get('translation_condition','-')}")
PY
}

# ── 검증 (라이다 → 이미지 투영) ────────────────────────────
do_verify() {                  # do_verify [이미지…]
  local ext caminfo=() imgs=() outs=() a img base pcd out sess latest
  ext="$(realpath "$EXT_DIR/active" 2>/dev/null)"
  [ -f "$ext" ] || { echo "⚠ active extrinsic 없음 — 검증 생략"; return 0; }
  [ -f "$K_YAML" ] && caminfo=(--camera-info "$K_YAML")

  if [ $# -ge 1 ]; then
    for a in "$@"; do imgs+=("$(resolve "$a")"); done
  else
    sess="${SESSION:-$(latest_session)}"
    [ -n "$sess" ] || { echo "⚠ 캡처 세션이 없어 검증 생략"; return 0; }
    latest="$(ls -t "$sess"/*.png 2>/dev/null | grep -v '_view\.png\|_R\.png' | head -1)"
    [ -n "$latest" ] || { echo "⚠ $sess 에 이미지가 없어 검증 생략"; return 0; }
    imgs=("$latest")
    echo "▶ 검증 이미지: $latest"
  fi

  for img in "${imgs[@]}"; do
    [ -f "$img" ] || { echo "  ✗ 파일 없음: $img"; continue; }
    base="${img%.png}"; base="${base%_L}"; base="${base%_R}"
    pcd="$base.pcd"
    [ -f "$pcd" ] || { echo "  ✗ 짝 pcd 없음: $pcd"; continue; }
    out="$DEBUG_DIR/fusion_$(basename "$base").png"
    ros2 run charuco_lidar_calib verify "$img" --pcd "$pcd" \
         --extrinsic "$ext" "${caminfo[@]}" --out "$out" && outs+=("$out")
  done
  [ ${#outs[@]} -ge 1 ] || { echo "⚠ 생성된 투영 이미지 없음"; return 0; }
  echo "▶ 투영 결과: ${outs[*]}"
  command -v xdg-open >/dev/null && xdg-open "${outs[0]}" >/dev/null 2>&1 &
}

# ── extrinsic 목록/선택 ────────────────────────────────────
list_extrinsics() {
  python3 - "$EXT_DIR" <<'PY'
import os, sys, yaml
d = sys.argv[1]
link = os.path.join(d, 'active')
active = os.path.realpath(link) if os.path.exists(link) else None
rows = []
for f in sorted(os.listdir(d)):
    if not f.endswith('.yaml'):
        continue
    p = os.path.join(d, f)
    try:
        e = yaml.safe_load(open(p)) or {}
    except Exception:
        continue
    t = (e.get('lidar_to_camera') or {}).get('t') or [0, 0, 0]
    m = e.get('metrics') or {}
    rows.append(('*' if os.path.realpath(p) == active else ' ', f[:-5], t, m))
if not rows:
    print('  (없음 — ./calib.sh 먼저)')
    sys.exit(0)
w = max(len(r[1]) for r in rows)
print(f"    {'이름'.ljust(w)}   {'t = (x, y, z) [m]':<30} {'plane':>8} {'poses':>6}")
for mark, name, t, m in rows:
    tt = f"({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f})"
    r = m.get('rmse_plane_mm')
    r = f"{r:.1f}mm" if isinstance(r, (int, float)) else '-'
    print(f"  {mark} {name.ljust(w)}   {tt:<30} {r:>8} {str(m.get('n_poses','-')):>6}")
PY
}

# ── 모드별 실행 ────────────────────────────────────────────
SESSION=""
case "$MODE" in
  list)
    echo "▶ $EXT_DIR"
    list_extrinsics
    exit 0
    ;;
  use)
    [ $# -ge 1 ] || { echo "✗ 이름이 필요합니다. 예: ./calib.sh -e 2026-07-26_0223"; echo; list_extrinsics; exit 1; }
    NEW=""
    for c in "$EXT_DIR/$1" "$EXT_DIR/$1.yaml" "$1"; do
      [ -f "$c" ] && { NEW="$(realpath "$c")"; break; }
    done
    [ -n "$NEW" ] || { echo "✗ 없는 extrinsic: $1"; echo; list_extrinsics; exit 1; }
    ln -sfn "$(basename "$NEW")" "$EXT_DIR/active"
    echo "▶ active → $(basename "$NEW")"
    list_extrinsics
    exit 0
    ;;
  capture)
    ensure_sensors
    do_capture
    echo "▶ 솔브: ./calib.sh -s $SESSION"
    ;;
  solve)
    if [ $# -ge 1 ]; then
      SESS=(); for a in "$@"; do SESS+=("$(resolve "$a")"); done
    else
      SESSION="$(latest_session)" || { echo "✗ 캡처 세션이 없습니다 — ./calib.sh -c 먼저"; exit 1; }
      echo "▶ 세션 미지정 → 최신 사용: $SESSION"
      SESS=("$SESSION")
    fi
    do_solve "${SESS[@]}"
    do_verify
    echo "▶ 적용됨. 실행:  ./run.sh"
    ;;
  verify)
    do_verify "$@"
    ;;
  all)
    ensure_sensors
    do_capture
    do_solve "$SESSION"
    do_verify
    echo
    echo "▶ 완료. 실행:  ./run.sh        (다른 값으로 실행: ./run.sh -l 로 목록 확인)"
    ;;
esac
