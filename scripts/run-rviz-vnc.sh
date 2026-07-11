#!/usr/bin/env bash
set -Eeuo pipefail
IFS=$'\n\t'
umask 077

PROGRAM_NAME="eufs-vnc"
VERSION="1.0.0"

SSH_HOST="${EUFS_SSH_HOST:-lab}"
REMOTE_ROOT="${EUFS_REMOTE_ROOT:-/home/dohyun/FS/HYU-Formula-Student}"
REMOTE_SCRIPT="${EUFS_REMOTE_SCRIPT:-${REMOTE_ROOT}/scripts/run-rviz-vnc.sh}"
REMOTE_STATE_DIR="${EUFS_REMOTE_STATE_DIR:-/home/dohyun/.cache/eufs-vnc}"
SESSION_NAME="${EUFS_TMUX_SESSION:-eufs-vnc-managed}"
DISPLAY_NUM="${EUFS_DISPLAY_NUM:-99}"
REMOTE_VNC_PORT="${EUFS_REMOTE_VNC_PORT:-5901}"
LOCAL_VNC_PORT="${EUFS_LOCAL_VNC_PORT:-5901}"
TRACK="${EUFS_TRACK:-small_track}"
CONDA_PYTHON="${EUFS_CONDA_PYTHON:-/home/dohyun/anaconda3/envs/eufs/bin/python3}"
VNC_PASSWORD_FILE="${EUFS_VNC_PASSWORD_FILE:-/home/dohyun/.vnc/passwd}"
START_TIMEOUT="${EUFS_START_TIMEOUT:-240}"
NO_OPEN="${EUFS_NO_OPEN:-0}"
SCREEN_GEOMETRY="${EUFS_SCREEN_GEOMETRY:-1920x1080x24}"
OPERATION_LOCKED="${EUFS_OPERATION_LOCKED:-0}"

LOCAL_STATE_DIR="${EUFS_LOCAL_STATE_DIR:-${HOME}/.cache/eufs-vnc}"
CONTROL_SOCKET="${LOCAL_STATE_DIR}/ssh-control-${LOCAL_VNC_PORT}"
LOCAL_LOCK_DIR="${LOCAL_STATE_DIR}/operation.lock"
LOCAL_LOCK_HELD=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
SCRIPT_PATH="${SCRIPT_DIR}/$(basename "${BASH_SOURCE[0]}")"

say() {
    printf '[%s] %s\n' "${PROGRAM_NAME}" "$*"
}

warn() {
    printf '[%s] WARN: %s\n' "${PROGRAM_NAME}" "$*" >&2
}

die() {
    printf '[%s] ERROR: %s\n' "${PROGRAM_NAME}" "$*" >&2
    exit 1
}

usage() {
    cat <<'USAGE'
Usage: eufs-vnc.sh <command> [options]

Commands:
  start             Start or reuse EUFS, create the SSH tunnel, and open VNC
  stop              Stop only the managed EUFS stack and its SSH tunnel
  restart           Stop and start the managed stack
  status            Show remote stack and local tunnel status
  logs [LINES]      Show recent manager, launch, VNC, and Xvfb logs (default 120)
  attach            Attach to the remote manager tmux session
  doctor            Check local and remote prerequisites
  setup-password    Securely create/update the remote x11vnc password
  open              Repair the tunnel if needed and open macOS Screen Sharing
  version           Print the script version
  help, --help      Show this help

Common environment overrides:
  EUFS_SSH_HOST=lab
  EUFS_TRACK=small_track
  EUFS_LOCAL_VNC_PORT=5901
  EUFS_REMOTE_VNC_PORT=5901
  EUFS_DISPLAY_NUM=99
  EUFS_NO_OPEN=1             Do not open Screen Sharing after start

The default perception interpreter is:
  /home/dohyun/anaconda3/envs/eufs/bin/python3
USAGE
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"
}

validate_name() {
    local label="$1"
    local value="$2"
    if [[ ! "${value}" =~ ^[A-Za-z0-9_][A-Za-z0-9_.@%+=,-]*$ ]] || [[ "${value}" == *..* ]]; then
        die "${label} contains unsupported characters: ${value}"
    fi
}

validate_absolute_path() {
    local label="$1"
    local value="$2"
    if [[ ! "${value}" =~ ^/[A-Za-z0-9_./@%+=,-]+$ ]]; then
        die "${label} must be a safe absolute path: ${value}"
    fi
    case "${value}" in
        *//*|*/../*|*/..|*/./*|*/.) die "${label} must not contain path traversal: ${value}" ;;
    esac
}

validate_uint() {
    local label="$1"
    local value="$2"
    local minimum="$3"
    local maximum="$4"
    local numeric
    if [[ ! "${value}" =~ ^[1-9][0-9]*$ ]] || (( ${#value} > ${#maximum} )); then
        die "${label} must be an integer from ${minimum} to ${maximum}: ${value}"
    fi
    numeric=$((10#${value}))
    if (( numeric < minimum || numeric > maximum )); then
        die "${label} must be an integer from ${minimum} to ${maximum}: ${value}"
    fi
}

validate_config() {
    validate_name "EUFS_SSH_HOST" "${SSH_HOST}"
    validate_name "EUFS_TMUX_SESSION" "${SESSION_NAME}"
    validate_name "EUFS_TRACK" "${TRACK}"
    validate_absolute_path "EUFS_REMOTE_ROOT" "${REMOTE_ROOT}"
    validate_absolute_path "EUFS_REMOTE_SCRIPT" "${REMOTE_SCRIPT}"
    validate_absolute_path "EUFS_REMOTE_STATE_DIR" "${REMOTE_STATE_DIR}"
    validate_absolute_path "EUFS_CONDA_PYTHON" "${CONDA_PYTHON}"
    validate_absolute_path "EUFS_VNC_PASSWORD_FILE" "${VNC_PASSWORD_FILE}"
    validate_absolute_path "EUFS_LOCAL_STATE_DIR" "${LOCAL_STATE_DIR}"
    case "${REMOTE_ROOT}" in /home/dohyun/*) ;; *) die "EUFS_REMOTE_ROOT must stay under /home/dohyun" ;; esac
    case "${REMOTE_SCRIPT}" in "${REMOTE_ROOT}/scripts/"*.sh) ;; *) die "EUFS_REMOTE_SCRIPT must stay under ${REMOTE_ROOT}/scripts" ;; esac
    case "${REMOTE_STATE_DIR}" in /home/dohyun/.cache/eufs-vnc|/home/dohyun/.cache/eufs-vnc/*) ;; *) die "EUFS_REMOTE_STATE_DIR must stay under /home/dohyun/.cache/eufs-vnc" ;; esac
    case "${CONDA_PYTHON}" in /home/dohyun/anaconda3/envs/eufs/bin/*) ;; *) die "EUFS_CONDA_PYTHON must use the eufs Conda environment" ;; esac
    case "${VNC_PASSWORD_FILE}" in /home/dohyun/.vnc/*) ;; *) die "EUFS_VNC_PASSWORD_FILE must stay under /home/dohyun/.vnc" ;; esac
    case "${LOCAL_STATE_DIR}" in "${HOME}"|"${HOME}/"*) ;; *) die "EUFS_LOCAL_STATE_DIR must stay under the current home directory" ;; esac
    if [[ ! "${SCREEN_GEOMETRY}" =~ ^[0-9]+x[0-9]+x(16|24|32)$ ]]; then
        die "EUFS_SCREEN_GEOMETRY must look like 1920x1080x24"
    fi
    validate_uint "EUFS_DISPLAY_NUM" "${DISPLAY_NUM}" 1 999
    validate_uint "EUFS_REMOTE_VNC_PORT" "${REMOTE_VNC_PORT}" 1024 65535
    validate_uint "EUFS_LOCAL_VNC_PORT" "${LOCAL_VNC_PORT}" 1024 65535
    validate_uint "EUFS_START_TIMEOUT" "${START_TIMEOUT}" 60 900
    if [[ "${NO_OPEN}" != "0" && "${NO_OPEN}" != "1" ]]; then
        die "EUFS_NO_OPEN must be 0 or 1"
    fi
    if [[ "${OPERATION_LOCKED}" != "0" && "${OPERATION_LOCKED}" != "1" ]]; then
        die "EUFS_OPERATION_LOCKED must be 0 or 1"
    fi
}

cleanup_local_lock() {
    if [[ "${LOCAL_LOCK_HELD}" == "1" ]]; then
        rm -f "${LOCAL_LOCK_DIR}/pid" 2>/dev/null || true
        rmdir "${LOCAL_LOCK_DIR}" 2>/dev/null || true
        LOCAL_LOCK_HELD=0
    fi
}

trap cleanup_local_lock EXIT

ensure_local_state_dir() {
    [[ ! -L "${LOCAL_STATE_DIR}" ]] || die "Local state path must not be a symlink: ${LOCAL_STATE_DIR}"
    if [[ -e "${LOCAL_STATE_DIR}" ]]; then
        [[ -d "${LOCAL_STATE_DIR}" && -O "${LOCAL_STATE_DIR}" ]] || die "Local state path must be an owned directory: ${LOCAL_STATE_DIR}"
    else
        mkdir -m 700 "${LOCAL_STATE_DIR}"
    fi
    chmod 700 "${LOCAL_STATE_DIR}"
}

acquire_local_lock() {
    local owner_pid=""
    ensure_local_state_dir

    if mkdir "${LOCAL_LOCK_DIR}" 2>/dev/null; then
        printf '%s\n' "$$" > "${LOCAL_LOCK_DIR}/pid"
        LOCAL_LOCK_HELD=1
        return 0
    fi

    if [[ -r "${LOCAL_LOCK_DIR}/pid" ]]; then
        owner_pid="$(sed -n '1p' "${LOCAL_LOCK_DIR}/pid")"
    fi
    if [[ "${owner_pid}" =~ ^[0-9]+$ ]] && kill -0 "${owner_pid}" 2>/dev/null; then
        die "Another ${PROGRAM_NAME} operation is running (PID ${owner_pid})"
    fi

    rm -f "${LOCAL_LOCK_DIR}/pid" 2>/dev/null || true
    rmdir "${LOCAL_LOCK_DIR}" 2>/dev/null || true
    mkdir "${LOCAL_LOCK_DIR}" || die "Could not acquire local operation lock"
    printf '%s\n' "$$" > "${LOCAL_LOCK_DIR}/pid"
    LOCAL_LOCK_HELD=1
}

build_remote_command() {
    local action="$1"
    shift
    local q_root q_script q_state q_session q_display q_remote_port
    local q_track q_python q_password q_timeout q_geometry q_locked q_action q_arg
    local command

    printf -v q_root '%q' "${REMOTE_ROOT}"
    printf -v q_script '%q' "${REMOTE_SCRIPT}"
    printf -v q_state '%q' "${REMOTE_STATE_DIR}"
    printf -v q_session '%q' "${SESSION_NAME}"
    printf -v q_display '%q' "${DISPLAY_NUM}"
    printf -v q_remote_port '%q' "${REMOTE_VNC_PORT}"
    printf -v q_track '%q' "${TRACK}"
    printf -v q_python '%q' "${CONDA_PYTHON}"
    printf -v q_password '%q' "${VNC_PASSWORD_FILE}"
    printf -v q_timeout '%q' "${START_TIMEOUT}"
    printf -v q_geometry '%q' "${SCREEN_GEOMETRY}"
    printf -v q_locked '%q' "${OPERATION_LOCKED}"
    printf -v q_action '%q' "${action}"

    command="/usr/bin/env EUFS_REMOTE_ROOT=${q_root} EUFS_REMOTE_SCRIPT=${q_script} EUFS_REMOTE_STATE_DIR=${q_state} EUFS_TMUX_SESSION=${q_session} EUFS_DISPLAY_NUM=${q_display} EUFS_REMOTE_VNC_PORT=${q_remote_port} EUFS_TRACK=${q_track} EUFS_CONDA_PYTHON=${q_python} EUFS_VNC_PASSWORD_FILE=${q_password} EUFS_START_TIMEOUT=${q_timeout} EUFS_SCREEN_GEOMETRY=${q_geometry} EUFS_OPERATION_LOCKED=${q_locked} /bin/bash ${q_script} __remote ${q_action}"
    for q_arg in "$@"; do
        printf -v q_arg '%q' "${q_arg}"
        command="${command} ${q_arg}"
    done
    printf '%s' "${command}"
}

remote_exec() {
    local command
    command="$(build_remote_command "$@")"
    ssh "${SSH_HOST}" "${command}"
}

remote_script_exists() {
    local q_script state
    printf -v q_script '%q' "${REMOTE_SCRIPT}"
    if ! state="$(ssh "${SSH_HOST}" "if test -x ${q_script}; then printf present; elif test -e ${q_script}; then printf invalid; else printf absent; fi")"; then
        die "SSH failed while checking the remote manager"
    fi
    case "${state}" in
        present) return 0 ;;
        absent) return 1 ;;
        invalid) die "Remote manager exists but is not executable: ${REMOTE_SCRIPT}" ;;
        *) die "Unexpected remote manager state: ${state}" ;;
    esac
}

sync_remote_script() {
    local remote_dir="${REMOTE_SCRIPT%/*}"
    local upload_template="${remote_dir}/.eufs-vnc-upload.XXXXXX"
    local q_dir q_script q_state q_backup q_template upload_command
    local local_checksum remote_checksum

    printf -v q_dir '%q' "${remote_dir}"
    printf -v q_script '%q' "${REMOTE_SCRIPT}"
    printf -v q_state '%q' "${REMOTE_STATE_DIR}"
    printf -v q_backup '%q' "${REMOTE_STATE_DIR}/legacy-script.sh"
    printf -v q_template '%q' "${upload_template}"

    upload_command="set -eu; umask 077; test -d ${q_dir}; test ! -L ${q_dir}; test \"\$(stat -c %u ${q_dir})\" = \"\$(id -u)\"; if test -L ${q_state}; then exit 1; fi; if test -e ${q_state}; then test -d ${q_state}; else mkdir -m 700 -- ${q_state}; fi; chmod 700 -- ${q_state}; test \"\$(stat -c %u ${q_state})\" = \"\$(id -u)\"; if test -L ${q_script}; then exit 1; fi; if test -e ${q_script}; then test -f ${q_script}; fi; if test -L ${q_backup}; then exit 1; fi; if test -e ${q_backup}; then test -f ${q_backup}; elif test -f ${q_script}; then cp -p -- ${q_script} ${q_backup}; fi; if test -f ${q_backup}; then chmod 600 -- ${q_backup}; fi; temp=\$(mktemp ${q_template}); trap 'rm -f -- \"\$temp\"' EXIT HUP INT TERM; cat > \"\$temp\"; chmod 700 -- \"\$temp\"; mv -f -- \"\$temp\" ${q_script}; trap - EXIT HUP INT TERM"
    ssh "${SSH_HOST}" "${upload_command}" < "${SCRIPT_PATH}"

    local_checksum="$(cksum "${SCRIPT_PATH}" | awk '{print $1 ":" $2}')"
    remote_checksum="$(ssh "${SSH_HOST}" "cksum ${q_script}" | awk '{print $1 ":" $2}')"
    if [[ "${local_checksum}" != "${remote_checksum}" ]]; then
        die "Remote script checksum mismatch after upload"
    fi
    say "Remote manager synchronized (${remote_checksum})"
}

local_preflight() {
    require_command ssh
    require_command cksum
    require_command awk
    require_command lsof
    ssh -G "${SSH_HOST}" >/dev/null 2>&1 || die "SSH host alias is invalid: ${SSH_HOST}"
}

tunnel_is_running() {
    [[ -S "${CONTROL_SOCKET}" ]] || return 1
    ssh -S "${CONTROL_SOCKET}" -O check "${SSH_HOST}" >/dev/null 2>&1
}

local_port_is_busy() {
    lsof -nP -iTCP:"${LOCAL_VNC_PORT}" -sTCP:LISTEN >/dev/null 2>&1
}

start_tunnel() {
    ensure_local_state_dir

    if tunnel_is_running; then
        say "SSH tunnel already running on 127.0.0.1:${LOCAL_VNC_PORT}"
        return 0
    fi

    if [[ -e "${CONTROL_SOCKET}" ]]; then
        rm -f "${CONTROL_SOCKET}"
    fi

    if local_port_is_busy; then
        lsof -nP -iTCP:"${LOCAL_VNC_PORT}" -sTCP:LISTEN >&2 || true
        die "Local port ${LOCAL_VNC_PORT} is owned by another process"
    fi

    ssh -M -S "${CONTROL_SOCKET}" -fnNT \
        -o ExitOnForwardFailure=yes \
        -o ServerAliveInterval=30 \
        -o ServerAliveCountMax=3 \
        -L "127.0.0.1:${LOCAL_VNC_PORT}:127.0.0.1:${REMOTE_VNC_PORT}" \
        "${SSH_HOST}"

    tunnel_is_running || die "SSH tunnel did not become healthy"
    say "SSH tunnel ready: 127.0.0.1:${LOCAL_VNC_PORT} -> ${SSH_HOST}:127.0.0.1:${REMOTE_VNC_PORT}"
}

stop_tunnel() {
    if tunnel_is_running; then
        ssh -S "${CONTROL_SOCKET}" -O exit "${SSH_HOST}" >/dev/null 2>&1 || true
        say "SSH tunnel stopped"
    fi
    if [[ -e "${CONTROL_SOCKET}" ]]; then
        rm -f "${CONTROL_SOCKET}"
    fi
}

open_vnc() {
    if [[ "$(uname -s)" != "Darwin" ]]; then
        warn "Screen Sharing auto-open is only available on macOS"
        return 0
    fi
    require_command open
    open "vnc://127.0.0.1:${LOCAL_VNC_PORT}"
    say "Opened Screen Sharing at vnc://127.0.0.1:${LOCAL_VNC_PORT}"
}

start_impl() {
    local created_tunnel=0
    local_preflight
    sync_remote_script
    if ! tunnel_is_running; then
        start_tunnel
        created_tunnel=1
    else
        say "SSH tunnel already running on 127.0.0.1:${LOCAL_VNC_PORT}"
    fi
    if ! remote_exec start; then
        if [[ "${created_tunnel}" == "1" ]]; then
            stop_tunnel
        fi
        return 1
    fi
    if [[ "${NO_OPEN}" == "0" ]]; then
        open_vnc
    fi
}

stop_impl() {
    local_preflight
    stop_tunnel
    if remote_script_exists; then
        remote_exec stop
    else
        say "Remote manager is not deployed; nothing remote to stop"
    fi
}

status_impl() {
    local remote_rc=0
    local_preflight
    if remote_script_exists; then
        remote_exec status || remote_rc=$?
    else
        printf 'remote=not-deployed\n'
        remote_rc=1
    fi

    if tunnel_is_running; then
        printf 'tunnel=running\nlocal_vnc=127.0.0.1:%s\n' "${LOCAL_VNC_PORT}"
    elif local_port_is_busy; then
        printf 'tunnel=unowned-port-conflict\n'
        remote_rc=1
    else
        printf 'tunnel=stopped\n'
    fi
    return "${remote_rc}"
}

logs_impl() {
    local lines="${1:-120}"
    validate_uint "LINES" "${lines}" 1 5000
    local_preflight
    remote_script_exists || die "Remote manager is not deployed"
    remote_exec logs "${lines}"
}

attach_impl() {
    local command
    local_preflight
    remote_script_exists || die "Remote manager is not deployed"
    command="$(build_remote_command attach)"
    exec ssh -t "${SSH_HOST}" "${command}"
}

doctor_impl() {
    local_preflight
    say "Local prerequisites OK: ssh, cksum, awk, lsof, host ${SSH_HOST}"
    sync_remote_script
    remote_exec doctor
}

setup_password_impl() {
    local q_password q_dir
    local password_dir="${VNC_PASSWORD_FILE%/*}"
    local_preflight
    printf -v q_password '%q' "${VNC_PASSWORD_FILE}"
    printf -v q_dir '%q' "${password_dir}"
    ssh -t "${SSH_HOST}" "umask 077; mkdir -p ${q_dir}; chmod 700 ${q_dir}; x11vnc -storepasswd ${q_password}; chmod 600 ${q_password}"
    say "VNC password stored securely at ${SSH_HOST}:${VNC_PASSWORD_FILE}"
}

open_impl() {
    local_preflight
    remote_script_exists || die "Remote manager is not deployed"
    remote_exec status >/dev/null || die "Remote stack is not ready; run '$0 start' first"
    start_tunnel
    open_vnc
}

remote_init_paths() {
    PHASE_FILE="${REMOTE_STATE_DIR}/phase"
    READY_FILE="${REMOTE_STATE_DIR}/ready"
    FAILURE_FILE="${REMOTE_STATE_DIR}/failure"
    SUPERVISOR_PID_FILE="${REMOTE_STATE_DIR}/supervisor.pid"
    XVFB_PID_FILE="${REMOTE_STATE_DIR}/xvfb.pid"
    VNC_PID_FILE="${REMOTE_STATE_DIR}/vnc.pid"
    LAUNCH_PID_FILE="${REMOTE_STATE_DIR}/launch.pid"
    LOCK_FILE="${REMOTE_STATE_DIR}/operation.lock"
    MANAGER_LOG="${REMOTE_STATE_DIR}/manager.log"
    LAUNCH_LOG="${REMOTE_STATE_DIR}/launch.log"
    XVFB_LOG="${REMOTE_STATE_DIR}/xvfb.log"
    VNC_LOG="${REMOTE_STATE_DIR}/vnc.log"
    HEALTH_LOG="${REMOTE_STATE_DIR}/health.log"
    SPAWN_LOG="${REMOTE_STATE_DIR}/spawn.log"
    PERCEPTION_WRAPPER="${REMOTE_STATE_DIR}/eufs-perception-python"
    DISPLAY_NAME=":${DISPLAY_NUM}"
}

ensure_remote_state_dir() {
    [[ ! -L "${REMOTE_STATE_DIR}" ]] || die "Remote state path must not be a symlink: ${REMOTE_STATE_DIR}"
    if [[ -e "${REMOTE_STATE_DIR}" ]]; then
        [[ -d "${REMOTE_STATE_DIR}" && -O "${REMOTE_STATE_DIR}" ]] || die "Remote state path must be an owned directory: ${REMOTE_STATE_DIR}"
    else
        mkdir -m 700 -- "${REMOTE_STATE_DIR}"
    fi
    chmod 700 -- "${REMOTE_STATE_DIR}"
}

remote_require() {
    command -v "$1" >/dev/null 2>&1 || die "Remote command not found: $1"
}

remote_source_ros() {
    [[ -r /opt/ros/galactic/setup.bash ]] || die "Missing /opt/ros/galactic/setup.bash"
    [[ -r "${REMOTE_ROOT}/install/setup.bash" ]] || die "Workspace is not built: ${REMOTE_ROOT}/install/setup.bash"
    set +u
    source /opt/ros/galactic/setup.bash
    source "${REMOTE_ROOT}/install/setup.bash"
    set -u
    export ROS_LOCALHOST_ONLY=1
    export EUFS_MASTER="${REMOTE_ROOT}"
    export DISPLAY="${DISPLAY_NAME}"
    export LIBGL_ALWAYS_SOFTWARE=1
    export QT_X11_NO_MITSHM=1
}

write_phase() {
    local phase="$1"
    printf '%s\n' "${phase}" > "${PHASE_FILE}.tmp"
    mv -f "${PHASE_FILE}.tmp" "${PHASE_FILE}"
    printf '[%s] phase=%s\n' "$(date '+%F %T')" "${phase}" | tee -a "${MANAGER_LOG}"
}

record_failure() {
    local message="$1"
    printf '%s\n' "${message}" > "${FAILURE_FILE}"
    printf '[%s] ERROR: %s\n' "$(date '+%F %T')" "${message}" | tee -a "${MANAGER_LOG}" >&2
}

supervisor_fail() {
    record_failure "$*"
    write_phase "failed"
    exit 1
}

managed_session_exists() {
    local pane_command
    tmux has-session -t "${SESSION_NAME}" 2>/dev/null || return 1
    pane_command="$(tmux list-panes -t "${SESSION_NAME}" -F '#{pane_start_command}' 2>/dev/null | sed -n '1p')"
    [[ "${pane_command}" == *"${REMOTE_SCRIPT}"*"__remote supervise"* ]]
}

session_name_exists() {
    tmux has-session -t "${SESSION_NAME}" 2>/dev/null
}

read_pid_file() {
    local file="$1"
    local pid=""
    if [[ -r "${file}" ]]; then
        pid="$(sed -n '1p' "${file}")"
    fi
    if [[ "${pid}" =~ ^[0-9]+$ ]]; then
        printf '%s' "${pid}"
        return 0
    fi
    return 1
}

proc_start_time() {
    local pid="$1"
    awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null
}

proc_group_id() {
    local pid="$1"
    awk '{print $5}' "/proc/${pid}/stat" 2>/dev/null
}

record_pid_identity() {
    local file="$1"
    local pid="$2"
    local start_time group_id temp
    start_time="$(proc_start_time "${pid}")" || return 1
    group_id="$(proc_group_id "${pid}")" || return 1
    [[ "${start_time}" =~ ^[0-9]+$ ]] || return 1
    [[ "${group_id}" =~ ^[0-9]+$ ]] || return 1
    temp="$(mktemp "${file}.XXXXXX")" || return 1
    if ! printf '%s\n%s\n%s\n' "${pid}" "${start_time}" "${group_id}" > "${temp}"; then
        rm -f -- "${temp}"
        return 1
    fi
    chmod 600 -- "${temp}"
    mv -f -- "${temp}" "${file}"
}

pid_identity_matches() {
    local file="$1"
    local pid recorded_start current_start recorded_group current_group
    pid="$(read_pid_file "${file}")" || return 1
    recorded_start="$(sed -n '2p' "${file}" 2>/dev/null)"
    recorded_group="$(sed -n '3p' "${file}" 2>/dev/null)"
    [[ "${recorded_start}" =~ ^[0-9]+$ ]] || return 1
    [[ "${recorded_group}" =~ ^[0-9]+$ ]] || return 1
    current_start="$(proc_start_time "${pid}")" || return 1
    current_group="$(proc_group_id "${pid}")" || return 1
    [[ "${current_start}" == "${recorded_start}" && "${current_group}" == "${recorded_group}" ]]
}

pid_matches() {
    local file="$1"
    local expected="$2"
    local pid
    pid_identity_matches "${file}" || return 1
    pid="$(read_pid_file "${file}")" || return 1
    tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null | grep -Fq -- "${expected}"
}

vnc_listener_ready() {
    local line local_address found=0
    while IFS= read -r line; do
        [[ -n "${line}" ]] || continue
        local_address="$(printf '%s\n' "${line}" | awk '{print $4}')"
        case "${local_address}" in
            "127.0.0.1:${REMOTE_VNC_PORT}"|"[::1]:${REMOTE_VNC_PORT}") found=1 ;;
            *) return 1 ;;
        esac
    done < <(ss -H -ltn "( sport = :${REMOTE_VNC_PORT} )" 2>/dev/null)
    (( found == 1 ))
}

remote_quick_health() {
    [[ -f "${READY_FILE}" ]] || return 1
    managed_session_exists || return 1
    pid_matches "${XVFB_PID_FILE}" "Xvfb ${DISPLAY_NAME}" || return 1
    pid_matches "${VNC_PID_FILE}" "x11vnc" || return 1
    pid_matches "${VNC_PID_FILE}" "-localhost" || return 1
    pid_matches "${VNC_PID_FILE}" "-rfbauth ${VNC_PASSWORD_FILE}" || return 1
    pid_matches "${LAUNCH_PID_FILE}" "ros2 launch eufs_launcher" || return 1
    vnc_listener_ready || return 1
}

remote_status() {
    local phase="stopped"
    local pid=""

    if [[ -r "${PHASE_FILE}" ]]; then
        phase="$(sed -n '1p' "${PHASE_FILE}")"
    fi

    if remote_quick_health; then
        printf 'remote=running\nphase=%s\ndisplay=%s\nremote_vnc=127.0.0.1:%s\nsession=%s\n' \
            "${phase}" "${DISPLAY_NAME}" "${REMOTE_VNC_PORT}" "${SESSION_NAME}"
        pid="$(read_pid_file "${LAUNCH_PID_FILE}" || true)"
        printf 'launch_pid=%s\n' "${pid}"
        return 0
    fi

    if managed_session_exists; then
        printf 'remote=starting-or-unhealthy\nphase=%s\nsession=%s\n' "${phase}" "${SESSION_NAME}"
        if [[ -r "${FAILURE_FILE}" ]]; then
            printf 'failure=%s\n' "$(sed -n '1p' "${FAILURE_FILE}")"
        fi
        return 2
    fi

    if session_name_exists; then
        printf 'remote=unowned-session-conflict\nsession=%s\n' "${SESSION_NAME}"
        return 3
    fi

    printf 'remote=stopped\nphase=%s\n' "${phase}"
    return 1
}

remote_doctor() {
    local missing=0
    local command_name
    local password_mode

    ensure_remote_state_dir
    for command_name in bash tmux Xvfb x11vnc xdpyinfo glxinfo setsid flock timeout ss awk mktemp stat gz gzserver gzclient; do
        if ! command -v "${command_name}" >/dev/null 2>&1; then
            printf 'missing_command=%s\n' "${command_name}" >&2
            missing=1
        fi
    done
    (( missing == 0 )) || die "Remote prerequisites are missing"

    [[ -x "${CONDA_PYTHON}" ]] || die "Conda eufs Python is missing: ${CONDA_PYTHON}"
    [[ -r /lib/x86_64-linux-gnu/libffi.so.7 ]] || die "System libffi.so.7 is missing"
    [[ -r "${VNC_PASSWORD_FILE}" ]] || die "VNC password is missing; run '$0 setup-password'"
    chmod 600 "${VNC_PASSWORD_FILE}"
    password_mode="$(stat -c '%a' "${VNC_PASSWORD_FILE}")"
    [[ "${password_mode}" == "600" ]] || die "VNC password permissions must be 600"

    remote_source_ros
    remote_require ros2
    remote_require rviz2
    [[ -r "${REMOTE_ROOT}/install/eufs_racecar/share/eufs_racecar/robots/eufs/robot.urdf" ]] || die "Spawn URDF is missing"

    printf 'remote_doctor=ok\nros_distro=%s\nworkspace=%s\nconda_env=eufs\nvnc_password_mode=%s\n' \
        "${ROS_DISTRO:-unknown}" "${REMOTE_ROOT}" "${password_mode}"
    if ! ros2 pkg prefix eufs_rviz_plugins >/dev/null 2>&1; then
        printf 'warning_optional_rviz_plugin=eufs_rviz_plugins/WaypointArrayStamped unavailable\n'
    fi
    if ! ldconfig -p 2>/dev/null | grep -q 'libhector_gazebo_ros_magnetic.so'; then
        printf 'warning_optional_sensor_plugin=libhector_gazebo_ros_magnetic.so unavailable\n'
    fi
}

rotate_remote_logs() {
    local file
    for file in "${MANAGER_LOG}" "${LAUNCH_LOG}" "${XVFB_LOG}" "${VNC_LOG}" "${HEALTH_LOG}" "${SPAWN_LOG}"; do
        if [[ -f "${file}" ]]; then
            mv -f "${file}" "${file}.previous"
        fi
        : > "${file}"
    done
}

wait_for_recorded_exit() {
    local file="$1"
    local expected="$2"
    local seconds="$3"
    local i
    for (( i=0; i<seconds*10; i++ )); do
        pid_matches "${file}" "${expected}" || return 0
        sleep 0.1
    done
    return 1
}

stop_launch_group() {
    local file="$1"
    local expected="ros2 launch eufs_launcher"
    local pid group_id
    pid_matches "${file}" "${expected}" || return 0
    pid="$(read_pid_file "${file}")"
    group_id="$(proc_group_id "${pid}")" || return 1
    [[ "${group_id}" == "${pid}" ]] || return 1
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    wait_for_recorded_exit "${file}" "${expected}" 10 && return 0
    pid_matches "${file}" "${expected}" || return 0
    pid="$(read_pid_file "${file}")"
    [[ "$(proc_group_id "${pid}")" == "${pid}" ]] || return 1
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    wait_for_recorded_exit "${file}" "${expected}" 8 && return 0
    pid_matches "${file}" "${expected}" || return 0
    pid="$(read_pid_file "${file}")"
    [[ "$(proc_group_id "${pid}")" == "${pid}" ]] || return 1
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
}

stop_recorded_process() {
    local file="$1"
    local expected="$2"
    local wait_seconds="${3:-5}"
    local pid
    pid_matches "${file}" "${expected}" || return 0
    pid="$(read_pid_file "${file}")"
    kill "${pid}" 2>/dev/null || true
    wait_for_recorded_exit "${file}" "${expected}" "${wait_seconds}" && return 0
    pid_matches "${file}" "${expected}" || return 0
    pid="$(read_pid_file "${file}")"
    kill -KILL "${pid}" 2>/dev/null || true
}

force_stop_recorded_processes() {
    stop_launch_group "${LAUNCH_PID_FILE}" || warn "Launch process identity changed before cleanup"
    stop_recorded_process "${VNC_PID_FILE}" "x11vnc" || warn "x11vnc process identity changed before cleanup"
    stop_recorded_process "${XVFB_PID_FILE}" "Xvfb ${DISPLAY_NAME}" || warn "Xvfb process identity changed before cleanup"
}

recorded_processes_alive() {
    pid_matches "${LAUNCH_PID_FILE}" "ros2 launch eufs_launcher" \
        || pid_matches "${VNC_PID_FILE}" "x11vnc" \
        || pid_matches "${XVFB_PID_FILE}" "Xvfb ${DISPLAY_NAME}"
}

supervisor_cleanup() {
    local rc=$?
    trap - EXIT INT TERM HUP
    rm -f "${READY_FILE}"

    force_stop_recorded_processes
    if recorded_processes_alive; then
        rc=1
        record_failure "Managed process cleanup did not complete"
    else
        rm -f "${SUPERVISOR_PID_FILE}" "${XVFB_PID_FILE}" "${VNC_PID_FILE}" "${LAUNCH_PID_FILE}"
    fi
    if (( rc == 0 )); then
        write_phase "stopped"
    elif [[ ! -r "${FAILURE_FILE}" ]]; then
        record_failure "Supervisor exited with code ${rc}"
        write_phase "failed"
    fi
    exit "${rc}"
}

launch_is_alive() {
    pid_matches "${LAUNCH_PID_FILE}" "ros2 launch eufs_launcher"
}

joint_states_ready() {
    ros2 topic info /eufs/joint_states 2>/dev/null | grep -Eq 'Publisher count: [1-9][0-9]*'
}

wait_for_joint_states() {
    local seconds="$1"
    local i
    for (( i=0; i<seconds; i++ )); do
        launch_is_alive || supervisor_fail "ROS launch exited while waiting for joint states"
        joint_states_ready && return 0
        sleep 1
    done
    return 1
}

topic_has_header_sample() {
    local topic="$1"
    local message_type="$2"
    local seconds="$3"
    local sample
    sample="$(timeout "${seconds}" ros2 topic echo "${topic}" "${message_type}" --qos-profile sensor_data --no-arr --no-str 2>/dev/null | grep -m1 '^header:' || true)"
    [[ "${sample}" == "header:" ]]
}

remote_supervise() {
    local xvfb_pid vnc_pid launch_pid ros2_bin
    local i service_ready=0 robot_present=0 gz_camera_topic="" tf_output=""

    ensure_remote_state_dir
    rotate_remote_logs
    rm -f "${READY_FILE}" "${FAILURE_FILE}" "${SUPERVISOR_PID_FILE}" "${XVFB_PID_FILE}" "${VNC_PID_FILE}" "${LAUNCH_PID_FILE}"
    record_pid_identity "${SUPERVISOR_PID_FILE}" "$$" || die "Could not record supervisor identity"

    trap supervisor_cleanup EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP

    remote_source_ros
    ros2_bin="$(command -v ros2)"
    [[ -n "${ros2_bin}" ]] || supervisor_fail "ros2 is unavailable after sourcing the workspace"

    write_phase "starting-xvfb"
    Xvfb "${DISPLAY_NAME}" -screen 0 "${SCREEN_GEOMETRY}" +extension GLX +render -noreset >> "${XVFB_LOG}" 2>&1 &
    xvfb_pid=$!
    record_pid_identity "${XVFB_PID_FILE}" "${xvfb_pid}" || supervisor_fail "Could not record Xvfb identity"
    for (( i=0; i<150; i++ )); do
        xdpyinfo -display "${DISPLAY_NAME}" >/dev/null 2>&1 && break
        pid_matches "${XVFB_PID_FILE}" "Xvfb ${DISPLAY_NAME}" || supervisor_fail "Xvfb exited during startup"
        sleep 0.1
    done
    xdpyinfo -display "${DISPLAY_NAME}" >/dev/null 2>&1 || supervisor_fail "Xvfb did not become ready in 15 seconds"
    DISPLAY="${DISPLAY_NAME}" glxinfo -B >> "${HEALTH_LOG}" 2>&1 || supervisor_fail "GLX software rendering is unavailable"

    write_phase "starting-vnc"
    x11vnc -display "${DISPLAY_NAME}" -localhost -rfbport "${REMOTE_VNC_PORT}" \
        -rfbauth "${VNC_PASSWORD_FILE}" -forever -shared -noxdamage >> "${VNC_LOG}" 2>&1 &
    vnc_pid=$!
    record_pid_identity "${VNC_PID_FILE}" "${vnc_pid}" || supervisor_fail "Could not record x11vnc identity"
    for (( i=0; i<150; i++ )); do
        vnc_listener_ready && break
        pid_matches "${VNC_PID_FILE}" "x11vnc" || supervisor_fail "x11vnc exited during startup"
        sleep 0.1
    done
    vnc_listener_ready || supervisor_fail "x11vnc did not bind localhost:${REMOTE_VNC_PORT} in 15 seconds"

    write_phase "preparing-perception"
    printf '%s\n' '#!/usr/bin/env bash' > "${PERCEPTION_WRAPPER}"
    printf '%s\n' 'set -e' >> "${PERCEPTION_WRAPPER}"
    printf 'export LD_PRELOAD=%q${LD_PRELOAD:+:${LD_PRELOAD}}\n' '/lib/x86_64-linux-gnu/libffi.so.7' >> "${PERCEPTION_WRAPPER}"
    printf 'exec %q "$@"\n' "${CONDA_PYTHON}" >> "${PERCEPTION_WRAPPER}"
    chmod 700 "${PERCEPTION_WRAPPER}"

    write_phase "launching-eufs"
    setsid "${ros2_bin}" launch eufs_launcher simulation.launch.py \
        track:="${TRACK}" \
        gazebo_gui:=true \
        rviz:=true \
        show_rqt_gui:=false \
        perception:=true \
        perception_bbox_source:=yolov8 \
        perception_publish_fusion_debug:=true \
        perception_publish_yolo_debug_image:=true \
        perception_python_executable:="${PERCEPTION_WRAPPER}" >> "${LAUNCH_LOG}" 2>&1 &
    launch_pid=$!
    record_pid_identity "${LAUNCH_PID_FILE}" "${launch_pid}" || supervisor_fail "Could not record ROS launch identity"

    write_phase "waiting-gazebo-factory"
    for (( i=0; i<90; i++ )); do
        launch_is_alive || supervisor_fail "ROS launch exited before Gazebo factory became ready"
        if ros2 service list 2>/dev/null | grep -qx '/spawn_entity'; then
            service_ready=1
            break
        fi
        sleep 1
    done
    (( service_ready == 1 )) || supervisor_fail "Gazebo /spawn_entity service was not ready in 90 seconds"

    write_phase "waiting-robot"
    if ! wait_for_joint_states 20; then
        if gz model -l 2>/dev/null | grep -qx 'eufs'; then
            robot_present=1
        fi
        if (( robot_present == 0 )); then
            write_phase "retrying-robot-spawn"
            timeout 60 /usr/bin/python3 /opt/ros/galactic/lib/gazebo_ros/spawn_entity.py \
                -entity eufs \
                -file "${REMOTE_ROOT}/install/eufs_racecar/share/eufs_racecar/robots/eufs/robot.urdf" \
                -x -13.0 -y 10.3 -z 0.1 -R 0.0 -P 0.0 -Y 0.0 -timeout 45.0 >> "${SPAWN_LOG}" 2>&1 || true
        fi
        wait_for_joint_states 40 || supervisor_fail "Robot joint states are absent after the guarded spawn retry"
    fi

    write_phase "checking-rviz-and-wheels"
    timeout 15 ros2 service call /unpause_physics std_srvs/srv/Empty '{}' >> "${HEALTH_LOG}" 2>&1 || warn "Could not call /unpause_physics; continuing with runtime checks"
    for (( i=0; i<30; i++ )); do
        launch_is_alive || supervisor_fail "ROS launch exited while waiting for RViz"
        ros2 node list 2>/dev/null | grep -qx '/rviz' && break
        sleep 1
    done
    ros2 node list 2>/dev/null | grep -qx '/rviz' || supervisor_fail "RViz node was not discovered"
    tf_output="$(timeout 15 ros2 run tf2_ros tf2_echo base_footprint left_front_wheel 2>&1 || true)"
    printf '%s\n' "${tf_output}" >> "${HEALTH_LOG}"
    printf '%s\n' "${tf_output}" | grep -q 'Translation:' || supervisor_fail "Wheel TF base_footprint -> left_front_wheel is unavailable"

    write_phase "warming-zed-camera"
    for (( i=0; i<30; i++ )); do
        launch_is_alive || supervisor_fail "ROS launch exited while waiting for the ZED camera"
        gz_camera_topic="$(gz topic -l 2>/dev/null | grep '/zed_left_camera/image$' | sed -n '1p' || true)"
        [[ -n "${gz_camera_topic}" ]] && break
        sleep 1
    done
    [[ -n "${gz_camera_topic}" ]] || supervisor_fail "Gazebo ZED left camera topic was not created"
    timeout 15 gz topic -z "${gz_camera_topic}" -d 8 >> "${HEALTH_LOG}" 2>&1 || true
    topic_has_header_sample /zed/left/image_rect_color sensor_msgs/msg/Image 30 || supervisor_fail "ZED left camera has a publisher but no image samples"

    write_phase "checking-yolov8"
    topic_has_header_sample /yolo_bounding_boxes eufs_msgs/msg/BoundingBoxes 45 || supervisor_fail "YOLOv8 bounding boxes produced no samples"
    topic_has_header_sample /yolo_bounding_boxes/debug_image sensor_msgs/msg/Image 30 || supervisor_fail "YOLOv8 debug image produced no samples"

    write_phase "ready"
    printf '%s\n' "$(date '+%F %T')" > "${READY_FILE}"

    while true; do
        launch_is_alive || supervisor_fail "ROS launch process exited after readiness"
        pid_matches "${XVFB_PID_FILE}" "Xvfb ${DISPLAY_NAME}" || supervisor_fail "Xvfb exited after readiness"
        pid_matches "${VNC_PID_FILE}" "x11vnc" || supervisor_fail "x11vnc exited after readiness"
        sleep 2
    done
}

stop_managed_session_no_lock() {
    local pane_pid="" supervisor_pid="" i

    if ! session_name_exists; then
        rm -f "${READY_FILE}" "${SUPERVISOR_PID_FILE}" "${XVFB_PID_FILE}" "${VNC_PID_FILE}" "${LAUNCH_PID_FILE}"
        printf '%s\n' "stopped" > "${PHASE_FILE}"
        return 0
    fi
    managed_session_exists || die "Refusing to stop unowned tmux session: ${SESSION_NAME}"

    pane_pid="$(tmux display-message -p -t "${SESSION_NAME}:0.0" '#{pane_pid}' 2>/dev/null || true)"
    supervisor_pid="$(read_pid_file "${SUPERVISOR_PID_FILE}")" || die "Managed supervisor identity is missing"
    [[ "${pane_pid}" == "${supervisor_pid}" ]] || die "tmux pane PID does not match the recorded supervisor"
    pid_matches "${SUPERVISOR_PID_FILE}" "${REMOTE_SCRIPT} __remote supervise" || die "Managed supervisor identity no longer matches"
    stop_recorded_process "${SUPERVISOR_PID_FILE}" "${REMOTE_SCRIPT} __remote supervise" 30
    for (( i=0; i<50; i++ )); do
        session_name_exists || break
        sleep 0.1
    done
    session_name_exists && die "tmux session remained after supervisor exit; refusing an unverified kill"

    for (( i=0; i<150; i++ )); do
        if ! recorded_processes_alive; then
            break
        fi
        sleep 0.1
    done
    if recorded_processes_alive; then
        force_stop_recorded_processes
    fi
    recorded_processes_alive && die "Managed processes did not stop cleanly; PID files were preserved"
    rm -f "${READY_FILE}" "${SUPERVISOR_PID_FILE}" "${XVFB_PID_FILE}" "${VNC_PID_FILE}" "${LAUNCH_PID_FILE}"
    printf '%s\n' "stopped" > "${PHASE_FILE}"
}

run_with_remote_operation_lock() {
    local action="$1"
    local command
    OPERATION_LOCKED=1
    command="$(build_remote_command "${action}")"
    exec flock --close -w 15 "${LOCK_FILE}" /bin/bash -lc "exec ${command}"
}

remote_stop() {
    ensure_remote_state_dir
    if [[ "${OPERATION_LOCKED}" != "1" ]]; then
        run_with_remote_operation_lock stop
    fi
    if session_name_exists; then
        stop_managed_session_no_lock
        printf 'remote=stopped\n'
    else
        force_stop_recorded_processes
        recorded_processes_alive && die "Recorded managed processes could not be stopped"
        rm -f "${READY_FILE}" "${SUPERVISOR_PID_FILE}" "${XVFB_PID_FILE}" "${VNC_PID_FILE}" "${LAUNCH_PID_FILE}"
        printf '%s\n' "stopped" > "${PHASE_FILE}"
        printf 'remote=already-stopped\n'
    fi
}

remote_start() {
    local command last_phase="" phase="" deadline

    ensure_remote_state_dir
    if [[ "${OPERATION_LOCKED}" != "1" ]]; then
        run_with_remote_operation_lock start
    fi

    remote_doctor
    if remote_quick_health; then
        printf 'remote=already-running\n'
        remote_status
        return 0
    fi
    if session_name_exists && ! managed_session_exists; then
        die "tmux session ${SESSION_NAME} exists but is not owned by this manager"
    fi
    if managed_session_exists; then
        stop_managed_session_no_lock
    fi
    if xdpyinfo -display "${DISPLAY_NAME}" >/dev/null 2>&1; then
        die "X display ${DISPLAY_NAME} is already owned by another process"
    fi
    if vnc_listener_ready; then
        die "Remote VNC port ${REMOTE_VNC_PORT} is already owned by another process"
    fi

    rm -f "${READY_FILE}" "${FAILURE_FILE}"
    command="$(build_remote_command supervise)"
    tmux new-session -d -s "${SESSION_NAME}" "exec ${command}"
    say "Remote session created: ${SESSION_NAME}"

    deadline=$(( SECONDS + START_TIMEOUT ))
    while (( SECONDS < deadline )); do
        if [[ -r "${PHASE_FILE}" ]]; then
            phase="$(sed -n '1p' "${PHASE_FILE}")"
            if [[ "${phase}" != "${last_phase}" ]]; then
                printf 'phase=%s\n' "${phase}"
                last_phase="${phase}"
            fi
        fi
        if remote_quick_health; then
            remote_status
            return 0
        fi
        if [[ -r "${FAILURE_FILE}" ]]; then
            printf 'failure=%s\n' "$(sed -n '1p' "${FAILURE_FILE}")" >&2
            tail -n 60 "${MANAGER_LOG}" "${LAUNCH_LOG}" >&2 || true
            return 1
        fi
        managed_session_exists || {
            tail -n 60 "${MANAGER_LOG}" "${LAUNCH_LOG}" >&2 || true
            die "Remote manager session exited before readiness"
        }
        sleep 1
    done

    record_failure "Startup exceeded ${START_TIMEOUT} seconds"
    stop_managed_session_no_lock
    return 1
}

remote_logs() {
    local lines="${1:-120}"
    validate_uint "LINES" "${lines}" 1 5000
    tail -n "${lines}" "${MANAGER_LOG}" "${LAUNCH_LOG}" "${VNC_LOG}" "${XVFB_LOG}" "${HEALTH_LOG}" "${SPAWN_LOG}" 2>/dev/null || true
}

remote_attach() {
    managed_session_exists || die "Managed tmux session is not running: ${SESSION_NAME}"
    exec tmux attach-session -t "${SESSION_NAME}"
}

remote_main() {
    local action="${1:-}"
    shift || true
    validate_config
    remote_init_paths
    case "${action}" in
        start) remote_start "$@" ;;
        stop) remote_stop "$@" ;;
        status) remote_status "$@" ;;
        logs) remote_logs "$@" ;;
        doctor) remote_doctor "$@" ;;
        attach) remote_attach "$@" ;;
        supervise) remote_supervise "$@" ;;
        *) die "Unknown remote action: ${action}" ;;
    esac
}

main() {
    local command="${1:-help}"
    shift || true
    OPERATION_LOCKED=0
    validate_config
    case "${command}" in
        start)
            acquire_local_lock
            start_impl "$@"
            ;;
        stop)
            acquire_local_lock
            stop_impl "$@"
            ;;
        restart)
            acquire_local_lock
            stop_impl
            start_impl
            ;;
        status)
            status_impl "$@"
            ;;
        logs)
            logs_impl "$@"
            ;;
        attach)
            attach_impl "$@"
            ;;
        doctor)
            acquire_local_lock
            doctor_impl "$@"
            ;;
        setup-password)
            acquire_local_lock
            setup_password_impl "$@"
            ;;
        open)
            acquire_local_lock
            open_impl "$@"
            ;;
        version|--version)
            printf '%s %s\n' "${PROGRAM_NAME}" "${VERSION}"
            ;;
        help|--help|-h)
            usage
            ;;
        *)
            usage >&2
            die "Unknown command: ${command}"
            ;;
    esac
}

if [[ "${1:-}" == "__remote" ]]; then
    shift
    remote_main "$@"
else
    main "$@"
fi
