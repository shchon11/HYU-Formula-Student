#!/usr/bin/env bash
set -euo pipefail

log_file="$(mktemp "${TMPDIR:-/tmp}/path_selector_launch.XXXXXX.log")"
launch_pid=""

cleanup() {
  local exit_code=$?
  if [[ -n "$launch_pid" ]]; then
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ "$exit_code" -ne 0 ]]; then
    cat "$log_file"
  fi
  rm -f "$log_file"
  trap - EXIT
  exit "$exit_code"
}
trap cleanup EXIT

setsid ros2 launch path_selector path_selector.launch.py use_sim_time:=false >"$log_file" 2>&1 &
launch_pid=$!

for _ in $(seq 1 50); do
  if grep -Fq "Strict selector ready" "$log_file" && kill -0 "$launch_pid" 2>/dev/null; then
    exit 0
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    exit 1
  fi
  sleep 0.1
done

exit 1
