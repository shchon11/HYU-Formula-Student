#!/usr/bin/env bash
# Serve the cone labeler to teammates who are not on this network.
#
#   src/perception/scripts/serve_labeler.sh datasets/0801_cones
#
# Starts cone_labeler_web.py bound to 127.0.0.1 and puts a Cloudflare tunnel in
# front of it, so the campus network never sees an open port -- the only route in
# is the https URL this prints. The access token is stored in the dataset root
# and reused on every restart, so the links you hand out keep working.
#
# Ctrl+C stops the tunnel and the server together.
#
#   --port N           local port (default 8770)
#   --tunnel NAME      run a named tunnel instead of a throwaway quick tunnel;
#                      the hostname is whatever you routed to it, and you can put
#                      Cloudflare Access in front of it. Requires a prior
#                      `cloudflared tunnel login` / `create` / `route dns`.
#   --no-tunnel        local only, for testing on this machine
set -euo pipefail

ROOT=""
PORT=8770
TUNNEL=""
USE_TUNNEL=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --tunnel) TUNNEL="$2"; shift 2 ;;
    --no-tunnel) USE_TUNNEL=0; shift ;;
    -h|--help) sed -n '2,20p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) ROOT="$1"; shift ;;
  esac
done

if [[ -z "$ROOT" ]]; then
  echo "usage: $0 <dataset-root> [--port N] [--tunnel NAME] [--no-tunnel]" >&2
  exit 2
fi
if [[ ! -d "$ROOT/images" ]]; then
  echo "no images directory in $ROOT" >&2
  exit 2
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOKEN_FILE="$ROOT/.labeler_token"
if [[ ! -s "$TOKEN_FILE" ]]; then
  python3 -c "import secrets; print(secrets.token_urlsafe(18))" > "$TOKEN_FILE"
  chmod 600 "$TOKEN_FILE"
fi
TOKEN="$(cat "$TOKEN_FILE")"

LOG_DIR="$(mktemp -d)"
SERVER_PID=""
TUNNEL_PID=""

cleanup() {
  trap - INT TERM EXIT
  [[ -n "$TUNNEL_PID" ]] && kill "$TUNNEL_PID" 2>/dev/null || true
  [[ -n "$SERVER_PID" ]] && kill "$SERVER_PID" 2>/dev/null || true
  wait 2>/dev/null || true
  rm -rf "$LOG_DIR"
  echo "stopped"
}
trap cleanup INT TERM EXIT

# -u: the startup banner has to reach the log file before the readiness loop
# below reads it back, and a redirected stdout is block-buffered otherwise.
python3 -u "$HERE/cone_labeler_web.py" "$ROOT" --port "$PORT" --token "$TOKEN" \
  > "$LOG_DIR/server.log" 2>&1 &
SERVER_PID=$!

for _ in $(seq 1 50); do
  if curl -fs -o /dev/null "http://127.0.0.1:$PORT/api/session?t=$TOKEN" 2>/dev/null; then break; fi
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "server failed to start:" >&2
    cat "$LOG_DIR/server.log" >&2
    exit 1
  fi
  sleep 0.2
done

head -3 "$LOG_DIR/server.log"

if [[ "$USE_TUNNEL" -eq 0 ]]; then
  echo
  echo "local only:  http://127.0.0.1:$PORT/?t=$TOKEN"
  echo "Ctrl+C to stop."
  wait "$SERVER_PID"
  exit 0
fi

if ! command -v cloudflared >/dev/null; then
  echo "cloudflared is not on PATH -- see src/perception/docs/remote_labeling.md" >&2
  exit 1
fi

if [[ -n "$TUNNEL" ]]; then
  cloudflared tunnel run --url "http://127.0.0.1:$PORT" "$TUNNEL" \
    > "$LOG_DIR/tunnel.log" 2>&1 &
  TUNNEL_PID=$!
  echo
  echo "named tunnel '$TUNNEL' is up. Your hostname is whatever you routed to it:"
  echo "    https://<your-hostname>/?t=$TOKEN"
  echo "Put Cloudflare Access in front of that hostname to add SSO on top."
else
  cloudflared tunnel --url "http://127.0.0.1:$PORT" --no-autoupdate \
    > "$LOG_DIR/tunnel.log" 2>&1 &
  TUNNEL_PID=$!

  URL=""
  for _ in $(seq 1 100); do
    URL="$(grep -ohE 'https://[a-z0-9-]+\.trycloudflare\.com' "$LOG_DIR/tunnel.log" | head -1 || true)"
    [[ -n "$URL" ]] && break
    if ! kill -0 "$TUNNEL_PID" 2>/dev/null; then
      echo "cloudflared exited:" >&2
      tail -20 "$LOG_DIR/tunnel.log" >&2
      exit 1
    fi
    sleep 0.3
  done
  if [[ -z "$URL" ]]; then
    echo "cloudflared did not report a URL; last output:" >&2
    tail -20 "$LOG_DIR/tunnel.log" >&2
    exit 1
  fi

  echo
  echo "=============================================================="
  echo " hand this to the team:"
  echo "     $URL/?t=$TOKEN"
  echo
  echo " or one bookmark per person, which skips the name prompt:"
  for who in sojun minji dohyun; do
    echo "     $URL/?t=$TOKEN&user=$who"
  done
  echo "=============================================================="
  echo " This URL is public. The token is the only thing keeping"
  echo " strangers out, so share it in a team channel, not in public."
  echo " It dies when you Ctrl+C, and a quick tunnel gets a new"
  echo " address every restart -- use --tunnel NAME for a stable one."
  echo "=============================================================="
fi

echo
echo "server log:  $LOG_DIR/server.log"
echo "tunnel log:  $LOG_DIR/tunnel.log"
echo "Ctrl+C to stop both."
wait "$SERVER_PID"
