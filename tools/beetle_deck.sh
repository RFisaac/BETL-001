#!/usr/bin/env bash
# beetle_deck.sh — one command on the Steam Deck.
# Starts the teleop UDP sender headless, opens the console fullscreen.
# Closing the browser (or Ctrl-C) stops teleop too — a burst of zeros goes
# out on exit, and the X5-side watchdogs are the backstop anyway.
#
# Usage:
#   ./beetle_deck.sh              # find the robot automatically
#   ./beetle_deck.sh 192.168.86.58   # or name it
#   ./beetle_deck.sh --hud        # old beetle_hud.html instead of the console
#
# TWO THINGS RUN, ON PURPOSE:
#   deck_teleop.py   UDP:5005, holds the deadman, drives the robot
#   browser          rosbridge over websocket, tunes and monitors
# Driving stays on the raw socket. A websocket adds latency and a reconnect
# loop to the one path that must not have either.
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
PAGE="beetle_console.html"
X5=""

for arg in "$@"; do
    case "$arg" in
        --hud)  PAGE="beetle_hud.html" ;;
        -*)     echo "unknown option: $arg" >&2; exit 2 ;;
        *)      X5="$arg" ;;
    esac
done

# No IP given: ask the beacon. beetle_agent runs from boot, so this works
# before the ROS stack is up — which is the point, since the console is how
# you start it.
if [[ -z "$X5" ]]; then
    echo "locating robot..."
    X5="$(python3 "$DIR/beetle_find.py" 2>/dev/null || true)"
    if [[ -z "$X5" ]]; then
        echo "robot not found." >&2
        echo "  is it powered on?  systemctl status beetle-agent" >&2
        echo "  or pass the IP:    ./beetle_deck.sh 192.168.86.58" >&2
        exit 1
    fi
    echo "found at $X5"
fi

URL="http://${X5}:8080/${PAGE}"

cleanup() {
    [[ -n "${TELEOP_PID:-}" ]] && kill "$TELEOP_PID" 2>/dev/null || true
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 "$DIR/deck_teleop.py" --host "$X5" &
TELEOP_PID=$!
sleep 1
kill -0 "$TELEOP_PID" 2>/dev/null || {
    echo "teleop died on startup — run it directly to see why:" >&2
    echo "  python3 $DIR/deck_teleop.py --host $X5" >&2
    exit 1
}

echo "teleop -> ${X5}:5005   (hold L1 to drive)"
echo "console -> $URL"

# Kiosk browser: chromium if present, else firefox. 1280x800 native.
if command -v chromium >/dev/null 2>&1; then
    chromium --kiosk --noerrdialogs --disable-session-crashed-bubble \
        --autoplay-policy=no-user-gesture-required "$URL"
elif command -v chromium-browser >/dev/null 2>&1; then
    chromium-browser --kiosk --noerrdialogs "$URL"
else
    firefox --kiosk "$URL"
fi
# browser exit falls through to cleanup -> teleop killed, zeros sent
