#!/usr/bin/env bash
# Make BETL start automatically when the X5 boots (systemd service "betl").
# Run as the user BETL should run as (normally sunrise), from anywhere:
#
#   bash scripts/install_service.sh            # with the local LLM if it's set up
#   bash scripts/install_service.sh --no-llm   # without it
#   bash scripts/install_service.sh --remove   # uninstall
#
# Afterwards:
#   sudo systemctl start betl        start now          sudo systemctl stop betl     stop
#   sudo systemctl restart betl      after config edits  sudo systemctl disable betl  no autostart
#   journalctl -u betl -f            live log            journalctl -u betl -b        this boot's log
set -euo pipefail

if [ "${1:-}" = "--remove" ]; then
    sudo systemctl disable --now betl 2>/dev/null || true
    sudo rm -f /etc/systemd/system/betl.service
    sudo systemctl daemon-reload
    echo "betl service removed"
    exit 0
fi

USER_NAME="$(id -un)"
USER_HOME="$HOME"
WS="${BETL_WS:-$USER_HOME/betl_ws}"
ROS_SETUP=""
for f in /opt/tros/humble/setup.bash /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash; do
    if [ -f "$f" ]; then ROS_SETUP="$f"; break; fi
done
[ -n "$ROS_SETUP" ] || { echo "No ROS 2 setup.bash found under /opt/tros or /opt/ros"; exit 1; }
[ -f "$WS/install/setup.bash" ] || { echo "Build first: cd $WS && colcon build --symlink-install"; exit 1; }

LLM=false
if [ "${1:-}" != "--no-llm" ] && [ -x "$USER_HOME/betl_models/llm/llama-server" ] \
   && [ -e "$USER_HOME/betl_models/llm/model.gguf" ]; then
    LLM=true
fi
THREADS=$(( $(nproc) > 4 ? 4 : $(nproc) ))

sudo tee /etc/systemd/system/betl.service > /dev/null <<EOF
[Unit]
Description=BETL robot voice (ROS 2)
After=network-online.target sound.target
Wants=network-online.target

[Service]
Type=simple
User=$USER_NAME
SupplementaryGroups=audio dialout
WorkingDirectory=$USER_HOME
Environment=HOME=$USER_HOME
Environment=PYTHONUNBUFFERED=1
Environment=RCUTILS_COLORIZED_OUTPUT=0
# give USB devices (reSpeaker, RP2350) a moment to show up after power-on
ExecStartPre=/bin/sleep 8
ExecStart=/bin/bash -c 'source $ROS_SETUP && source $WS/install/setup.bash && exec ros2 launch betl_voice betl.launch.py llm:=$LLM llm_threads:=$THREADS'
Restart=on-failure
RestartSec=5
KillSignal=SIGINT
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable betl
echo
echo "Installed: runs as $USER_NAME, ROS from $ROS_SETUP, workspace $WS, LLM=$LLM"
echo "Start it now:   sudo systemctl start betl"
echo "Watch the log:  journalctl -u betl -f"
