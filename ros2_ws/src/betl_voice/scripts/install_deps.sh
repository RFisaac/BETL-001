#!/usr/bin/env bash
# Install BETL voice dependencies on the RDK X5 (Ubuntu 22.04 + ROS 2 Humble / TROS).
# Run once:  bash scripts/install_deps.sh
set -euo pipefail

echo "== system packages"
sudo apt-get update
sudo apt-get install -y libportaudio2 libusb-1.0-0 python3-pip python3-numpy python3-scipy \
    python3-serial python3-usb python3-cffi espeak-ng

echo "== python packages"
# Keep numpy below 2 so ROS 2 Humble's own Python packages keep working.
pip3 install --user "numpy<2" sherpa-onnx ctranslate2 sentencepiece sounddevice pypinyin

echo "== permissions for the reSpeaker control interface and the LED ring"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
sudo cp "$HERE/udev/99-betl.rules" /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
sudo usermod -aG audio,dialout "$USER" || true

echo
echo "Done. Unplug and replug the reSpeaker and the RP2350, and log out/in once for group changes."
echo "Next: bash scripts/download_models.sh"
