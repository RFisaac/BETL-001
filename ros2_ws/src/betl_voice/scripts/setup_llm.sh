#!/usr/bin/env bash
# Optional: give BETL a small local language model for chat (runs on the X5's CPU, offline).
# Builds llama.cpp's server and downloads a Qwen 2.5 instruct model (speaks English and Chinese).
#
#   bash scripts/setup_llm.sh            # 0.5B model (~400 MB, fastest, simplest answers)
#   bash scripts/setup_llm.sh 1.5b       # 1.5B model (~1 GB, noticeably smarter, ~3x slower)
#
# Then launch BETL with:  ros2 launch betl_voice betl.launch.py llm:=true
# Do this before travelling; Hugging Face is blocked in China (hf-mirror.com sometimes works:
# HF=https://hf-mirror.com bash scripts/setup_llm.sh).
set -euo pipefail

SIZE="${1:-0.5b}"
DEST="$HOME/betl_models/llm"
HF="${HF:-https://huggingface.co}"
mkdir -p "$DEST"

echo "== build llama.cpp server"
sudo apt-get install -y cmake build-essential git
if [ ! -x "$DEST/llama.cpp/build/bin/llama-server" ]; then
    [ -d "$DEST/llama.cpp" ] || git clone --depth 1 https://github.com/ggml-org/llama.cpp.git "$DEST/llama.cpp"
    cmake -S "$DEST/llama.cpp" -B "$DEST/llama.cpp/build" -DLLAMA_CURL=OFF -DGGML_NATIVE=ON \
          -DCMAKE_BUILD_TYPE=Release
    cmake --build "$DEST/llama.cpp/build" -j "$(nproc)" --target llama-server
fi
ln -sf "$DEST/llama.cpp/build/bin/llama-server" "$DEST/llama-server"

echo "== download model ($SIZE)"
case "$SIZE" in
    0.5b) REPO=Qwen/Qwen2.5-0.5B-Instruct-GGUF; FILE=qwen2.5-0.5b-instruct-q4_k_m.gguf ;;
    1.5b) REPO=Qwen/Qwen2.5-1.5B-Instruct-GGUF; FILE=qwen2.5-1.5b-instruct-q4_k_m.gguf ;;
    *) echo "size must be 0.5b or 1.5b"; exit 1 ;;
esac
[ -f "$DEST/$FILE" ] || curl -fL --retry 3 -o "$DEST/$FILE" "$HF/$REPO/resolve/main/$FILE"
ln -sf "$DEST/$FILE" "$DEST/model.gguf"

echo
echo "Ready: $DEST/model.gguf -> $FILE"
echo "Test it alone:  $DEST/llama-server -m $DEST/model.gguf -t 4 -c 2048 --port 8080"
echo "Or with BETL:   ros2 launch betl_voice betl.launch.py llm:=true"
