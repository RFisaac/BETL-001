#!/usr/bin/env bash
# Download every model BETL needs (about 600 MB of downloads, ~420 MB on disk afterwards).
# Everything runs offline after this. Do it BEFORE travelling: GitHub downloads can be slow or
# blocked in China.
#
#   bash scripts/download_models.sh            # into ~/betl_models
#   bash scripts/download_models.sh /data/betl # somewhere else (then set models_dir in config)
#   BETL_EXTRA_VOICES=en_US-ryan-medium bash scripts/download_models.sh   # also the clearer ryan
set -euo pipefail

DEST="${1:-$HOME/betl_models}"
mkdir -p "$DEST"
cd "$DEST"
GH=https://github.com/k2-fsa/sherpa-onnx/releases/download

fetch_tar() {          # $1 = release/tag path, $2 = folder it unpacks to
    if [ -d "$2" ]; then echo "  have $2"; return; fi
    echo "  downloading $2"
    curl -fL --retry 3 "$GH/$1" | tar xj
}

echo "== wake word (keyword spotter, English)"
fetch_tar kws-models/sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01.tar.bz2 \
          sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01

echo "== speech recognition (streaming, Chinese + English)"
fetch_tar asr-models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16.tar.bz2 \
          sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16

echo "== voices"
for v in en_US-ryan-low zh_CN-huayan-medium ${BETL_EXTRA_VOICES:-}; do
    fetch_tar "tts-models/vits-piper-$v.tar.bz2" "vits-piper-$v"
done

echo "== translation (Argos Translate 1.9 packages, used directly with CTranslate2)"
mkdir -p argos
for pair in en_zh zh_en; do
    if [ -d "argos/translate-${pair}-1_9" ]; then echo "  have $pair"; continue; fi
    echo "  downloading $pair"
    curl -fL --retry 3 -o "argos/$pair.zip" "https://argos-net.com/v1/translate-${pair}-1_9.argosmodel"
    python3 -c "import zipfile,sys; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" \
        "argos/$pair.zip" argos
    rm "argos/$pair.zip"
done

echo "== trimming files BETL doesn't use"
rm -rf sherpa-onnx-*/test_wavs
# the bilingual model ships extra variants in sub-folders; only the top-level fp32 files are used
rm -rf sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/{64,96}
rm -f sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/*.int8.onnx
rm -f sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01/{encoder,decoder,joiner}-epoch-12-avg-2-chunk-16-left-64.onnx
rm -rf argos/*/stanza

du -sh "$DEST"
echo "Models are in $DEST"
