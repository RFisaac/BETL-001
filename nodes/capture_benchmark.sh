#!/usr/bin/env bash
# capture_benchmark.sh — one-shot evidence capture for docs/benchmark.md
#
# Run ON THE X5, sitting next to yolo_bench.py, with YOLO already publishing
# to /beetle_detections.
#
#   bash capture_benchmark.sh concurrent   # run while the CPU-side load (lidar,
#                                          # wheel_odom, scan_accumulator, whatever's
#                                          # up) is ALSO running - this is the state
#                                          # you're in right now, so run this first
#
#   bash capture_benchmark.sh isolated     # run with YOLO the ONLY thing going -
#                                          # stop the rest of the stack first
#
# Writes, under ./bench_capture/:
#   bench/<label>.json + .csv   (from yolo_bench.py — per-stage mean/median/p95/etc.)
#   logs/bpu_proof_<label>.log  (hrut_somstatus sampled ~1 Hz across the capture window)
#   logs/versions.txt           (written once, on whichever run happens first)
#
# Doesn't touch anything else running - it only subscribes to /beetle_detections
# and shells out to hrut_somstatus. Safe to run while driving.

set -euo pipefail

LABEL="${1:-}"
if [[ "$LABEL" != "concurrent" && "$LABEL" != "isolated" ]]; then
  echo "usage: $0 concurrent|isolated" >&2
  echo "  (run concurrent first - it matches whatever's live right now)" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLO_BENCH="$SCRIPT_DIR/yolo_bench.py"
if [[ ! -f "$YOLO_BENCH" ]]; then
  echo "yolo_bench.py not found next to this script ($SCRIPT_DIR)." >&2
  echo "Copy it here first: it's nodes/yolo_bench.py in the repo." >&2
  exit 1
fi

OUT="$SCRIPT_DIR/bench_capture"
mkdir -p "$OUT/bench" "$OUT/logs"

source /opt/tros/humble/setup.bash 2>/dev/null || true

echo "=== Checking /beetle_detections is alive ==="
if ! timeout 8 ros2 topic hz /beetle_detections --window 20 2>/dev/null | grep -q "average rate"; then
  echo "WARNING: no messages seen on /beetle_detections in 8s." >&2
  echo "Is the YOLO node (dnn_node_example) actually running? Continuing anyway -" >&2
  echo "yolo_bench.py will just sit and wait for the first message." >&2
fi

# ---- versions.txt: written once, first run wins ----------------------------
VERSIONS="$OUT/logs/versions.txt"
if [[ ! -f "$VERSIONS" ]]; then
  echo "=== Writing $VERSIONS ==="
  {
    echo "=== captured $(date -Is) ==="
    echo
    echo "=== ROS / TROS ==="
    ros2 --version 2>&1 || true
    echo
    echo "=== dnn_node_example ==="
    ros2 pkg xml dnn_node_example 2>&1 || echo "package.xml not found via ros2 pkg xml"
    echo
    echo "=== rtabmap_ros (if installed - used as the concurrent CPU load) ==="
    ros2 pkg xml rtabmap_ros 2>&1 || echo "not installed"
    echo
    echo "=== OS / kernel ==="
    uname -a
    echo
    echo "=== BPU status at capture time ==="
    hrut_somstatus 2>&1 || echo "hrut_somstatus not found"
  } > "$VERSIONS"
else
  echo "=== $VERSIONS already exists, leaving it alone ==="
fi

# ---- BPU utilization trace, backgrounded for the capture window -----------
BPU_LOG="$OUT/logs/bpu_proof_${LABEL}.log"
: > "$BPU_LOG"
echo "=== Starting BPU trace -> $BPU_LOG (~1 Hz, stops when the capture below finishes) ==="
(
  for _ in $(seq 300); do
    { date +%T; hrut_somstatus; } >> "$BPU_LOG" 2>&1
    sleep 1
  done
) &
BPU_PID=$!
# Don't let a stray background job hang the shell on exit.
trap 'kill "$BPU_PID" 2>/dev/null || true' EXIT

# ---- the actual benchmark capture ------------------------------------------
echo
echo "=== Capturing '$LABEL' - 200 frames, first 10 discarded as warm-up ==="
echo "    Leave the robot doing whatever it's currently doing. Don't touch it."
echo
python3 "$YOLO_BENCH" --ros-args \
  -p label:="$LABEL" -p target:=200 -p warmup:=10 -p outdir:="$OUT/bench"

kill "$BPU_PID" 2>/dev/null || true
wait "$BPU_PID" 2>/dev/null || true
trap - EXIT

echo
echo "=== Done: $LABEL ==="
echo "  $OUT/bench/${LABEL}.json"
echo "  $OUT/bench/${LABEL}.csv"
echo "  $BPU_LOG"
echo "  $VERSIONS"
echo

if [[ "$LABEL" == "concurrent" ]]; then
  cat <<'EOF'
Next: stop the CPU-side load (lidar / wheel_odom / scan_accumulator / anything
else besides YOLO) so YOLO is the ONLY thing running, then:

    bash capture_benchmark.sh isolated

Once both runs are done, pull everything back to your laptop / repo checkout
(run FROM the laptop, not the robot):

    scp -r root@<X5_IP>:/userdata/bench_capture/bench  bench
    scp -r root@<X5_IP>:/userdata/bench_capture/logs   logs

Then drop the isolated.json / concurrent.json / bpu_proof_*.log / versions.txt
into BETL-001/bench/ and BETL-001/logs/ per docs/benchmark.md.
EOF
else
  cat <<'EOF'
If you've already run 'concurrent' too, both captures are done. Pull everything
back to your laptop / repo checkout (run FROM the laptop, not the robot):

    scp -r root@<X5_IP>:/userdata/bench_capture/bench  bench
    scp -r root@<X5_IP>:/userdata/bench_capture/logs   logs

Then drop the isolated.json / concurrent.json / bpu_proof_*.log / versions.txt
into BETL-001/bench/ and BETL-001/logs/ per docs/benchmark.md.
EOF
fi
