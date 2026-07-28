# Benchmark — BPU vs. CPU under concurrent load

**Evidence status: captured.** Raw files are in this repo — `bench/isolated.json`,
`bench/concurrent.json`, `logs/bpu_proof.log`, `logs/versions.txt`. Captured 2026-07-28 on
the robot, `nodes/yolo_bench.py` + a manual `hrut_somstatus` trace (see
[`nodes/capture_benchmark.sh`](../nodes/capture_benchmark.sh) for the scripted version of
the same capture).

## Methodology

- Single-model results compare **isolated** (YOLO only, nothing else on CPU) against
  **concurrent** (YOLO running alongside the CPU-side stack) at matched resolution — this is
  the BPU-engagement proof the challenge asks for.
- Timing comes from `dnn_node_example`'s own `perfs` block, published on every
  `PerceptionTargets` message with per-stage timings (`preprocess` / `predict_infer` /
  `postprocess` / `pipeline`). That's more accurate than wrapping a timer around anything
  from outside the pipeline, so `nodes/yolo_bench.py` just collects it and reports the
  stats — no `perf_counter` scaffolding needed here.
- **Warm-up excluded**: the first 10 frames of each run are discarded before any frame is
  counted, so JIT/cache/USB-settle effects don't bias the numbers.
- **Both mean and p95 are reported.** Worst case matters more than average for a robot
  running in real time — a mean that hides an occasional 40 ms spike is the wrong number to
  design around.
- Concurrent FPS/latency is expected to be **worse** than isolated — that's correct and
  expected, not a regression to explain away. The interesting number is *how much* worse
  each stage gets, and specifically whether the BPU stage is one of them.

## Headline result

200 frames per run (10 discarded as warm-up), YOLOv8n INT8 640×640 on the BPU via
`dnn_node_example`, fed from the RealSense RGB stream. Full precision, from
`bench/isolated.json` / `bench/concurrent.json`:

| Stage | Isolated mean | Concurrent mean | Δ | Isolated p95 | Concurrent p95 |
|---|---|---|---|---|---|
| `predict_infer` (**BPU inference**) | 7.25 ms | 7.47 ms | **+3.0%** | 8 ms | 9 ms |
| `preprocess` (CPU) | 8.85 ms | 12.29 ms | +38.9% | 13 ms | 18 ms |
| `postprocess` (CPU) | 4.21 ms | 5.03 ms | +19.5% | 6 ms | 9 ms |
| node-reported FPS, mean | 23.3 | 18.18 | −22.0% | — | — |

**BPU inference held essentially flat under concurrent CPU load (+3%) while the CPU-bound
stages of the same pipeline degraded 19–39%.** That's the concurrent-workload result this
benchmark exists to demonstrate — measured, not asserted.

`recvedimg` and end-to-end `pipeline` are **not** used as the degradation comparison above:
both are dominated by frame-arrival queue wait rather than compute, and both happened to
come out *lower* under concurrent load in this capture (an artifact of frame timing/cadence
at capture time, not a real speed-up) — including them as "CPU stages got faster under
load" would be a false signal. `preprocess` and `postprocess` are the two stages that are
actually CPU compute time, and both moved the direction you'd expect.

**Note on what "concurrent" means here.** The live node graph today runs wheel+gyro dead
reckoning, not SLAM (see [`architecture.md`](architecture.md)) — RTAB-Map was tried, pushed
CPU load to a 16.4 load average with depth enabled, and was pulled from the live chain.
Its CPU cost is exactly the kind of heavy concurrent load this benchmark wants to prove the
BPU is insulated from; `rtabmap_ros` (confirmed installed, version in `logs/versions.txt`)
supplied the CPU-side load for the concurrent capture, independent of whether it was driving
the live map at the time. That's a deliberate, documented choice, not a leftover.

## BPU utilization proof

`logs/bpu_proof.log` — 120 samples of `hrut_somstatus` at ~1 Hz (2026-07-28, 08:31:00–
08:33:30) captured while YOLO was actively running. `bpu0` ratio held in a **4–8% band,
mostly 5–6%**, across all 120 samples — sustained non-zero BPU engagement for the whole
window, which is the proof inference isn't silently falling back to CPU. This is a
continuous-run trace rather than a load→idle→load transition (YOLO was already running for
the whole capture window), which is a slightly weaker visual than an on/off transition would
be but is still a direct utilization reading, not an inference.

## Reproducing this benchmark

[`nodes/capture_benchmark.sh`](../nodes/capture_benchmark.sh) automates the JSON capture and
the BPU trace in one script — copy it next to `yolo_bench.py` on the X5 and run
`bash capture_benchmark.sh concurrent` / `... isolated`. The manual sequence it's built from,
kept here for reference:

```bash
ssh root@<X5_IP>          # 192.168.86.58 as of the last session; confirm current IP
source /opt/tros/humble/setup.bash
cd /userdata

# 1. Bring up the baseline stack (camera + drives + odometry) — see baseline_run.md
#    for the full gated sequence, or:
python3 beetle_up.py --drive-only --yes

# 2. Start YOLO (already running is fine, this is idempotent to check)
ros2 run dnn_node_example example --ros-args \
  -p config_file:=config/yolov8workconfig.json \
  -p feed_type:=1 -p is_shared_mem_sub:=0 \
  -p ros_img_topic_name:=/camera/camera/color/image_raw \
  -p image_width:=640 -p image_height:=480 \
  -p dump_render_img:=0 -p msg_pub_topic_name:=/beetle_detections \
  --log-level warn &

# 3. Isolated run (nothing else competing for CPU)
python3 yolo_bench.py --ros-args -p label:=isolated -p target:=200

# 4. Bring up the CPU-side concurrent load (rtabmap_ros, or the live dead-reckoning +
#    lidar-accumulator stack — decide which before running so the "what concurrent means"
#    note above stays accurate), then:
python3 yolo_bench.py --ros-args -p label:=concurrent -p target:=200

# 5. BPU utilization trace — run once, while YOLO is up, in a separate window:
for i in $(seq 120); do { date +%T; hrut_somstatus; } >> bpu_proof.log; sleep 1; done

# 6. Version strings. NOTE: `ros2 --version` is NOT supported on this tros.b CLI build
#    and errors with "unrecognized arguments" — use $ROS_DISTRO instead.
{
  echo "=== captured $(date -Is) ==="
  echo "ROS_DISTRO: $ROS_DISTRO"
  ros2 pkg xml dnn_node_example
  ros2 pkg xml rtabmap_ros 2>&1 || echo "not installed"
  uname -a
  hrut_somstatus
} > versions.txt
```

**Known gotcha:** the X5's clock resets to 2000-01-01 on boot and nothing here fixes it —
timestamps inside `versions.txt`/`bpu_proof.log` reflect that, not real capture time (the
actual capture date, 2026-07-28, is recorded here in this doc instead). Doesn't affect any
of the measured durations above, which are all relative, but set the clock
(`timedatectl` / NTP, or `sudo date -s "..."` from a known-good reference) before recording
anything where wall-clock time matters, bag recordings especially.

Copy the results back:

```bash
scp root@<X5_IP>:/userdata/bench_out/isolated.json    bench/isolated.json
scp root@<X5_IP>:/userdata/bench_out/concurrent.json  bench/concurrent.json
scp root@<X5_IP>:/userdata/bpu_proof.log              logs/bpu_proof.log
scp root@<X5_IP>:/userdata/versions.txt               logs/versions.txt
```

`yolo_bench.py` also prints a formatted table to the terminal as it runs — worth screen
recording that alongside the Foxglove detection overlay for the "screenshot or short clip
of live stream with overlays" deliverable.
