#!/usr/bin/env python3
"""
yolo_bench.py — harvest dnn_node_example's own timing into benchmark numbers.

dnn_node_example publishes a `perfs` block on every PerceptionTargets
message with per-stage timings (preprocess / predict_infer / postprocess
/ pipeline). That's better data than wrapping perf_counter around
anything from outside, so this just collects it and does the stats.

Run (X5), with the YOLO node already running:

    source /opt/tros/humble/setup.bash
    python3 yolo_bench.py --ros-args -p label:=isolated
    # ...bring up RTAB-Map, then:
    python3 yolo_bench.py --ros-args -p label:=concurrent

Writes bench_out/<label>.json + .csv and prints a table ready to paste
into docs/benchmark.md.
"""

import json
import statistics
from pathlib import Path

import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets


def percentile(sorted_vals, p):
    """Nearest-rank percentile — no numpy needed."""
    if not sorted_vals:
        return None
    k = max(1, int(round(p / 100.0 * len(sorted_vals))))
    return sorted_vals[min(k, len(sorted_vals)) - 1]


def summarize(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return None
    return {
        "n": n,
        "mean_ms": round(statistics.fmean(s), 2),
        "median_ms": round(percentile(s, 50), 2),
        "p95_ms": round(percentile(s, 95), 2),
        "p99_ms": round(percentile(s, 99), 2),
        "min_ms": round(s[0], 2),
        "max_ms": round(s[-1], 2),
        "stdev_ms": round(statistics.pstdev(s), 2) if n > 1 else 0.0,
    }


class YoloBench(Node):
    def __init__(self):
        super().__init__("yolo_bench")
        self.declare_parameter("topic", "/beetle_detections")
        self.declare_parameter("label", "run")
        self.declare_parameter("target", 200)
        self.declare_parameter("warmup", 10)
        self.declare_parameter("outdir", "bench_out")

        gp = lambda n: self.get_parameter(n).value
        self.label = gp("label")
        self.target = int(gp("target"))
        self.warmup = int(gp("warmup"))
        self.outdir = Path(gp("outdir"))

        self.stages = {}      # stage name -> [ms, ...]
        self.fps_vals = []
        self.n_seen = 0
        self.done = False

        self.sub = self.create_subscription(
            PerceptionTargets, gp("topic"), self.on_msg, 50)
        self.get_logger().info(
            f"[{self.label}] collecting {self.target} frames from "
            f"{gp('topic')} (discarding first {self.warmup})")

    def on_msg(self, msg):
        if self.done:
            return
        self.n_seen += 1
        if self.n_seen <= self.warmup:       # warm-up discarded
            return

        for p in msg.perfs:
            d = float(p.time_ms_duration)
            if d <= 0.0:                      # unpopulated stage (e.g. parse)
                continue
            name = p.type
            for prefix in ("yolov8n_640x640_nv12_", "yolov8_640x640_nv12_"):
                if name.startswith(prefix):
                    name = name[len(prefix):]
                    break
            self.stages.setdefault(name, []).append(d)

        if msg.fps > 0:
            self.fps_vals.append(float(msg.fps))

        kept = self.n_seen - self.warmup
        if kept % 50 == 0:
            self.get_logger().info(f"  {kept}/{self.target}")
        if kept >= self.target:
            self.done = True
            self.report()

    def report(self):
        out = {"label": self.label,
               "frames": self.n_seen - self.warmup,
               "warmup_discarded": self.warmup,
               "stages": {k: summarize(v) for k, v in self.stages.items()}}
        if self.fps_vals:
            out["node_reported_fps"] = summarize(self.fps_vals)

        self.outdir.mkdir(parents=True, exist_ok=True)
        (self.outdir / f"{self.label}.json").write_text(
            json.dumps(out, indent=2) + "\n")

        with (self.outdir / f"{self.label}.csv").open("w") as f:
            cols = sorted(self.stages)
            f.write("i," + ",".join(cols) + "\n")
            rows = max(len(v) for v in self.stages.values())
            for i in range(rows):
                f.write(str(i) + "," + ",".join(
                    f"{self.stages[c][i]:.3f}" if i < len(self.stages[c]) else ""
                    for c in cols) + "\n")

        print(f"\n=== {self.label} — {out['frames']} frames "
              f"(first {self.warmup} discarded) ===")
        print(f"{'stage':<18}{'mean':>9}{'p95':>9}{'median':>9}"
              f"{'min':>9}{'max':>9}")
        order = ["recvedimg", "preprocess", "predict_infer",
                 "predict_parse", "postprocess", "pipeline"]
        for name in order + [k for k in sorted(self.stages) if k not in order]:
            s = out["stages"].get(name)
            if not s:
                continue
            print(f"{name:<18}{s['mean_ms']:>9}{s['p95_ms']:>9}"
                  f"{s['median_ms']:>9}{s['min_ms']:>9}{s['max_ms']:>9}")
        if self.fps_vals:
            print(f"\nnode-reported FPS: mean "
                  f"{out['node_reported_fps']['mean_ms']}")
        print(f"\nwrote {self.outdir}/{self.label}.json and .csv")
        print("\nThe headline number for docs/benchmark.md is "
              "predict_infer (BPU inference).")
        print("pipeline is end-to-end latency; note that task_num=4 means "
              "throughput is higher than 1/pipeline.")


def main():
    rclpy.init()
    node = YoloBench()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        if node.stages:
            node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
