# BETL-001

**BPU Education Transport & Lidar robot** — built on the D-Robotics **RDK X5**.

Participant entry for the [Robotics Dream Keeper Challenge](https://github.com/D-Robotics/Robotics-Dream-Keeper-Challenge). This README documents Stage 1 (Ignite Challenge) bring-up: flash → network → SSH → sensors → first on-device AI task.

**Stage 1 community check-in:** [Discord post](https://discord.com/channels/1300358874280230994/1509220927462969575/1514489192158330931)

---

## Stage 1 — Ignite Challenge

### 1. System image flash

- Flashed a supported RDK X5 OS image using **RDK Studio**.
- Board boots to a normal login prompt and accepts SSH connections.
- **Note on evidence:** a screenshot of the flash-success screen itself was not captured. However, the board connecting in RDK Studio ([screenshot](./media/stage1/rdkstudio_connection.png)) and the live SSH/network session below are only possible after a successful flash and boot.

### 2. Network connectivity

- Configured Wi-Fi via RDK Studio's Connect Device flow; board obtained IP `192.168.128.10`.
- Verified public internet reach and DNS resolution from the board (ping + DNS lookup).
- Evidence: [dns_resolution.png](./media/stage1/dns_resolution.png)

### 3. SSH login

- SSH enabled; interactive shell from PC to board as `root` on port 22:

```bash
ssh root@192.168.128.10
uname -a
```

- Evidence: terminal session in [dns_resolution.png](./media/stage1/dns_resolution.png) and connection details in [rdkstudio_connection.png](./media/stage1/rdkstudio_connection.png)

### 4. Sensor — stereo depth camera

- **Hardware:** D-Robotics **GS130W MIPI stereo camera module** (dual SC132GS global-shutter sensors), factory stereo calibration in onboard EEPROM (~80 mm baseline).
- **Interface:** **MIPI CSI**, dual-lane (one sensor per lane); sensors enumerate on **I2C** at `0x32` / `0x33` (buses 4 and 6). Driven through the `hobot_mipi_cam` / `hobot_stereonet` ROS 2 packages.
- Ran on-device stereo depth estimation: **DStereoV2.4_int16** (IGEV-based), BPU-accelerated, **~14.6 fps at ~95–100% BPU utilization**. Output is the colorized depth map served over the web preview (distance overlay visible in the screenshot).
- Evidence: [depthmap.png](./media/stage1/depthmap.png)

Run:

```bash
source /opt/tros/humble/setup.bash
cp -r /opt/tros/humble/share/hobot_stereonet/script/run_stereo.sh .
bash run_stereo.sh
# View at http://<board-ip>:8000  (RGB + colorized depth map)
```

**Camera rotation note:** the GS130W's sensors are physically mounted rotated 90°, so the stereo scripts default to `mipi_rotation 90.0`. The depth web preview appears upside-down, which tempts a 180° correction (rotation 270°) — but only 90° has valid GDC rectification calibration for this module; other values collapse the computed FOV to 0° ("Target FOV out of valid range") and the camera node aborts. The depth data itself is geometrically correct regardless of preview orientation — the upside-down appearance is cosmetic. Planned fix: correct orientation physically via mount design, keeping `mipi_rotation 90.0`.

### 5. First AI task — YOLO object detection (on-device)

- Ran **YOLOv8n** (nano, COCO 80-class) object detection on the X5 **BPU** (not on PC), fed live from the stereo camera's left eye.
- **Model:** the stock pre-installed `.bin` shipped on the board — `/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin` (self-reports as `yolov8n_640x640_nv12`), 640×640 NV12 input. Run via the `dnn_node_example` ROS 2 package (no separate model zoo checkout).
- Observable output: annotated frames with bounding boxes and class labels (`person` detected).
- Evidence: [yolo_detection.jpeg](./media/stage1/yolo_detection.jpeg)

Run:

```bash
# Terminal 1 — camera:
source /opt/tros/humble/setup.bash
bash run_cam.sh --image_width 1920 --image_height 1080 --rotation 0.0 --cal_rotation 0.0 --log_level INFO

# Terminal 2 — detector (subscribe mode, fed from left eye):
source /opt/tros/humble/setup.bash
cp -r /opt/tros/humble/lib/dnn_node_example/config .
ros2 run dnn_node_example example --ros-args \
  -p feed_type:=1 -p is_shared_mem_sub:=0 \
  -p config_file:=config/yolov8workconfig.json \
  -p dump_render_img:=1 \
  -p ros_img_topic_name:=/image_left_raw
```

---

## Dependencies

| Component | Version |
|---|---|
| OS base | Ubuntu 22.04.5 LTS, kernel 6.1.83 (aarch64) |
| ROS 2 | Humble (TogetheROS.Bot / tros.b) |
| BPU Platform | 1.3.6 |
| HBRT (libdnn runtime) | 3.15.55.0 |
| DNN Runtime | 1.24.5 |
| Detection model | yolov8n_640x640_nv12 (stock, on-board) |
| Stereo model | DStereoV2.4_int16 (IGEV-based) |

<!-- TODO when board is back in hand:
  RDK OS image version:   cat /etc/version   (and/or rdkos_info)
  tros/hobot package versions:  apt list --installed 2>/dev/null | grep -iE 'tros|hobot|stereonet|mipi-cam'
-->

---

## Media

| File | Shows |
|---|---|
| [media/stage1/rdkstudio_connection.png](./media/stage1/rdkstudio_connection.png) | RDK Studio connected to board (post-flash), IP/user/port |
| [media/stage1/dns_resolution.png](./media/stage1/dns_resolution.png) | SSH session, network + DNS verification |
| [media/stage1/depthmap.png](./media/stage1/depthmap.png) | Stereo depth map (sensor evidence) |
| [media/stage1/yolo_detection.jpeg](./media/stage1/yolo_detection.jpeg) | YOLOv8n detection running on-device |

---

## Roadmap

- **Stage 2 — Build:** robot concept, AI system architecture, execution plan; physical mount fix for camera orientation.
- **Stage 3 — Launch:** integrated demo with BPU-accelerated real-time inference.
