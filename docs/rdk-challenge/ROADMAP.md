# Roadmap — RustedFriend's RDK X5 Robot ("Beetle")

*Robotics Dream Keeper Challenge — through the July 15 Stage 3 deliverable*
*Version 1.0 — 2026-06-27*

> **Status update (2026-07-28):** this roadmap is the plan as of late June — kept as-written
> below for the historical record, not edited to match what actually shipped. The real
> Stage 3 build diverged from it in ways that matter: skid steer stayed skid steer (swerve
> was a later, separate design thread that didn't reach this robot), and **the mapping stack
> is wheel+gyro dead reckoning, not `slam_toolbox` SLAM** — depth-based SLAM (RTAB-Map) was
> tried, overloaded the CPU, and was pulled from the live chain. See
> [`docs/architecture.md`](../architecture.md) for what's actually running and
> [`docs/failure-recovery.md`](../failure-recovery.md) for how that decision got made.
> Milestone checkboxes below are left exactly as last written; they were not maintained
> day-to-day and shouldn't be read as a live status tracker.

Build strategy: electronics integration is **decoupled from the welded frame**. Bring-up
happens on a temporary "breadboard chassis" immediately, in parallel with waiting on weld
stock; known-good subsystems transplant onto the welded frame when parts arrive (~July 3).
This removes the frame-parts delivery date from the software critical path.

**Must-hit scope for July 15:** the two demo segments — (1) concurrent room-mapping +
object detection, (2) teleop-drive-while-mapping. Trained policies are a bonus, not required.

---

## Milestones

### M0 — Foundation (done / in progress)
- ✅ Registered; Discord intro thread; GitHub repo established
- ✅ RDK X5 ordered, booted, **BPU/YOLO path confirmed** (first test on D-Robotics stereo)
- ✅ Stage 2 proposal: concept, architecture, engineering plan
- ✅ Long-lead parts ordered (drivers, micros, power, LEDs, encoders)
- ⬜ Source design docs brought current

### M1 — Breadboard chassis (Jun 27 – Jul 2) — *not frame-dependent*
- ⬜ Electronics mounted to scrap board
- ⬜ 4× ODESC on CAN; motors driving under closed-loop velocity
- ⬜ X5 ↔ micros (odometry, stage) comms up (micro-ROS)
- ⬜ YOLOv8s + slam_toolbox running concurrently on the bench
- ⬜ Roller-dyno motor characterization (ODESC V/I logging)
- ⬜ Both demo segments roughed on the test rig
- ⬜ Power system: hot-swap logic rail, buck + USB-pack diode-OR, power monitor

### M2 — Weld & transplant (Jul 3 – Jul 8) — *frame-dependent*
- ⬜ Weld parts arrive (~Jul 3) → weld frame
- ⬜ Transplant known-good electronics onto the welded frame
- ⬜ Mount spinning-lidar package (stepper + slip ring); reprojection + mapping-while-driving
- ⬜ Mount cheese plate + payload configs; verify swap
- ⬜ Continuous footage capture begins

### M3 — Reliability & benchmark (Jul 9 – Jul 13) — *frame-dependent*
- ⬜ Reliability passes on both demo segments
- ⬜ Tune obstacle avoidance (soft→hard refusal) + e-stop
- ⬜ Benchmark runs: isolated vs. concurrent, p95 latency

### M4 — Submit (Jul 14 – Jul 15)
- ⬜ Shoot + edit 3–7 min 1080p demo video
- ⬜ Finalize `docs/benchmark.md`, README, `docs/`
- ⬜ PR to official repo `projects/`; community share; permalink in PR

### Global livestream — Jul 23

---

## Parked (post-deadline / if-time-permits)
Present in the architecture, not on the critical path to the two demo segments:
voice command system, load cell + cargo monitor, software payload-capability system,
trained policies (mass-conditioned locomotion, Y-horn ball-push), side-camera Foxglove
integration, weathering/finishing, suspension.

## Future arcs (beyond the competition)
Deployable lidar mast (+ stage IMU, two-IMU cross-check); Ackermann steering; suspension
(+ per-arm IMU array); power-slide/rally policy; battery-efficient scan-coverage policy;
cinematography platform; interior 3D scanning; separate docent robot.
