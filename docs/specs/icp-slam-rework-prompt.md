# Prompt: Design the ICP/SLAM Quality Rework

> Usage: start a fresh session and paste this file's content, or just say
> "Read docs/specs/icp-slam-rework-prompt.md and follow it".

---

Design a rework of the SLAM / scan-matching subsystem of the HomeRobot Rust
server. **This is a design task: produce a written plan first — do not
implement anything until the design is reviewed and approved.**

## System context

- Differential-drive home robot, ESP32-C6 firmware (Zephyr), RPLidar A1M8:
  triangulation lidar, 2000 samples/s standard mode, ~360 pts/rev at 5–6Hz,
  0.15–12m range. Real scans have missing angular slices (see issues.md #7);
  the dashboard visualizes them as red wedges.
- Odometry: wheel encoders (360 ticks/rev) fused with gyro Z at 98/2 in a
  complementary filter (`server/src/odometry.rs`). CAVEAT: the right encoder
  hardware is currently faulty (issues.md #6, always-positive + noise at
  standstill) — the design should degrade gracefully under bad odometry, and
  hardware repair is in progress.
- Server: event-driven per-connection `Session` (`server/src/session.rs`)
  feeding a shared `WorldModel` (`server/src/world.rs`) that owns the SLAM
  state; it survives robot reconnects and is persisted to `house_map.bin`
  (format `HRWORLD1`: grid + pose; the ICP reference cloud is NOT persisted).
- Consumers that must keep working: frontier exploration
  (`mapping.rs::find_frontiers`), A* planning over an inflated costmap
  (`pathfinding.rs`), the navigator state machine (`navigator.rs`), the GTK
  dashboard and Rerun logging.

## Current implementation (`server/src/slam.rs`, `BasicSlam`)

- Point-to-point ICP of each sweep against a **rolling reference cloud of the
  last 1000 points** (not the map): brute-force nearest neighbor, 15
  iterations, 20cm outlier gate, SVD-free closed-form 2D rigid fit, 0.95
  damping ("odometry spring") per iteration.
  Cost: ~300 valid points × 1000 reference × 15 iterations ≈ 4.5M distance
  checks per sweep at 5Hz.
- Motion prediction from odometry deltas; per-point deskewing by interpolating
  a 2s odometry pose history (`docs/lidar_deskewing.md`).
- Map: 600×600 @ 5cm log-odds occupancy grid (`mapping.rs`), updated per
  deskewed sweep (hit +20, ray −5, clamped ±100). Shared thresholds:
  free ≤ −20, occupied ≥ +20, unknown == 0.
- Scan points filtered to 0.2–8m before matching/mapping.
- `BasicSlam::restore()` after a server restart reloads grid+pose but ICP is
  inert until the rolling cloud refills (dead-reckoning only).

## Known weaknesses to address (from the 2026-07-02 codebase review)

1. **No drift correction on revisit**: matching against the last-1000-points
   window means loop closure is impossible; error accumulates without bound.
   In sim, drift painted free space beyond the arena walls.
2. **Brute-force correspondence** is O(N·M·iters) per sweep.
3. **Not map-anchored**: the authoritative artifact (the occupancy grid) is
   never used for matching, only written to.
4. **No relocalization**: after restart the robot cannot re-register against
   the loaded map; after a kidnap/manual move it silently diverges.
5. Restore hack: `update_count = 1` to keep ICP path alive after load.

## Constraints

- Rust, no ROS. The crate currently has NO linear-algebra dependency —
  everything is hand-rolled. A small, justified dependency (e.g. `nalgebra`)
  is acceptable; a heavy framework is not.
- Real-time budget: sweep matching at 5–6Hz plus 1Hz replanning on a desktop
  (build/test machine is powerful). Debug-build tests must stay fast.
- The `Slam` trait (update / add_odom_pose / get_map_data / save_map /
  get_frontiers / plan_path) may evolve, but session/navigator/tests
  currently depend on it — evolve it deliberately, not incidentally.
- The persistence format may be versioned/extended (`HRWORLD1` magic).
- Keep the WorldModel single-writer model (session holds the lock per event).

## What the design must cover

1. **Matching architecture**: compare at least (a) scan-to-map matching
   against the log-odds grid (likelihood field / correlative matching),
   (b) grid-bucketed or kd-tree ICP against a persistent keyframe/submap
   cloud, (c) hybrid. Recommend one with rationale for THIS robot (sparse
   ~360-pt scans with gaps, small home environments, 5cm grid).
2. **Drift management**: map-anchored matching and/or explicit loop closure;
   state what bounds the error on revisit and what happens in feature-poor
   corridors.
3. **Relocalization**: coarse re-registration against the persisted map on
   restart, and divergence detection + recovery during operation.
4. **Robustness to bad odometry** (encoder fault mode): what happens when the
   motion prior is garbage; gyro-only prior fallback.
5. **Evaluation methodology** — this is mandatory, not optional: the Gazebo
   sim on the build machine is trustworthy (movement and lidar verified) and
   publishes ground-truth pose on
   `/world/homerobot_world/pose/info` (`GZ_PARTITION=homerobot_sim`).
   Define ATE/RPE-style metrics, a scripted drive pattern, and pass/fail
   thresholds so the rework is measured against `BasicSlam`, not eyeballed.
   `tools/regression_test.py` and `tools/ci.sh` exist as starting points.
6. **Task breakdown**: incremental, testable tasks in the style of
   `docs/server-improvement-plan.md` (acceptance criteria, verification
   command, scope estimate, checkpoints). Each task must leave the suite
   (54 tests, `make ci`) green; `BasicSlam` should remain available behind
   the trait until the replacement beats it in the sim benchmark.

## Files to read first

- `server/src/slam.rs`, `server/src/mapping.rs`, `server/src/world.rs`
- `server/src/session.rs` (how sweeps reach SLAM), `server/src/scan.rs`
- `server/src/pathfinding.rs`, `server/src/navigator.rs` (consumers)
- `docs/lidar_deskewing.md`, `docs/SLAM_RESOURCES.md`
- `docs/server-improvement-plan.md` (style + deferred list), `issues.md`

## Build/test environment (macOS host cannot build the server)

- `make ci` → rsync to the Linux build machine + clippy `-D warnings` + tests
  in nix-shell (host configurable via `HR_CI_HOST`).
- Simulation on the build machine: rsync the tree, `nix-shell --run
  "./tools/start_sim.sh --headless"`; the sim server binds 127.0.0.1
  (HR_BIND) so it never captures the real robot. Drive it with
  `tools/cmd_sender --proxy --host <machine> ...`.

## Deliverable

A design document at `docs/specs/icp-slam-rework.md` containing: the compared
options with trade-offs, the chosen architecture, the evaluation plan with
concrete metrics and thresholds, the task breakdown with checkpoints, and open
questions for the maintainer. Present the plan for review before writing any
implementation code.
