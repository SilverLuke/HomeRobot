# Known Issues

## 1. Real robot: no lidar data on the dashboard

**Observed:** 2026-07-02, real hardware, branch `refactor/server-event-model`. Robot
connects and drives correctly, but the GTK dashboard shows no lidar points.

**Context:**
- In headless simulation the full lidar pipeline works: sweeps assemble at 5Hz
  (`[LIDAR Summary (last 5s)]` server logs), SLAM updates, map/GUI messages emitted.
  The GTK *rendering* path was never exercised in the sim runs (headless only).
- A sweep is forwarded to the GUI only when a `scan_completed` point arrives AND the
  buffered sweep has ≥30 points AND ≥80ms passed since the last flush
  (`server/src/scan.rs`). These thresholds are identical to the pre-refactor code.
- The server enables the lidar on connect (`LidarControl active=true, 5Hz`).

**Debug steps:**
1. With the robot connected, check the server log for `[LIDAR Summary (last 5s)]` lines.
   - **Present** → sweeps are assembling; the problem is GUI-side. Check the
     "Plot Sensor Reads" toggle (`btn_show_sensor` → `GUI_STATE.show_lidar`) and the
     `GuiUpdate::Lidar` handler in `server/src/gui/mod.rs`.
   - **Absent** → wire/robot side. Check robot syslog for lidar forwards; verify the
     `scan_completed` sync marker arrives and count points per revolution (a sweep
     with <30 points is silently dropped).
2. Run the pre-refactor server (`main`) on the same hardware to classify
   regression vs pre-existing.

## 2. Rotate-90° over-rotates physically; dashboard shows ~90°

**Observed:** 2026-07-02, real hardware. `ExecuteMotion` rotate (GUI button or
`cmd_sender motion --action rotate --angle 90`) turns the robot visibly past 90°,
while the dashboard heading reads the expected ~90°.

**Analysis:**
- Rotation tick targets are computed as `angle_rad × (wheel_track/2) × ticks_per_meter`
  from capabilities-derived `ROBOT_SIZES` (`server/src/main.rs::motion_ticks`,
  duplicated in the GUI rotate buttons). The firmware closed-loop then drives until
  the encoder targets are reached.
- If `ticks_per_meter` or `wheel_track` underestimate reality, the robot physically
  over-rotates while encoder-based odometry — computed from the *same wrong
  constants* — reads exactly 90°. This matches the symptom exactly: the error cancels
  out on the dashboard but not on the floor.
- The dashboard heading normally fuses gyro at 98% (`server/src/odometry.rs`). If it
  still reads 90° during a physically larger turn, gyro calibration may not have
  completed (requires 30 stationary IMU samples → `[ODOMETRY] Gyro calibration
  completed!` log) or the IMU wasn't streaming during the motion.

**Debug steps:**
1. Log the capabilities on connect (`wheel_diameter_mm`, `wheel_track_mm`,
   `encoder_ticks_per_rev`) and verify against the physical robot, including gear
   ratio — commit 64c4797 changed encoder counting, so these may be stale.
2. Command rotate-90 and record: encoder deltas, gyro-integrated angle, and physical
   angle (floor markings). Encoders hitting target while physical > target confirms
   the geometry constants are wrong.
3. Confirm the gyro calibration log appeared before the test.

## 3. Simulation: robot spins in place on forward drive (pre-existing)

**Observed:** 2026-07-02, headless Gazebo sim on pcluca. Real hardware drives
correctly, so this is sim-model-specific.

**Evidence:**
- Both motors commanded forward — bridge logs `SX Dir=1 Pwr=100 -> p=0.3922` and the
  same for `DX` — yet the robot yaws in place (~59° during a 4s test, ground-truth
  quaternion) with near-zero translation, and the right encoder counts *negative*.
- The server wire command was verified byte-accurate (robot log:
  `MOVE: L=100 (1.0) R=100 (1.0)`), so the inversion is below
  `robot/src/bridge/gazebo_bridge.cpp`: sim motor driver or right-wheel joint axis in
  the model SDF.
- Consequences: `ExecuteMotion` RPCs time out in sim (closed-loop never reaches its
  tick targets) and exploration cannot translate.
- Likely interacts with commit 64c4797 (right encoder direction fix for real
  hardware): the fix may need a sim-driver counterpart.
