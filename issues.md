# Known Issues

## 6. Right encoder: direction never flips, counts noise at standstill

**Observed:** 2026-07-02 on real hardware, encoder deltas logged server-side
while driving small rotations in both directions
(`cmd_sender motion --action rotate --angle 30 / --angle=-30`):

- CCW rotation: `L=-5..-8, R=+2..+8` — signs correct.
- CW rotation: `L=+6..+12, R=+2..+14` — right wheel is driven BACKWARD but its
  encoder keeps counting POSITIVE.
- Robot idle, motors off: `L=0, R=+2..+18` per 100ms telemetry packet —
  continuous phantom forward ticks at ~40–180 ticks/s.

**Impact:** odometry accumulates phantom forward-right motion at rest, closed-
loop motions involving the right wheel can't converge (both rotate RPCs hit the
firmware 10s timeout), SLAM input is degraded.

**Analysis:** the right encoder is PCNT unit 1 — signal on GPIO23
(`PCNT1_CH0SIG`), direction/control on GPIO22 (`PCNT1_CH0CTRL`), both
`bias-pull-up` (overlay `pcnt_default`). The two symptoms point to the same
place:
1. Direction never flipping = the CTRL/B-phase input stuck at one level. With
   the pull-up enabled, a disconnected or broken B wire reads constantly HIGH —
   exactly this behavior.
2. Counting at standstill = noise pulses on the SIG/A line being counted; the
   1023-cycle glitch filter from 64c4797 reduced but did not eliminate it.

**Swap test result (2026-07-02): CONFIRMED HARDWARE — the fault follows the
encoder.** With the left/right connectors swapped at the board, the same
misbehavior appears on the LEFT channel. ESP32 pins, PCNT configuration and
firmware are exonerated; the right wheel's encoder assembly or its cable is
faulty.

Both symptoms are consistent with a single failure in the encoder/cable:
- B/direction channel dead (broken wire, cold joint on the encoder PCB, or a
  dead sensor channel) → direction never flips (pull-up reads constant HIGH).
- The same damaged cable, or the dead output floating, picking up motor EMI →
  phantom counts at standstill.

**Next hardware checks:**
1. Continuity-test the B wire end-to-end (encoder pin → connector pin), with a
   wiggle test near the wheel where the cable flexes — breaks cluster there.
2. Inspect the connector crimps and the solder joints on the encoder PCB.
3. If the encoder is a detachable module, swap the left/right encoder UNITS at
   the wheels: fault following the unit = replace the encoder; fault staying
   with the position = the cable harness.

## 7. Real lidar: recurring missing slices up to ~8.5° per revolution

**Observed:** 2026-07-02, real hardware. Sweep stats show avg delta ~1.0–1.66°
(217–359 points/rev) but max deltas of 8.3–8.5° in every 5s window — several
consecutive samples missing per revolution.

**Tooling (done):** the dashboard now shades missing angular sections as
translucent red wedges radiating from the robot (threshold: >4° without
readings; override with the `HR_GAP_THRESHOLD` env var, degrees). Invalid
readings (distance < 10mm) also count as missing.

**First observation from the live view:** the gaps are NOT uniformly
distributed — they cluster in a consistent angular fan on one side of the
robot. That pattern suggests a physical occlusion (mounting post, cabling,
chassis edge in the beam plane) or a fixed interference sector, rather than
random UART/packet loss. Compare the wedge sector against the physical mounting
to confirm; if it rotates with the robot chassis it is mechanical.

**Second observation (robot logs, 2026-07-02):** the firmware lidar driver
reports `PtsRcv == PtsRcvGood, PtsRcvErrors=0` — every point received over UART
is valid. The missing slices therefore never leave the sensor: the RPLidar
suppresses failed measurements (absorbing/black or specular surfaces, or an
occluder) rather than sending invalid points. Consistent with the occlusion
hypothesis; not a UART/firmware data-loss problem.


## 5. Two live robots flap the single session slot

**Observed:** 2026-07-02 during sim testing on pcluca. The REAL robot
(192.168.199.123) is configured to connect to the same server the simulation
uses. With the single-active-session policy (new connection replaces old,
commit f955fec), the real robot and the sim robot replaced each other ~30
times/second (~4000 replacements in a couple of minutes).

The replacement mechanism itself behaves correctly — one writer at a time,
sockets force-closed — but two persistent robots make the slot flap and no
session lives long enough to assemble lidar sweeps.

**Workaround:** power off / disconnect the real robot while running the sim on
pcluca (or vice versa).

**Proper fix (deferred):** make the server bind address/port configurable (env
var or flag) so sim and real deployments don't share an endpoint; optionally
add a replacement debounce (ignore new connections for the first ~2s of a
session) to bound worst-case churn.


## 4. [FIXED] Simulation: lidar returned a constant ring (zero FOV)

**Observed/Fixed:** 2026-07-02. Every one of the 180 samples returned the SAME
distance — measured at robot (0.74, 2.0) all rays read 7.0m and at (0.70, 3.39)
all read 8.4m: in both cases exactly the distance to the wall directly BEHIND
the robot. The scan was one backward ray replicated 180 times.

**Root cause:** commit 399f509 ("fix simulation lidar direction") reversed the
sensor sweep by setting `min_angle=3.14159, max_angle=-3.14159` in
`simulation/homerobot/model.sdf`. Gazebo's gpu_lidar degenerates this to a
zero-FOV scan where all samples share one direction. Since that commit, every
sim map/SLAM result was built from garbage (uniform free discs through walls —
hence the phantom out-of-arena frontiers).

**Fix:**
1. `model.sdf`: restored a valid CCW sweep (`min=-pi, max=+pi`).
2. `gazebo_bridge.cpp`: negate the angle when converting to the RPLidar
   clockwise convention the server expects — the mirroring 399f509 originally
   tried to fix, now fixed in the right place.

**Verified** headless on pcluca: ranges span 0.49–7.63m; with ground-truth pose,
130/180 world-projected points land within 20cm of the actual walls (the rest
hit the interior divider/pillars) vs 25/180 for the mirrored interpretation.
Exploration now selects in-arena frontiers and reached one ("Goal reached
within 0.20m") for the first time in sim.

**Note for issue 1:** the dashboard lidar view could never have shown correct
sim data before this fix. Re-test the dashboard against the sim GUI now — if it
renders, issue 1 is specific to the real robot's data path.

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

## 3. [FIXED] Simulation: robot spins in place on forward drive

**Fixed 2026-07-02** by two sim-side changes (real firmware untouched):
1. `gazebo_bridge.cpp`: removed the right-motor torque negation introduced by
   64c4797 — both wheel joints share the same model-frame axis, so forward means
   positive torque on BOTH joints. Encoder direction is already handled by the
   Motor `invert_encoder` flags + the `ticks_[0]` sign.
2. `simulation/homerobot/model.sdf`: wheel radius 0.033→0.0399 and track
   0.26→0.24 to match the firmware capabilities (79.85mm / 240mm) — tick-based
   motions were geometrically wrong in sim (rotate-90 ≈ 68° + PID crawl + timeout).

Verified headless on pcluca: 1.6m dead-straight drive (y drift <1e-7, encoders
L=R positive), rotate-90 RPC completes (~96° physical), 90s autonomous
exploration drove the robot ~4m while mapping.

**Follow-up observation (RESOLVED — see issue 4):** frontier goals outside the
±5m arena were NOT SLAM drift; the sim lidar was returning a constant ring
(issue 4), painting a fake free-space disc through the walls.

### Original report (for reference)

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
