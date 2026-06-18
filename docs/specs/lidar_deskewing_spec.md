# Spec: LiDAR Deskewing (Motion Distortion Compensation)

## Objective
Implement LiDAR deskewing (motion distortion compensation) within the Rust control server. By interpolating the robot's pose for each individual point's measurement time during a scan (using a rolling buffer of high-frequency odometry telemetry and dynamic scan period measurement), we eliminate mapping and localization errors caused by robot motion (specifically rotation) during 360° sweeps.

## Tech Stack
* **Language:** Rust (1.92+)
* **Dependencies:** Standard library (`std::collections::VecDeque`, `std::time::Instant`)

## Commands
* **Build:** `cargo build --manifest-path server/Cargo.toml`
* **Test:** `cargo test --manifest-path server/Cargo.toml`
* **Run Simulation Tests:** `make test`

## Project Structure
* `server/src/slam.rs`: Holds `BasicSlam` and handles ICP matching. We will store the rolling pose buffer here.
* `server/src/mapping.rs`: Holds `OccupancyGrid` and updates cell probabilities.
* `docs/specs/lidar_deskewing_spec.md`: This specification file.

## Code Style
We follow standard Rust practices. When interpolating poses, we interpolate the translation linearly and interpolate the angle taking wrapping into account:

```rust
// Example pose interpolation snippet
fn interpolate_pose(p1: &Pose, p2: &Pose, t: f32) -> Pose {
    let x = p1.x + (p2.x - p1.x) * t;
    let y = p1.y + (p2.y - p1.y) * t;
    
    // Angle interpolation with wrap-around
    let mut diff = p2.theta - p1.theta;
    while diff > std::f32::consts::PI { diff -= 2.0 * std::f32::consts::PI; }
    while diff < -std::f32::consts::PI { diff += 2.0 * std::f32::consts::PI; }
    let theta = p1.theta + diff * t;
    
    Pose { x, y, theta }
}
```

## Testing Strategy
* **Unit Tests:** Add unit tests to `server/src/slam.rs` verifying:
  1. Rolling pose buffer insertion and retrieval.
  2. Trajectory/pose interpolation correctness (including angle wrap-around).
* **Regression Tests:** Verify that `make test` (the regression test in Gazebo) continues to compile and pass successfully.

## Boundaries
* **Always do:** Normalize all angles to $[-\pi, \pi]$ during interpolation.
* **Ask first:** Modifying the Protobuf protocol or message structures (not needed for this feature).
* **Never do:** Use hardcoded default scan speeds that ignore dynamic scan duration measurements.

## Success Criteria
* The server compiles successfully without errors or warnings.
* All unit tests pass.
* The SLAM algorithm operates with deskewed scans for both ICP matching and map updates.
* The Gazebo-based regression test (`make test`) passes successfully.
