# LiDAR Deskewing & Motion Distortion Compensation

When a robot moves or rotates while acquiring a LiDAR scan, the resulting map data suffers from **motion distortion** (or **skewing**). This document outlines the problem, how data fusion (IMU + Encoders) resolves it, and how we can infer point timing geometrically without hardware-level microsecond timestamps.

---

## 1. The Distortion Problem (Skewing)

A 2D spin LiDAR (like the RP-Lidar A1M8) does not capture a 360° scan instantaneously. Instead, it rotates physically, taking approximately **100ms to 200ms** to complete one full sweep.

If the robot is rotating at $180^\circ/\text{s}$ ($3.14\text{ rad/s}$):
* During a $100\text{ms}$ scan, the robot rotates by $18^\circ$.
* If we project all points using the robot's pose at the *start* of the scan, the points captured at the *end* of the scan will be off by $18^\circ$.
* This causes flat walls to appear curved ("banana effect") and corner angles to warp.

```
       Actual Wall                Warped Wall (During CW Rotation)
     +-------------+               +
     |             |                \
     |  [Robot]    |                 \   [Robot]
     |   (Spinning)|                  \   (Spinning)
     |             |                   \
     +-------------+                    +
```

---

## 2. Sensor Fusion for Deskewing

To correct (or **deskew**) the scan, we must project each individual point $i$ using the robot's exact pose at the moment that specific point was measured:

$$Pose(t_i) = Pose(t_{\text{start}}) + \Delta Pose(t_i)$$

Where:
* $Pose(t_i)$ is interpolated from high-frequency wheel encoder and IMU (gyro) readings.
* The point is projected from the sensor frame using its interpolated heading $\theta(t_i)$ and position $(x(t_i), y(t_i))$:

$$x_{\text{global}, i} = x(t_i) + d_i \cos(\theta(t_i) + \phi_i)$$
$$y_{\text{global}, i} = y(t_i) + d_i \sin(\theta(t_i) + \phi_i)$$

---

## 3. Inferring Timestamps Geometrically (No Hardware Timestamps Required)

**Question:** Is it strictly necessary for the hardware/firmware to tag each individual point with a microsecond timestamp?

**Answer:** **No.** Because the LiDAR spins at a highly stable angular velocity ($\omega$), we can infer the relative measurement time $t_i$ of any point geometrically based on its angle.

### Linear Angle Interpolation Method
Assuming the laser rotates at a constant speed, the relative time $t_i$ for a point at angle $\phi_i$ is linearly proportional to the angular sweep completed since the start of the scan:

$$t_i = \left( \frac{\phi_i - \phi_{\text{start}}}{\phi_{\text{end}} - \phi_{\text{start}}} \right) \times T_{\text{scan}}$$

Where:
* $\phi_i$: Angle of the current point (unwrapped to be monotonically increasing).
* $\phi_{\text{start}}$: Starting angle of the scan.
* $\phi_{\text{end}}$: Ending angle of the scan.
* $T_{\text{scan}}$: The total duration of one full rotation (calculated as the difference between consecutive scan packet arrival times, e.g., $100\text{ms}$).

### Implementation Strategy on the Server
Since the ESP32-C6 streams the LiDAR points in chronological sequence:
1. **Compute Scan Duration ($T_{\text{scan}}$):** The Rust server tracks the arrival time of consecutive scan batches: $T_{\text{scan}} = \text{Time}_{\text{now}} - \text{Time}_{\text{last\_scan}}$.
2. **Assign Relative Offsets:** For each point in the batch, assign $t_i$ using the index or the angle interpolation method above.
3. **Interpolate Robot Trajectory:** Query the high-frequency IMU/Odometry buffer to find the pose at $t_{\text{start}} + t_i$.
4. **Project Point:** Transform the range reading into the global coordinate frame using the interpolated pose.
