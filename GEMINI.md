# HomeRobot Project GEMINI.md

## Project Overview
HomeRobot is a personal robotics project focused on creating an autonomous vacuum robot. The system uses a **"Smart Server, Dumb Robot"** philosophy: the robot handles real-time hardware control and data streaming, while the high-level Rust server handles complex processing like SLAM, path planning, and odometry.

### Architecture
- **Robot Firmware (`zephyr-port/robot_app`)**: The primary firmware for the **ESP32-C6**, developed using **Zephyr RTOS**. It manages:
  - **Sensors**: IMU (BMI160), LiDAR (RP-Lidar A1M8), Encoders (PCNT), and Battery (ADC).
  - **Actuators**: Differential drive motors with PID speed control.
  - **Communication**: Bidirectional Protobuf over TCP/IP (Wi-Fi).
- **Control Server (`server/`)**: A **Rust** application that:
  - Consumes telemetry (IMU, raw Encoder ticks, Lidar scans).
  - Performs **Server-side Odometry** and eventually SLAM.
  - Issues real-time movement and configuration commands.
- **Protobuf (`proto/`)**: Standardized definitions in `messages.proto`. Communication uses a **2-byte Big-Endian length prefix** for framing.
- **Legacy Robot (`robot/`)**: Original Arduino/PlatformIO version (deprecated).

## Hardware Stack
- **MCU**: ESP32-C6 (Main controller)
- **LiDAR**: RP-Lidar A1M8 (360-degree point cloud)
- **IMU**: BMI160 (6-axis Accel/Gyro)
- **Actuators**: 2x DC Motors with Encoders (Differential Drive)
- **Power**: 4x Li-Ion 26650 batteries (4S configuration)

## Building and Running

### Prerequisites
- [Nix](https://nixos.org/) with `direnv` (recommended for environment setup)
- [Cargo](https://doc.rust-lang.org/cargo/) (Rust build tool)
- [West](https://docs.zephyrproject.org/latest/develop/west/index.html) (Zephyr build tool)

### Key Commands

#### Top-level (Makefile)
- `make all`: Builds both the server and the Zephyr app.
- `make server`: Builds the Rust control server.
- `make zephyr`: Builds the Zephyr-based robot application.
- `make flash`: Flashes the Zephyr app to the ESP32-C6.
- `make snapshot-logs`: Captures 5 seconds of logs from the ESP32 (useful for non-blocking status checks).

#### Control Server (Rust)
Navigate to `server/`:
- `cargo build`: Build the server.
- `cargo run`: Start the server (listens on port 12345).

## Development Conventions

### Sensor-Actuator Split (Zephyr)
- **Sensors**: Located in `src/sensors/`. They only handle data acquisition (e.g., `Encoders` returns raw ticks).
- **Actuators**: Located in `src/actuator/`. They handle hardware actions (e.g., `Motor` handles PWM and local PID).
- **Communication**: `ProtobufHandler` manages serialization and framing logic.

### Odometry & Logic
- **Odometry is Server-Side**: The robot sends raw encoder ticks; the server calculates $(x, y, \theta)$.
- **Remote Configuration**: PID gains and motor limits are tunable via `RobotConfig` messages without reflashing.

### Communication Protocol
- **Transport**: TCP Sockets.
- **Framing**: `[Length: 2 bytes (BE)][Protobuf Payload: N bytes]`.

### Logging
- Firmware uses Zephyr's logging subsystem (`LOG_INF`, `LOG_DBG`, etc.).
- Server uses standard Rust `println!` or logging crates.
