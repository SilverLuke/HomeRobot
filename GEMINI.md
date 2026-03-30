# HomeRobot Project GEMINI.md

## Project Overview
HomeRobot is a personal robotics project focused on creating an autonomous vacuum robot. The system uses a **"Smart Server, Dumb Robot"** philosophy: the robot handles real-time hardware control and data streaming, while the high-level Rust server handles complex processing like SLAM, path planning, and odometry.

### Architecture
- **Robot Firmware (`zephyr-port/robot_app`)**: The primary firmware for the **ESP32-C6**, developed using **Zephyr RTOS**. It manages:
  - **Sensors**: IMU (BMI160), LiDAR (RP-Lidar A1M8), Encoders (PCNT), and Battery (ADC).
  - **Actuators**: Differential drive motors with PID speed control.
  - **Communication**: Bidirectional Protobuf over TCP/IP (Wi-Fi). Includes a lightweight **RPC Dispatcher** for synchronous commands.
  - **Self-Diagnostics**: Integrated module to verify hardware health (Battery, IMU, Motors/Encoders) on boot or via remote RPC.
- **Control Server (`server/`)**: A **Rust** application that:
  - Consumes telemetry (IMU, raw Encoder ticks, Lidar scans).
  - Performs **Server-side Odometry** and eventually SLAM.
  - Issues real-time movement commands and **RPC requests** (e.g., remote diagnostics).
- **Protobuf (`proto/`)**: Standardized definitions in `messages.proto`. Communication uses a **2-byte Big-Endian length prefix** for framing.
- **Legacy Robot (`robot/`)**: Original Arduino/PlatformIO version (deprecated).

## Hardware Stack
- **MCU**: ESP32-C6 (Main controller)
- **LiDAR**: RP-Lidar A1M8 (360-degree point cloud)
  - **UART**: UART1 (TX: GPIO 7, RX: GPIO 6)
  - **Motor Enable**: GPIO 15
- **IMU**: BMI160 (6-axis Accel/Gyro)
  - **I2C**: I2C0 (SDA: GPIO 4, SCL: GPIO 5)
- **Actuators**: 2x DC Motors (Differential Drive)
  - **Motor SX**: Forward: GPIO 19 (PWM), Backward: GPIO 18 (PWM)
  - **Motor DX**: Forward: GPIO 11 (PWM), Backward: GPIO 10 (PWM)
- **Encoders**: 2x Incremental Encoders (PCNT)
  - **Encoder SX**: A: GPIO 21, B: GPIO 20 (PCNT Unit 0)
  - **Encoder DX**: A: GPIO 23, B: GPIO 22 (PCNT Unit 1)
- **Power**: 4x Li-Ion 26650 batteries (4S configuration)
  - **Battery Sense**: ADC0 Channel 2 (GPIO 2)

### Driver Notes
- **Encoders**: Uses a C wrapper (`pcnt_reader.c`) to access low-level ESP32-C6 PCNT hardware units directly, bypassing Zephyr's Unit 0 limitation.
- **Motors**: 2-pin PWM scheme (PWM on FWD or BWD pin, other pin LOW).
- **Console**: UART0 (GPIO 0/1) via CH343 USB bridge.

## Building and Running

### Prerequisites
- [Nix](https://nixos.org/) with `direnv` (recommended for environment setup)
- [Cargo](https://doc.rust-lang.org/cargo/) (Rust build tool)
- [West](https://docs.zephyrproject.org/latest/develop/west/index.html) (Zephyr build tool)

### Key Commands

#### Top-level (Makefile)
- `make all`: Builds both the server and the Zephyr app (default 16MB flash).
- `make server`: Builds the Rust control server.
- `make zephyr`: Builds the Zephyr-based robot application.
- `make build-c6 FLASH=8M`: Build specifically for 8MB flash hardware (defaults to 16M).
- `make flash`: Flashes the Zephyr app to the ESP32-C6.
- `make monitor`: Starts the serial monitor for the ESP32.
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
- **RPC System**: Uses `RpcRequest` and `RpcResponse` envelopes within the Protobuf stream to handle synchronous command/response patterns without disrupting the asynchronous telemetry flow.

### Remote Diagnostics
- **Trigger**: Press 'T' in the Rust server CLI to trigger a full hardware self-test.
- **Sequence**:
  1. Battery voltage/percentage check.
  2. IMU communication and gravity vector validation.
  3. Sequential motor test (Forward -> Backward) with real-time IMU vibration monitoring.
- **Feedback**: Results are returned as a structured `DiagnosticResult` message and displayed in the server logs.

### Logging
- Firmware uses Zephyr's logging subsystem (`LOG_INF`, `LOG_DBG`, etc.).
- Server uses standard Rust `println!` or logging crates.
