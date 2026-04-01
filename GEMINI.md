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
  - **UART**: UART1 (TX: GPIO 6, RX: GPIO 7)
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

### AI Agent Capabilities
The AI coding agent has direct connectivity to the robot hardware via the local development environment. It can autonomously:
- **Build**: Compile the Zephyr firmware and Rust server.
- **Flash**: Deploy firmware to the ESP32-C6 via `make flash`.
- **Monitor**: Capture and analyze real-time logs via `make monitor` or `make snapshot-logs`.
- **Test**: Execute manual movement and diagnostic commands using the `cmd_sender` utility to verify hardware behavior.

### Key Commands

#### Top-level (Makefile)
- `make all`: Builds both the server and the Zephyr app (default 16MB flash).
- `make server`: Builds the Rust control server.
- `make zephyr`: Builds the Zephyr-based robot application.
- `make build-c6 FLASH=8M`: Build specifically for 8MB flash hardware (defaults to 16M).
- `make flash`: Flashes the Zephyr app to the ESP32-C6.
- `make monitor`: Starts the serial monitor for the ESP32.
- `make snapshot-logs`: Captures 5 seconds of logs from the ESP32 (useful for non-blocking status checks).

#### Command Sender (Testing Utility)
Navigate to `tools/cmd_sender/` to send direct Protobuf commands:
- `cargo run -- move --left 100 --right 100`: Move forward at 100 power.
- `cargo run -- stop`: Send an immediate emergency stop.
- `cargo run -- diag`: Trigger the remote diagnostic suite.
- `cargo run -- interactive`: Enter WASD mode for manual navigation (Esc to exit).
- Add `--proxy` if testing via the `robot_proxy` bridge.

#### Manual Verification Workflow
To get immediate feedback after a code change (e.g., PID adjustment or sensor logic update):
1. **Flash & Monitor**: Run `make flash monitor` to deploy the change and observe serial output.
2. **Trigger Action**: In a separate terminal, use `cmd_sender` to move the robot (e.g., `cargo run -- move --left 50 --right 50`).
3. **Analyze**: Observe the PID error and encoder ticks in the serial logs to verify the hardware response matches expectations.
4. **Diagnostic**: Run `cargo run -- diag` to perform a comprehensive health check and verify that all systems are still operational.

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
- **Config Sync**: The robot is programmed to transmit its current `RobotConfig` (PID gains, etc.) immediately upon a successful TCP connection. This ensures the Server Dashboard is always synchronized with the hardware state without manual querying.

## Dashboard Architecture (GTK4 + Rust)
- **Threading Model**: 
    - **Main Thread**: Dedicated exclusively to the GTK4 Event Loop (`app.run()`).
    - **Background Thread 1 (Networking)**: Manages the TCP listener and per-connection `handle_connection` loops.
    - **Background Thread 2 (Input)**: Manages legacy SDL2 joystick and terminal `crossterm` events.
- **Async Bridge**: Since `glib::MainContext::channel` is deprecated in `glib-rs 0.20`, the server uses a standard `std::sync::mpsc` channel. The GTK thread polls this channel every 33ms (approx. 30 FPS) using `glib::timeout_add_local`.
- **Memory Safety**: To share GTK widgets (labels, canvas) with the polling closure, widgets must be **cloned** (incrementing the GObject reference count) before being moved into the closure.

## Telemetry Strategy
- **Fast Telemetry (10Hz)**: IMU (Accel/Gyro) and Encoders are bundled and sent every 100ms. This frequency is optimized for real-time dashboard responsiveness without saturating the ESP32 Wi-Fi buffer.
- **Slow Telemetry (0.2Hz)**: Battery voltage and health status are sent every 5 seconds.
- **LiDAR Streaming**: Sent as batches of points. The Dashboard uses a custom `GtkDrawingArea` with a scale of `0.05 px/mm` and renders a 50cm-interval polar grid for spatial reference.

## Development Gotchas
- **Nix Dev Headers**: GTK4 compilation in Nix requires the `.dev` output of libraries (e.g., `gtk4.dev`, `glib.dev`) to be present in `nativeBuildInputs` so that `pkg-config` can locate the `.pc` files.
- **Stop Logic**: Due to firmware-side tag limitations, the "Stop" command (sent on WASD release) is implemented as a `MotorMoveCommand` with `power=0`, rather than a dedicated boolean flag, to ensure cross-version compatibility.

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
