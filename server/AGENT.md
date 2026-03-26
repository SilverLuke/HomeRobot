# HomeRobot - Control Server AGENT.md

## Overview
Rust-based control server that processes telemetry and issues movement/configuration commands. Implements high-level logic like SLAM and path planning.

## Architecture & Logic
- **Server-side Odometry**: Calculates $(x, y, \theta)$ from raw encoder ticks sent by the robot.
- **Remote Configuration**: Tunable PID gains and motor limits via `RobotConfig` messages.
- **RPC Client**: Synchronous command/response pattern using `RpcRequest` and `RpcResponse`.

## Building and Running
- `cargo build`: Build the server.
- `cargo run`: Start the server (listens on port 12345).

## Communication Protocol
- **Transport**: TCP Sockets.
- **Framing**: `[Length: 2 bytes (BE)][Protobuf Payload: N bytes]`.
- **RPC Trigger**: Press 'T' in the CLI to trigger remote diagnostics on the connected robot.

## Feedback Handling
- **Diagnostics**: Decodes `DiagnosticResult` from `RpcResponse` and prints a structured summary of checks.
- **Telemetry**: Processes high-frequency IMU, Encoder, and Lidar data.

## Logging
- Uses standard Rust `println!` or logging crates. Logs incoming RPC responses and telemetry status.
