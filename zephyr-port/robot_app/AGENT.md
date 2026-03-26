# HomeRobot - Zephyr Robot Firmware AGENT.md

## Overview
The primary firmware for the HomeRobot project, targeting the **ESP32-C6** and built on **Zephyr RTOS**. It follows a "Smart Server, Dumb Robot" architecture, focusing on high-frequency sensor data streaming and low-level actuator control.

## Hardware Stack
- **MCU**: ESP32-C6 (Main controller)
- **LiDAR**: RP-Lidar A1M8 (360-degree point cloud)
- **IMU**: BMI160 (6-axis Accel/Gyro)
- **Actuators**: 2x DC Motors with Encoders (Differential Drive)
- **Power**: 4x Li-Ion 26650 batteries (4S configuration)

## Core Components
- **Sensors (`src/sensors/`)**: 
  - `Encoders`: Raw tick counting via PCNT.
  - `Imu`: BMI160 driver wrapper.
  - `Battery`: ADC monitoring for 4S pack.
  - `Lidar`: Custom parser for RPLIDAR A1 protocol.
- **Actuators (`src/actuator/`)**:
  - `Motor`: PWM control with local PID speed regulation.
  - `StatusLed`: WS2812 control via RMT for visual feedback.
- **Communication (`src/communication/`)**:
  - `ProtobufHandler`: Serialization using nanopb with 2-byte BE length prefix framing.
  - `ZephyrNetClient`: BSD Sockets implementation for TCP/IP.
- **Diagnostics (`src/diagnostic.cpp`)**: Hardware health verification module.

## Remote Procedure Calls (RPC)
The firmware implements a synchronous RPC dispatcher within the main loop:
- **Trigger**: Server sends an `RpcRequest` envelope.
- **Execution**: Firmware executes the requested method (e.g., `"RunDiagnostic"`).
- **Response**: Results are wrapped in an `RpcResponse` and sent back over the TCP stream.

## Key Build Commands
- `make build-c6`: Compile for the ESP32-C6.
- `make flash`: Flash the image to the device.
- `make snapshot-logs`: Capture 5s of serial diagnostic output.

## Logging
- Uses Zephyr's standard logging subsystem.
- Configured with `CONFIG_CBPRINTF_FP_SUPPORT=y` for floating-point output in logs.
