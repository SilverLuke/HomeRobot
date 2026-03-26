# Objective

Migrate the `robot` codebase from the Arduino Framework to Zephyr RTOS on the ESP32-C6.

# Key Areas and Strategy

## 1. Project Initialization & Devicetree

- **Create Zephyr Application:** Scaffold a standard Zephyr project structure.
- **Board Overlay (`esp32c6_devkitc.overlay`):** Map out the physical hardware to Zephyr Devicetree:
  - **I2C:** For the BMI160 IMU.
  - **UART:** For the RPLIDAR A1.
  - **PWM:** For motor speed control (Drive motors and LiDAR motor).
  - **GPIO:** For motor direction control (IN1, IN2).
  - **ADC:** For battery voltage monitoring.
  - **Sensors/Interrupts:** For wheel encoders (using PCNT or GPIO interrupts).

## 2. Sensor Migration

- **BMI160 (IMU):**
  - _Strategy:_ Drop the Arduino `F_BMI160` library. Zephyr has a **native, built-in driver** for the BMI160.
  - _Implementation:_ Enable `CONFIG_BMI160` and use the Zephyr `<zephyr/drivers/sensor.h>` API to fetch Accelerometer and Gyroscope data.
- **RPLIDAR A1 (LiDAR):**
  - _Strategy:_ Zephyr does not have a native driver for RPLIDAR. We will need to port the `LDS_RPLIDAR_A1` protocol parser.
  - _Implementation:_ Use Zephyr's **UART Async API** or Interrupt-driven API to receive the data stream from the LiDAR. Use the Zephyr **PWM API** to spin the LiDAR motor. We will rewrite the `Lidar` class to wrap the Zephyr UART calls instead of `HardwareSerial`.
- **Battery:**
  - _Strategy:_ Drop `analogRead`.
  - _Implementation:_ Use Zephyr's `<zephyr/drivers/adc.h>` API.

## 3. Actuator & Control Migration

- **Motors & PID:**
  - _Strategy:_ Adapt `Motor_PID.cpp`.
  - _Implementation:_ Replace `analogWrite` with `pwm_set_dt()`. Replace `digitalWrite` with `gpio_pin_set_dt()`. Convert `millis()` to `k_uptime_get()`.
- **Encoders:**
  - _Strategy:_ Drop the Arduino `Encoder` library.
  - _Implementation:_ Use Zephyr's GPIO interrupt callbacks or Pulse Counter (PCNT) drivers if available for ESP32-C6 in Zephyr.

## 4. Communications & Logic Migration

- **WiFi & Network:**
  - _Strategy:_ Drop Arduino `WiFi.h`.
  - _Implementation:_ Use Zephyr's L2 Wi-Fi stack and Networking subsystem (`<zephyr/net/wifi_mgmt.h>`).
- **Protocol (TCP Server Connection):**
  - _Strategy:_ Replace `WiFiClientAdapter`.
  - _Implementation:_ Implement a new `NetClient` using standard BSD sockets (`<zephyr/net/socket.h>`), which Zephyr supports natively.
- **State Machine:**
  - _Strategy:_ Migrate the main `loop()` and `state_machine.cpp`.
  - _Implementation:_ Convert the continuous polling loop to a Zephyr thread with `k_msleep()` or use Zephyr timers.

# Implementation Steps

1. **Scaffold Project & Overlay:** Define the hardware nodes in `.overlay` and configure `prj.conf`.
2. **Zephyr BMI160 & Battery Integration:** Verify the native sensors work.
3. **Zephyr Networking Setup:** Implement Wi-Fi connection and socket-based TCP communication.
4. **Motors & Encoders Port:** Implement PWM, GPIO, and interrupts for drive base.
5. **RPLIDAR Port:** Implement UART parsing for the LiDAR.
6. **Integration:** Bring all components together into the main application thread.

# Verification & Testing

- Ensure the device connects to WiFi and the Python/Rust TCP server successfully.
- Verify telemetry stream is sending accurate `TX_IMU` and `TX_BATTERY` packets.
- Verify bidirectional motor control via `RX_MOTOR_MOVE` commands.
- Verify LiDAR point cloud data is parsed correctly and sent to the server.

# Current status

The migration from Arduino to Zephyr for the HomeRobot project has reached a major milestone. 
The core firmware is operational on the ESP32-C6, and the Control Server (Rust) has been fully migrated to the new Protobuf-based protocol.

## Summary of Completed Work

1. Project Scaffolding:
   - Created the Zephyr application at zephyr-port/robot_app.
   - Configured prj.conf with support for C++20, Networking (Wi-Fi/TCP/Sockets), Sensors, PWM, ADC, and Protobuf (nanopb).

2. Hardware Mapping (Devicetree Overlay):
   - IMU: Configured I2C0 for the BMI160 native Zephyr driver.
   - LiDAR: Configured UART1 for the data stream and a dedicated PWM channel for the LiDAR motor.
   - Motors: Configured 4 PWM channels (LEDC) to handle forward and backward direction control.
   - Encoders: Configured PCNT (Pulse Counter) units for both wheel encoders.

3. Communication Layer & Protocol:
   - Implemented `ZephyrNetClient` (firmware) and `ProtocolManager` (server) using BSD Sockets.
   - Standardized on **Protobuf** with a 2-byte length-prefix framing for all bidirectional communication.
   - Implemented `ProtobufHandler` (firmware) and `sender/reader` modules (server).

4. Sensor & Actuator Port:
   - Ported `Motor` class with PID control and direct PCNT register access.
   - Ported `Lidar` parser for RPLIDAR A1 protocol over UART.
   - Integrated IMU data acquisition.

5. Control Server Migration (Rust):
   - Fully migrated to Protobuf using `prost`.
   - Unified command structure (`RobotCommand`) supporting movement, Lidar control, and configuration updates.
   - Implemented comprehensive telemetry logging for all incoming sensor data.

## Current State & Next Steps

The end-to-end communication pipeline is now functional using the modern Protobuf stack.

- Next Step: Extensive field testing and PID tuning on the physical hardware.
- Following Step: Implement Battery monitoring (ADC) and Encoders telemetry in the main Zephyr loop.
- Following Step: Refine the State Machine logic in Zephyr to match the original Arduino complexity.
