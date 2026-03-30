# HomeRobot

This is a personal project, the goal is to create a robot that can navigate in a house, avoid obstacles and do a mapping of the house.

## Hardware

- ESP32-C6
- RP-Lidar A1M8
- 2x DC motors
- 2x Encoders
- 1x IMU (Gyroscope + Accelerometer) BMI160
- 1 H-Bridge dual channel
- 4x Li-Ion 26650 batteries
- 3D printed chassis
- Wires

## Setup and Installation

Follow these steps to set up the development environment for HomeRobot.

### 1. Zephyr Project Setup
Initialize and update the Zephyr project workspace:

```bash
# Initialize the workspace using the zephyrproject-rtos manifest
west init zephyrproject
cd zephyrproject
west update
# Export zephyr for west
west zephyr-export
```

### 2. Zephyr SDK
The project is configured to look for the Zephyr SDK at `zephyr-port/zephyr-sdk-1.0.0` (Note: This matches the expected directory name, but the recommended SDK version is **0.16.8**).

To set it up:
1. Download the Zephyr SDK (Version 0.16.8 or compatible).
2. Extract it into the `zephyr-port/zephyr-sdk-1.0.0` directory.
3. Run the installation script:

```bash
cd zephyr-port/zephyr-sdk-1.0.0
./setup.sh -t all -h -c
```

*Note: In Nix environments, `ZEPHYR_SDK_INSTALL_DIR` is set automatically.*

### 3. Other Prerequisites
The easiest way to manage dependencies is using **Nix** and **direnv**:

```bash
# Install Nix and direnv, then run:
direnv allow
```

This will automatically set up:
- **Rust & Cargo**: Required for the `server/`.
- **Python Environment**: With all dependencies for Zephyr.
- **Protobuf Compiler**: For `nanopb` and server communication.
- **Build Tools**: CMake, Ninja, and cross-compilation toolchains.

If not using Nix, ensure you have the following installed:
- [Rust](https://rustup.rs/)
- [West](https://docs.zephyrproject.org/latest/develop/west/index.html) (`pip3 install west`)
- Protobuf Compiler (`apt install protobuf-compiler`)

### 4. Building and Flashing

Use the `Makefile` at the root for common tasks:

```bash
# Build the Rust server and the Zephyr app (Default 16MB flash)
make all

# Build only the Rust control server
make server

# Build for specific hardware (e.g., 8MB flash)
make build-c6 FLASH=8M

# Flash the Zephyr app to the ESP32-C6
make flash

# Start the serial monitor
make monitor
```

## Related repositories

To be completed
- [HomeRobot-CAD]() - CAD files for the robot
- [HomeRobot-Brain]() - Server that controls the robot

## To Do

- [ ] Check endianness before sending to / receiving from the network



