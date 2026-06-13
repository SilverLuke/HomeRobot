#!/usr/bin/env bash
set -e

TARGET_DIR="/home/luca/Projects/robotica/homeRobot/software/zephyrproject"
WORK_DIR="/home/luca/Projects/robotica/homeRobot/software"

echo "======================================"
echo "    Zephyr Workspace Setup Script"
echo "======================================"

if [ -d "$TARGET_DIR/.west" ]; then
    echo "Found existing west workspace at $TARGET_DIR"
    echo "Running 'west update' to sync modules..."
    cd "$TARGET_DIR"
    west update
else
    echo "Initializing new west workspace..."
    cd "$WORK_DIR"
    
    # You can customize the init command if you need a specific Zephyr version
    # e.g., west init -m https://github.com/zephyrproject-rtos/zephyr --mr v3.5.0 zephyrproject
    west init zephyrproject
    
    cd zephyrproject
    echo "Fetching modules with 'west update'. This may take a while..."
    west update
    
    # Optional: export Zephyr CMake package
    # west zephyr-export
fi

echo "======================================"
echo "Zephyr setup is complete in $TARGET_DIR!"
