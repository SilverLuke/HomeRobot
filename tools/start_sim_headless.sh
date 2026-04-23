#!/usr/bin/env bash

# Headless simulation starter for autonomous testing
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SIM_DIR="$PROJECT_ROOT/simulation"
WORLD_FILE="$SIM_DIR/sim.world"
MODEL_PATH="$SIM_DIR"
LOG_DIR="$PROJECT_ROOT/logs/sim"

export GZ_IP=127.0.0.1
export GZ_PARTITION=homerobot_sim
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:$GZ_SIM_RESOURCE_PATH"
export PYTHONPATH=$PYTHONPATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH

mkdir -p "$LOG_DIR"

echo "--- HomeRobot Headless Simulation (No Dashboard) ---"

# 1. Cleanup
pkill -9 -f "zephyr.exe" || true
pkill -9 -f "gazebo_bridge.py" || true
pkill -9 -f "gz sim" || true
sleep 1

# 2. Start Gazebo Sim (Headless-ish, we want physics running)
echo "[1/3] Starting Gazebo Physics Server..."
export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:$GZ_SIM_RESOURCE_PATH"
# Run with -s for server-only (headless)
gz sim -r -s "$WORLD_FILE" > "$LOG_DIR/gazebo.log" 2>&1 &
GZ_PID=$!

echo "Waiting for Physics Server (5s)..."
sleep 5

# 3. Start the UDP Bridge
echo "[2/3] Starting Gazebo-Zephyr UDP Bridge..."
python3 -u "$PROJECT_ROOT/tools/gazebo_bridge.py" > "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!

# 4. Start Zephyr (native_sim)
echo "[3/3] Starting Zephyr native_sim..."
ZEPHYR_EXE="$PROJECT_ROOT/build/sim/zephyr/zephyr.exe"
stdbuf -oL -eL "$ZEPHYR_EXE" > "$LOG_DIR/zephyr.log" 2>&1 &
ZEPHYR_PID=$!

echo "Simulation running. PIDs: GZ=$GZ_PID, Bridge=$BRIDGE_PID, Zephyr=$ZEPHYR_PID"
echo "You can now run tools/auto_test.py"

# Cleanup on exit
trap "kill -9 $GZ_PID $BRIDGE_PID $ZEPHYR_PID" EXIT
wait
