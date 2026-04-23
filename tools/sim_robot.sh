#!/usr/bin/env bash
# Restarts Bridge, Firmware, and Dashboard
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_ROOT/logs/sim"
export GZ_PARTITION=homerobot_sim
export GZ_IP=127.0.0.1
export PYTHONPATH=$PYTHONPATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH
export QT_QPA_PLATFORM=xcb

echo "--- Restarting Robot Logic Stack ---"

# 1. Cleanup old logic components (but NOT Gazebo)
pkill -9 -f "zephyr.exe" || true
pkill -9 -f "gazebo_bridge.py" || true
pkill -9 -f "server/target/debug/server" || true
sleep 1

# 2. Start UDP Bridge
echo "[1/3] Starting Bridge..."
python3 -u "$PROJECT_ROOT/tools/gazebo_bridge.py" > "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!

# 3. Start Dashboard
echo "[2/3] Starting Dashboard..."
stdbuf -oL -eL "$PROJECT_ROOT/server/target/debug/server" > "$LOG_DIR/server.log" 2>&1 &
SERVER_PID=$!

# 4. Start Zephyr
echo "[3/3] Starting Zephyr Firmware..."
ZEPHYR_EXE="$PROJECT_ROOT/build/sim/zephyr/zephyr.exe"
stdbuf -oL -eL "$ZEPHYR_EXE" > "$LOG_DIR/zephyr.log" 2>&1 &
ZEPHYR_PID=$!

echo "Robot logic is running. Dashboard is open."
# Cleanup on exit
trap "kill -9 $BRIDGE_PID $SERVER_PID $ZEPHYR_PID" EXIT
wait
