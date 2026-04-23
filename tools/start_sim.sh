#!/usr/bin/env bash

# Robust project root detection
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SIM_DIR="$PROJECT_ROOT/simulation"
WORLD_FILE="$SIM_DIR/sim.world"
MODEL_PATH="$SIM_DIR"
SERVER_BIN="$PROJECT_ROOT/server/target/debug/server"
LOG_DIR="$PROJECT_ROOT/logs/sim"

# Ensure environment is fully loaded for background sub-shells
export PATH=$PATH
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH

mkdir -p "$LOG_DIR"

echo "--- HomeRobot Headless Simulation (Data-Stream Mode) ---"

# 1. Force Cleanup and Archive Logs
echo "[0/4] Cleaning up and archiving logs..."
mkdir -p "$LOG_DIR/previous"
mv "$LOG_DIR"/*.log "$LOG_DIR/previous/" 2>/dev/null || true

pkill -9 -f "zephyr.exe" || true
pkill -9 -f "gazebo_bridge.py" || true
pkill -9 -f "gz sim" || true
pkill -9 -f "$SERVER_BIN" || true
sleep 2

# 2. Start the Rust Server
echo "[1/4] Starting Rust Server (Dashboard UI)..."
if [ ! -f "$SERVER_BIN" ]; then
    echo "Building server first..."
    cd "$PROJECT_ROOT/server" && cargo build && cd "$PROJECT_ROOT"
fi
# Use stdbuf to ensure logs are written immediately
stdbuf -oL -eL "$SERVER_BIN" > "$LOG_DIR/server.log" 2>&1 &
SERVER_PID=$!

# Wait for server to start listening
echo "Waiting for dashboard to bind to port 12345..."
MAX_RETRIES=20
RETRY_COUNT=0
while ! ss -lnt | grep -q :12345; do
    sleep 1
    ((RETRY_COUNT++))
    if [ $RETRY_COUNT -ge $MAX_RETRIES ]; then
        echo "ERROR: Dashboard UI failed to start. See $LOG_DIR/server.log"
        exit 1
    fi
done

# 3. Start Gazebo Sim (GUI Mode)
echo "[2/4] Starting Gazebo Physics Server..."
export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:$GZ_SIM_RESOURCE_PATH"
gz sim -r "$WORLD_FILE" > "$LOG_DIR/gazebo.log" 2>&1 &
GZ_PID=$!

echo "Waiting for Physics Server to stabilize (10s)..."
sleep 10

# 4. Start the UDP Bridge
echo "[3/4] Starting Gazebo-Zephyr UDP Bridge..."
python3 -u "$PROJECT_ROOT/tools/gazebo_bridge.py" > "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!

# 5. Start Zephyr (native_sim)
echo "[4/4] Starting Zephyr native_sim..."
# Rebuild since we changed native_sim.conf
make build-sim > /dev/null 2>&1
ZEPHYR_EXE="$PROJECT_ROOT/build/sim/zephyr/zephyr.exe"
if [ -f "$ZEPHYR_EXE" ]; then
    echo "--------------------------------------------------------"
    echo "SUCCESS: HEADLESS SIMULATION ACTIVE."
    echo "Use WASD in your GTK window to move the robot."
    echo "Logs available in: $LOG_DIR"
    echo "--------------------------------------------------------"
    stdbuf -oL -eL "$ZEPHYR_EXE" > "$LOG_DIR/zephyr.log" 2>&1 &
    ZEPHYR_PID=$!
else
    echo "ERROR: Zephyr executable not found."
fi

# Cleanup on exit
trap "kill -9 $GZ_PID $BRIDGE_PID $SERVER_PID $ZEPHYR_PID" EXIT

# Keep script alive to maintain cleanup trap
wait
