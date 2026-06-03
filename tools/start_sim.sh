#!/usr/bin/env bash

# Robust project root detection
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
SIM_DIR="$PROJECT_ROOT/simulation"
WORLD_FILE="$SIM_DIR/sim.world"
MODEL_PATH="$SIM_DIR"
SERVER_BIN="$PROJECT_ROOT/server/target/debug/server"
LOG_DIR="$PROJECT_ROOT/logs/sim"

mkdir -p "$LOG_DIR"

# Parse arguments
HEADLESS_MODE=false
for arg in "$@"; do
    if [ "$arg" = "--headless" ]; then
        HEADLESS_MODE=true
    fi
done

if [ "$HEADLESS_MODE" = true ]; then
    echo "--- HomeRobot Headless Simulation (Direct Link Mode) ---"
else
    echo "--- HomeRobot Native Simulation (Direct Link Mode) ---"
fi

# 1. Force Cleanup
echo "[0/4] Cleaning up old processes..."
pkill -9 -f "zephyr.exe" || true
pkill -f "[g]z sim" || true
pkill -9 -f "gazebo_bridge.main" || true
pkill -9 -f "$SERVER_BIN" || true
sleep 1

XWAYLAND_PID=""

# 2. Start Xwayland if needed (non-headless, Wayland session, no DISPLAY)
if [ "$HEADLESS_MODE" = false ] && [ -n "$WAYLAND_DISPLAY" ]; then
    # Gazebo's Ogre2 renderer uses GLX which requires an X11 display.
    # On Wayland-only compositors (like niri), we need Xwayland to bridge.
    XWAYLAND_DISPLAY=:99
    echo "[1/4] Starting Xwayland on $XWAYLAND_DISPLAY for Ogre2 GLX..."

    # Clean stale socket
    rm -f "/tmp/.X11-unix/X${XWAYLAND_DISPLAY#:}" 2>/dev/null || true

    Xwayland $XWAYLAND_DISPLAY -ac -noreset > /dev/null 2>&1 &
    XWAYLAND_PID=$!
    sleep 2

    if ps -p $XWAYLAND_PID > /dev/null 2>&1; then
        export DISPLAY=$XWAYLAND_DISPLAY
        export LIBGL_ALWAYS_SOFTWARE=1
        export GALLIUM_DRIVER=llvmpipe
        echo "  Xwayland running (PID=$XWAYLAND_PID, DISPLAY=$DISPLAY)"
    else
        echo "  WARNING: Xwayland failed to start. Gazebo GUI may not work."
        XWAYLAND_PID=""
    fi
else
    echo "[1/4] Display setup: using existing DISPLAY=$DISPLAY"
fi

# 3. Start the Rust Server (Dashboard)
if [ "$HEADLESS_MODE" = true ]; then
    echo "[2/4] Starting Headless Rust Server..."
    if [ ! -f "$SERVER_BIN" ]; then
        echo "Building server first..."
        cd "$PROJECT_ROOT/server" && cargo build && cd "$PROJECT_ROOT"
    fi
    HEADLESS=1 stdbuf -oL -eL "$SERVER_BIN" > "$LOG_DIR/server.log" 2>&1 &
    SERVER_PID=$!
else
    echo "[2/4] Starting Rust Server..."
    if [ ! -f "$SERVER_BIN" ]; then
        echo "Building server first..."
        cd "$PROJECT_ROOT/server" && cargo build && cd "$PROJECT_ROOT"
    fi
    stdbuf -oL -eL "$SERVER_BIN" > "$LOG_DIR/server.log" 2>&1 &
    SERVER_PID=$!
fi

# Wait for server
sleep 2

# 4. Start Gazebo Sim
export GZ_SIM_RESOURCE_PATH="$MODEL_PATH:$GZ_SIM_RESOURCE_PATH"

if [ "$HEADLESS_MODE" = true ]; then
    echo "[3/4] Starting Headless Gazebo Physics Server..."
    gz sim -r -s "$WORLD_FILE" > "$LOG_DIR/gazebo.log" 2>&1 &
    GZ_PID=$!
else
    echo "[3/4] Starting Gazebo Physics Server & GUI..."
    gz sim -r "$WORLD_FILE" > "$LOG_DIR/gazebo.log" 2>&1 &
    GZ_PID=$!
fi

echo "Waiting for Physics Server to stabilize..."
if [ "$HEADLESS_MODE" = true ]; then
    sleep 5
else
    sleep 8
fi

# 5. Start Zephyr (native_sim)
echo "[4/4] Starting Zephyr native_sim (Direct Link)..."
ZEPHYR_EXE="$PROJECT_ROOT/build/sim/sim/zephyr/zephyr.exe"
if [ ! -f "$ZEPHYR_EXE" ]; then
    echo "Building Zephyr Sim..."
    make build-sim
fi

stdbuf -oL -eL "$ZEPHYR_EXE" > "$LOG_DIR/zephyr.log" 2>&1 &
ZEPHYR_PID=$!

echo "--------------------------------------------------------"
if [ "$HEADLESS_MODE" = true ]; then
    echo "SUCCESS: HEADLESS DIRECT LINK SIMULATION ACTIVE."
    echo "PIDs: Server=$SERVER_PID, GZ=$GZ_PID, Zephyr=$ZEPHYR_PID"
    echo "Telemetry is being recorded to logs/sim/simulation.rrd"
else
    echo "SUCCESS: DIRECT LINK SIMULATION ACTIVE."
    echo "Use WASD in your GTK window to move the robot."
fi
echo "--------------------------------------------------------"

# Cleanup on exit (include Xwayland if we started it)
cleanup() {
    kill -9 $GZ_PID $SERVER_PID $ZEPHYR_PID 2>/dev/null
    [ -n "$XWAYLAND_PID" ] && kill -9 $XWAYLAND_PID 2>/dev/null
}
trap cleanup EXIT
wait

