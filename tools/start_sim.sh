#!/usr/bin/env bash

# Robust project root detection
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." && pwd )"
cd "$PROJECT_ROOT"
SIM_DIR="$PROJECT_ROOT/simulation"
# HR_WORLD selects the world by basename: sim (default, empty arena) or
# house (4 rooms + furniture). Both define world name "homerobot_world" so
# the ground-truth topic and all tooling stay identical.
WORLD_FILE="$SIM_DIR/${HR_WORLD:-sim}.world"
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

# 1. Force Cleanup — PATH-SCOPED to this checkout. This machine can host
# multiple HomeRobot checkouts (dev copy + CI/benchmark sandbox); generic
# name-based patterns here once killed the other checkout's sim mid-session.
echo "[0/4] Cleaning up old processes of $PROJECT_ROOT..."
pkill -9 -f "$PROJECT_ROOT/build/sim/sim/zephyr/zephyr.exe" || true
pkill -9 -f "gz.*$WORLD_FILE" || true
pkill -9 -f "$SERVER_BIN" || true
sleep 1

XWAYLAND_PID=""

# 2. Ensure an X11 bridge is available (non-headless, Wayland session)
if [ "$HEADLESS_MODE" = false ] && [ -n "$WAYLAND_DISPLAY" ]; then
    # Gazebo's Ogre2 renderer (QT_QPA_PLATFORM=xcb) and the GTK4 dashboard
    # (GDK_BACKEND=x11, see shell.nix) both need an X11 display. niri has no
    # native Xwayland support, so a per-window bridge (xwayland-satellite) is
    # required to give each X11 window its own real Wayland toplevel -
    # without it, X11 clients end up rendered as undecorated child windows
    # inside a single merged Wayland surface ("one strange window with 2
    # subwindows"). This repo's niri config already spawns xwayland-satellite
    # at startup on DISPLAY=:0 (see NixOS_configs/common/home-manager/apps/niri.nix),
    # so reuse it instead of starting a second, competing bridge.
    if [ -n "$DISPLAY" ]; then
        echo "[1/4] Reusing existing X11 bridge at DISPLAY=$DISPLAY"
    elif command -v xwayland-satellite >/dev/null 2>&1; then
        XWAYLAND_DISPLAY=:99
        rm -f "/tmp/.X11-unix/X${XWAYLAND_DISPLAY#:}" 2>/dev/null || true
        echo "[1/4] No DISPLAY set; starting xwayland-satellite on $XWAYLAND_DISPLAY..."
        xwayland-satellite $XWAYLAND_DISPLAY > /dev/null 2>&1 &
        XWAYLAND_PID=$!
        sleep 2
        if ps -p $XWAYLAND_PID > /dev/null 2>&1; then
            export DISPLAY=$XWAYLAND_DISPLAY
            echo "  xwayland-satellite running (PID=$XWAYLAND_PID, DISPLAY=$DISPLAY)"
        else
            echo "  WARNING: xwayland-satellite failed to start. Gazebo GUI may not work."
            XWAYLAND_PID=""
        fi
    else
        XWAYLAND_DISPLAY=:99
        rm -f "/tmp/.X11-unix/X${XWAYLAND_DISPLAY#:}" 2>/dev/null || true
        echo "[1/4] No DISPLAY and no xwayland-satellite found; falling back to bare Xwayland."
        echo "  WARNING: without per-window Xwayland integration, Gazebo and the dashboard"
        echo "  will likely render inside a single merged window on niri."
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
    fi
elif [ "$HEADLESS_MODE" = true ]; then
    echo "[1/4] Headless Display setup: Starting Xvfb on :99 for virtual X11 framebuffer"
    export LIBGL_ALWAYS_SOFTWARE=1
    export GALLIUM_DRIVER=llvmpipe
    Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render -noreset > /dev/null 2>&1 &
    XVFB_PID=$!
    export DISPLAY=:99
    sleep 2
else
    echo "[1/4] Display setup: using existing DISPLAY=$DISPLAY"
fi

# The simulation must not fight the real robot for the server: bind to
# localhost only so LAN robots cannot connect to this instance (issues.md #5).
export HR_BIND=127.0.0.1

# 3. Start the Rust Server (Dashboard)
if [ "$HEADLESS_MODE" = true ]; then
    echo "[2/4] Starting Headless Rust Server in Xvfb..."
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
# DO NOT export GZ_SIM_DISABLE_RENDERING=1 here if we want screenshots!

# Detect if nixGLIntel is available and wrap the Gazebo command to use host GPU drivers
GZ_LAUNCH_CMD="gz sim"
if command -v nixGLIntel >/dev/null 2>&1; then
    echo "  [INFO] nixGLIntel detected, wrapping Gazebo with GPU acceleration wrapper."
    GZ_LAUNCH_CMD="nixGLIntel gz sim"
fi

if [ "$HEADLESS_MODE" = true ]; then
    echo "[3/4] Starting Headless Gazebo Server..."
    $GZ_LAUNCH_CMD -s -r "$WORLD_FILE" > "$LOG_DIR/gazebo.log" 2>&1 &
    GZ_PID=$!
else
    echo "[3/4] Starting Gazebo Physics Server & GUI..."
    $GZ_LAUNCH_CMD -r "$WORLD_FILE" > "$LOG_DIR/gazebo.log" 2>&1 &
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
