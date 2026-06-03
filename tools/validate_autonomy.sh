#!/usr/bin/env bash
set -e

# Validation script for autonomous tools
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_ROOT/logs/sim"

echo "=== HomeRobot Autonomy Validation ==="

# 1. Rebuild components
echo "[0/4] Rebuilding components..."
nix-shell --run "cd $PROJECT_ROOT/server && cargo build"
nix-shell --run "cd $PROJECT_ROOT/tools/cmd_sender && cargo build"

# 2. Start headless simulation in the background
echo "[1/4] Starting Headless Simulation..."
# Cleanup first
pkill -9 -f "server" || true
pkill -9 -f "zephyr.exe" || true
pkill -9 -f "gz sim" || true
rm -f "$PROJECT_ROOT/house_map.pgm"
rm -f "$LOG_DIR/simulation.rrd"

nix-shell --run "$PROJECT_ROOT/tools/start_sim_headless.sh" &
SIM_PID=$!

echo "Waiting for simulation to stabilize (15s)..."
sleep 15

# 2. Drive the robot via Proxy
echo "[2/4] Driving robot via Proxy..."
nix-shell --run "cd $PROJECT_ROOT/tools/cmd_sender && cargo run -- --proxy constant --left 100 --right 100 --duration-secs 3"

# 3. Trigger Map Save
echo "[3/4] Triggering Map Save..."
nix-shell --run "cd $PROJECT_ROOT/tools/cmd_sender && cargo run -- --proxy save-map"
sleep 2

# 4. Verify Results
echo "[4/4] Verifying outputs..."
VALID=true

if [ -f "$PROJECT_ROOT/house_map.pgm" ]; then
    echo "  [✓] house_map.pgm generated."
else
    echo "  [✗] house_map.pgm MISSING!"
    VALID=false
fi

if [ -f "$LOG_DIR/simulation.rrd" ]; then
    FILE_SIZE=$(stat -c%s "$LOG_DIR/simulation.rrd")
    if [ "$FILE_SIZE" -gt 1000 ]; then
        echo "  [✓] simulation.rrd recorded ($FILE_SIZE bytes)."
    else
        echo "  [✗] simulation.rrd is too small ($FILE_SIZE bytes)!"
        VALID=false
    fi
else
    echo "  [✗] simulation.rrd MISSING!"
    VALID=false
fi

# Cleanup
echo "Cleaning up processes..."
pkill -9 -f "server" || true
pkill -9 -f "zephyr.exe" || true
pkill -9 -f "gz sim" || true

if [ "$VALID" = true ]; then
    echo "=== VALIDATION SUCCESSFUL ==="
    exit 0
else
    echo "=== VALIDATION FAILED ==="
    exit 1
fi
