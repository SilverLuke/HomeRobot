#!/usr/bin/env bash
set -e

# Autonomous exploration test with Rerun recording
PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )"
LOG_DIR="$PROJECT_ROOT/logs/sim"

echo "=== HomeRobot Autonomous Rerun Test ==="

# 1. Rebuild Server
echo "[1/4] Rebuilding Server..."
nix-shell --run "cd $PROJECT_ROOT/server && cargo build"

# 2. Start Headless Simulation
echo "[2/4] Starting Headless Simulation..."
pkill -9 -f "server" || true
pkill -9 -f "zephyr.exe" || true
pkill -9 -f "gz sim" || true
rm -f "$LOG_DIR/simulation.rrd"

nix-shell --run "$PROJECT_ROOT/tools/start_sim_headless.sh" &
SIM_PID=$!

echo "Waiting for simulation to stabilize (15s)..."
sleep 15

# 3. Enable Autonomy via Proxy
echo "[3/4] Enabling Autonomous Exploration..."
# We send a dummy lidar command first to wake up the system if needed, then enable exploration
nix-shell --run "cd $PROJECT_ROOT/tools/cmd_sender && cargo run -- --proxy lidar --active"
sleep 2
nix-shell --run "cd $PROJECT_ROOT/tools/cmd_sender && cargo run -- --proxy explore --enabled"

echo "Autonomous exploration active. Let's record for 60 seconds..."
sleep 60

# 4. Stop and Verify
echo "[4/4] Stopping Exploration and Verifying Rerun recording..."
nix-shell --run "cd $PROJECT_ROOT/tools/cmd_sender && cargo run -- --proxy explore"
sleep 2
nix-shell --run "cd $PROJECT_ROOT/tools/cmd_sender && cargo run -- --proxy save-map"
sleep 2

# Cleanup
pkill -9 -f "server" || true
pkill -9 -f "zephyr.exe" || true
pkill -9 -f "gz sim" || true

if [ -f "$LOG_DIR/simulation.rrd" ]; then
    SIZE=$(stat -c%s "$LOG_DIR/simulation.rrd")
    echo "SUCCESS: Rerun recording created ($SIZE bytes)."
    echo "View it with: rerun $LOG_DIR/simulation.rrd"
else
    echo "FAILED: simulation.rrd missing."
    exit 1
fi
