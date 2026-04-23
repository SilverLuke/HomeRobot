#!/usr/bin/env bash
# Starts only the Gazebo World
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
export GZ_SIM_RESOURCE_PATH="$PROJECT_ROOT/simulation:$GZ_SIM_RESOURCE_PATH"
export GZ_PARTITION=homerobot_sim
export GZ_IP=127.0.0.1
LOG_DIR="$PROJECT_ROOT/logs/sim"

mkdir -p "$LOG_DIR"

echo "--- Starting Gazebo Physics Server ---"
echo "Logs available at $LOG_DIR/gazebo.log"
gz sim -r "$PROJECT_ROOT/simulation/sim.world" > "$LOG_DIR/gazebo.log" 2>&1
