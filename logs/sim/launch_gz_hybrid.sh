#!/bin/bash
set -e

# Kill previous instances
pkill -f "[g]z sim" 2>/dev/null || true
pkill -f "[X]wayland" 2>/dev/null || true
sleep 1

# Wayland compositor variables
export WAYLAND_DISPLAY=wayland-1
export XDG_RUNTIME_DIR=/run/user/1000

# Step 1: Start Xwayland for GLX/OpenGL support (Ogre2 needs this)
echo "[1/2] Starting Xwayland on :99..."
Xwayland :99 -ac -noreset > /dev/null 2>&1 &
XWPID=$!
sleep 2

if ps -p $XWPID > /dev/null 2>&1; then
    echo "  Xwayland running (PID=$XWPID)"
else
    echo "  ERROR: Xwayland failed to start"
    exit 1
fi

# Step 2: Launch Gazebo with hybrid approach
# - Qt GUI uses native Wayland (QT_QPA_PLATFORM=wayland)
# - Ogre2 uses Xwayland GLX context via DISPLAY=:99
export DISPLAY=:99
export QT_QPA_PLATFORM=wayland
export GZ_SIM_RESOURCE_PATH="/home/luca/Projects/robotica/homeRobot/software/simulation"

echo "[2/2] Starting Gazebo (Qt=Wayland, Ogre2=GLX via :99)..."
gz sim -r "/home/luca/Projects/robotica/homeRobot/software/simulation/sim.world" > /home/luca/Projects/robotica/homeRobot/software/logs/sim/gz_hybrid.log 2>&1 &
GZPID=$!
echo "  Gazebo PID=$GZPID"

sleep 12
echo "--- Status Check ---"

if ps -p $GZPID > /dev/null 2>&1; then
    echo "Gazebo is RUNNING"
else
    echo "Gazebo EXITED"
fi

echo "--- Windows ---"
niri msg windows

echo "--- Ogre2 log ---"
tail -10 ~/.gz/rendering/ogre2.log 2>/dev/null || echo "no ogre2 log"

echo "--- Gazebo log tail ---"
tail -20 /home/luca/Projects/robotica/homeRobot/software/logs/sim/gz_hybrid.log
