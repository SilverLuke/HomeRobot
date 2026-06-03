#!/bin/bash
set -e

# Clean
pkill -f "[g]z sim" 2>/dev/null || true
pkill -f "[X]wayland" 2>/dev/null || true
sleep 1

export WAYLAND_DISPLAY=wayland-1
export XDG_RUNTIME_DIR=/run/user/1000

# Step 1: Start Xwayland for Ogre2 GLX
echo "[1/3] Starting Xwayland on :99..."
Xwayland :99 -ac -noreset > /dev/null 2>&1 &
sleep 2
echo "  Xwayland PID=$!"

# Step 2: Set correct Qt plugin path (VERSION-MATCHED to Gazebo's Qt 5.15.16)
QT_BASE_PLUGINS="/nix/store/m0mk8wjmpa08zz8zsm5cri0aqyvjj6fj-qtbase-5.15.16-bin/lib/qt-5.15.16/plugins"
QT_WAYLAND_PLUGINS="/nix/store/absxlw79mgpplarhnrn8dzq4haqm8fkh-qtwayland-5.15.16-bin/lib/qt-5.15.16/plugins"

export QT_PLUGIN_PATH="${QT_BASE_PLUGINS}:${QT_WAYLAND_PLUGINS}"
export DISPLAY=:99
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RESOURCE_PATH="/home/luca/Projects/robotica/homeRobot/software/simulation"

echo "[2/3] Qt plugin path: $QT_PLUGIN_PATH"
echo "  Available platform plugins:"
ls "${QT_BASE_PLUGINS}/platforms/" 2>/dev/null

# Step 3: Launch Gazebo
echo "[3/3] Launching Gazebo..."
gz sim -r "/home/luca/Projects/robotica/homeRobot/software/simulation/sim.world" > /home/luca/Projects/robotica/homeRobot/software/logs/sim/gz_fixed.log 2>&1 &
GZPID=$!
echo "  Gazebo PID=$GZPID"

sleep 15
echo "--- Status ---"
if ps -p $GZPID > /dev/null 2>&1; then
    echo "Gazebo is RUNNING!"
else
    echo "Gazebo EXITED"
fi

echo "--- Windows ---"
niri msg windows

echo "--- Log tail ---"
tail -20 /home/luca/Projects/robotica/homeRobot/software/logs/sim/gz_fixed.log
