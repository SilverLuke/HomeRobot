#!/bin/bash
set -e

export WAYLAND_DISPLAY=wayland-1
export XDG_RUNTIME_DIR=/run/user/1000
export QT_QPA_PLATFORM=wayland
unset DISPLAY
export GZ_SIM_RESOURCE_PATH="/home/luca/Projects/robotica/homeRobot/software/simulation"

echo "Starting Gazebo with QT_QPA_PLATFORM=wayland, headless-rendering..."
gz sim -r --headless-rendering "/home/luca/Projects/robotica/homeRobot/software/simulation/sim.world" > /home/luca/Projects/robotica/homeRobot/software/logs/sim/gz_wayland_hr.log 2>&1 &
GZPID=$!
echo "Gazebo PID=$GZPID"

sleep 12
echo "--- Checking ---"

if ps -p $GZPID > /dev/null 2>&1; then
    echo "Gazebo is RUNNING"
    ps -p $GZPID -o pid,stat,cmd --no-headers
else
    echo "Gazebo EXITED"
fi

echo "--- Windows ---"
niri msg windows

echo "--- Log snippet ---"
head -30 /home/luca/Projects/robotica/homeRobot/software/logs/sim/gz_wayland_hr.log
