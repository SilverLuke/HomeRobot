#!/bin/bash
set -e

# Clean
rm -f /tmp/.X11-unix/X99 2>/dev/null || true

export WAYLAND_DISPLAY=wayland-1
export XDG_RUNTIME_DIR=/run/user/1000

# Start Xwayland
echo "[1/3] Starting Xwayland on :99..."
Xwayland :99 -ac -noreset > /dev/null 2>&1 &
XWPID=$!
sleep 2

if ! ps -p $XWPID > /dev/null 2>&1; then
    echo "  ERROR: Xwayland failed to start"
    exit 1
fi
echo "  Xwayland running PID=$XWPID"

# Test basic X11 connection
echo "[2/3] Testing X11 display :99..."
export DISPLAY=:99
# Use xprop or similar to test - or just check if libX11 can connect
python3 -c "
import ctypes
libX11 = ctypes.cdll.LoadLibrary('libX11.so.6')
display = libX11.XOpenDisplay(b':99')
if display:
    print('  X11 connection OK, display=', hex(display))
    libX11.XCloseDisplay(display)
else:
    print('  X11 connection FAILED')
" 2>&1 || echo "  python X11 test failed"

# Launch Gazebo with xcb
echo "[3/3] Starting Gazebo with QT_QPA_PLATFORM=xcb DISPLAY=:99..."
export QT_QPA_PLATFORM=xcb
export XAUTHORITY=""
export QT_DEBUG_PLUGINS=1
export GZ_SIM_RESOURCE_PATH="/home/luca/Projects/robotica/homeRobot/software/simulation"

gz sim -r "/home/luca/Projects/robotica/homeRobot/software/simulation/sim.world" > /home/luca/Projects/robotica/homeRobot/software/logs/sim/gz_xcb_debug.log 2>&1 &
GZPID=$!
echo "  Gazebo PID=$GZPID"

sleep 15
echo "--- Status ---"
if ps -p $GZPID > /dev/null 2>&1; then
    echo "Gazebo RUNNING"
else
    echo "Gazebo EXITED"
fi

echo "--- Windows ---"
niri msg windows

echo "--- Log (first 40 lines) ---"
head -40 /home/luca/Projects/robotica/homeRobot/software/logs/sim/gz_xcb_debug.log
