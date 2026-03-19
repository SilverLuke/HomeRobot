#!/usr/bin/env bash
# tools/read_esp32_logs.sh - Capture a snapshot of ESP32 logs.

# Default values
DURATION=${1:-5}
DEVICE=${2:-/dev/ttyACM0}
BAUD=${3:-115200}

echo "--- Capturing ESP32 logs for $DURATION seconds on $DEVICE ---"
echo "(Waiting for device to be ready...)"

# Wait up to 5 seconds for the device to appear
for i in {1..10}; do
    if [ -e "$DEVICE" ]; then
        break
    fi
    sleep 0.5
done

if [ ! -e "$DEVICE" ]; then
    echo "Error: Device $DEVICE not found after waiting."
    exit 1
fi

# Try west espressif monitor first
if command -v west &> /dev/null; then
    # We use a subshell to capture output and handle the potential disconnect
    # 2>&1 to see everything.
    timeout --foreground --signal=SIGINT "${DURATION}s" west espressif monitor -p "$DEVICE" 2>&1
    
    RET=$?
    # 124 is timeout (which is what we expect)
    if [ $RET -eq 124 ] || [ $RET -eq 0 ]; then
        echo "--- Capture complete (via west) ---"
        exit 0
    fi
    # If it failed with 1 (often serial error), we'll try picocom as fallback
    echo "West monitor exited with $RET, trying picocom..."
fi

# Fallback to picocom
if command -v picocom &> /dev/null; then
    timeout --foreground --signal=SIGINT "${DURATION}s" picocom -b "$BAUD" -f n "$DEVICE" 2>&1
    echo "--- Capture complete (via picocom) ---"
else
    echo "Error: Neither 'west espressif monitor' nor 'picocom' worked or are installed."
    exit 1
fi
