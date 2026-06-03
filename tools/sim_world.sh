#!/usr/bin/env bash
# Starts the Gazebo World with GUI (Jetty)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

export GZ_SIM_RESOURCE_PATH="$PROJECT_ROOT/simulation:$GZ_SIM_RESOURCE_PATH"
export GZ_PARTITION=homerobot_sim
export GZ_IP=127.0.0.1
LOG_DIR="$PROJECT_ROOT/logs/sim"

mkdir -p "$LOG_DIR"

echo "--- Starting Gazebo Sim (Jetty) ---"

# Wayland handling
if [ -n "$WAYLAND_DISPLAY" ]; then
    echo "  [INFO] Wayland detected ($WAYLAND_DISPLAY)."
    export QT_QPA_PLATFORM=wayland
    export GDK_BACKEND=wayland
    export QT_QPA_PLATFORMTHEME=fusion
    export XDG_SESSION_TYPE=wayland
    
    # Mesa/EGL tweaks for Wayland
    export EGL_PLATFORM=wayland
    
    # Workaround: Some OGRE/Gazebo components still want DISPLAY even on Wayland
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:0
        echo "  [INFO] Setting DISPLAY=:0 as fallback for OGRE."
    fi
fi

# Ensure Ogre2 can find its plugins
if [ -z "$OGRE2_RESOURCE_PATH" ]; then
    echo "  [WARNING] OGRE2_RESOURCE_PATH not set, attempting fallback..."
    OGRE_NEXT_PATH=$(nix-build -E 'let pkgs = import <nixpkgs> { overlays = [ (final: prev: import ./gazebo-sim-overlay/pkgs { pkgs = final; }) ]; }; in pkgs.ogre-next' --no-out-link)
    export OGRE2_RESOURCE_PATH="$OGRE_NEXT_PATH/lib/OGRE-Next"
fi


echo "  [INFO] World: $PROJECT_ROOT/simulation/sim.world"
echo "  [INFO] Logs: $LOG_DIR/gazebo.log"

# Clean up any old instances
pkill -f gz-sim-server > /dev/null 2>&1

# Run Gazebo.
gz sim -v 4 -r "$PROJECT_ROOT/simulation/sim.world" 2>&1 | tee "$LOG_DIR/gazebo.log"
