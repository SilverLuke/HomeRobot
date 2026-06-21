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
    echo "  [INFO] Wayland detected ($WAYLAND_DISPLAY). Forcing Qt/GDK to X11/XCB for OGRE compatibility."
    export QT_QPA_PLATFORM=xcb
    export GDK_BACKEND=x11
    export QT_QPA_PLATFORMTHEME=fusion
    export XDG_SESSION_TYPE=x11
    
    # Workaround: Some OGRE/Gazebo components still want DISPLAY even on Wayland
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=:0
        echo "  [INFO] Setting DISPLAY=:0 as fallback for OGRE."
    fi
fi

# Disable DRI3 to force DRI2, which avoids Xwayland OpenGL context sharing BadAccess crashes
export LIBGL_DRI3_DISABLE=1

# Ensure Ogre2 can find its plugins and Media (HLMS shaders)
if [ -z "$OGRE2_RESOURCE_PATH" ] || [ -z "$OGRE_MEDIA_PATH" ]; then
    echo "  [WARNING] OGRE2 paths not fully set, attempting fallback..."
    OGRE_NEXT_PATH=$(nix-build -E 'let pkgs = import <nixpkgs> { overlays = [ (final: prev: import (let src = fetchTarball "https://github.com/SilverLuke/gazebo-sim-overlay/archive/pr-gazebo-10-support.tar.gz"; in "${src}/pkgs") { pkgs = final; }) ]; }; in pkgs.ogre-next' --no-out-link)
    
    if [ -z "$OGRE2_RESOURCE_PATH" ]; then
        export OGRE2_RESOURCE_PATH="$OGRE_NEXT_PATH/lib/OGRE-Next"
    fi
    if [ -z "$OGRE_MEDIA_PATH" ]; then
        export OGRE_MEDIA_PATH="$OGRE_NEXT_PATH/share/OGRE-Next/Media"
        export GZ_SIM_RESOURCE_PATH="$OGRE_MEDIA_PATH:$GZ_SIM_RESOURCE_PATH"
    fi
fi


echo "  [INFO] World: $PROJECT_ROOT/simulation/sim.world"
echo "  [INFO] Logs: $LOG_DIR/gazebo.log"

# Clean up any old instances
pkill -f gz-sim-server > /dev/null 2>&1

# Detect if nixGLIntel is available and wrap the Gazebo command to use host GPU drivers
GZ_LAUNCH_CMD="gz sim"
if command -v nixGLIntel >/dev/null 2>&1; then
    echo "  [INFO] nixGLIntel detected, wrapping Gazebo with GPU acceleration wrapper."
    GZ_LAUNCH_CMD="nixGLIntel gz sim"
fi

# Run Gazebo using OpenGL with DRI3 disabled for Xwayland stability.
$GZ_LAUNCH_CMD -v 4 -r "$PROJECT_ROOT/simulation/sim.world" 2>&1 | tee "$LOG_DIR/gazebo.log"
