{
  description = "HomeRobot Software Environment (Zephyr, Rust, Gazebo 10)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    gazebo-sim-overlay.url = "github:SilverLuke/gazebo-sim-overlay/gazebo-10";
    nixgl.url = "github:guibou/nixGL";
    systems.url = "github:nix-systems/default-linux";
    west2nix = {
      url = "github:adisbladis/west2nix/f84670d66f881d9340b7d7626fbfe499438c134b";
      flake = false;
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      rust-overlay,
      gazebo-sim-overlay,
      nixgl,
      systems,
      west2nix,
      ...
    }:
    let
      lib = nixpkgs.lib;
      forEachSystem = f: lib.genAttrs (import systems) (system: f pkgsFor.${system});
      pkgsFor = lib.genAttrs (import systems) (
        system:
        import nixpkgs {
          inherit system;
          config = {
            allowUnfree = true;
            permittedInsecurePackages = [
              "python3.13-ecdsa-0.19.2"
              "python3.14-ecdsa-0.19.2"
              "freeimage-unstable-2021-11-01"
              "freeimage-3.18.0-unstable-2024-04-18"
            ];
          };
          overlays = [
            rust-overlay.overlays.default
            gazebo-sim-overlay.overlays.default
            nixgl.overlays.default
          ];
        }
      );
    in
    {
      devShells = forEachSystem (
        pkgs:
        let
          west2nixPkg = pkgs.callPackage west2nix {
            callPackage = pkgs.callPackage;
            python3 = pkgs.python3;
          };
          projects = if builtins.pathExists ./projects.nix then import ./projects.nix else null;
          hookContent = if projects != null then west2nixPkg.mkWest2nixHook { manifest = projects; } else "";

          gz-jetty = pkgs.gz-jetty;

          pythonEnv = pkgs.python3.withPackages (
            ps: with ps; [
              anytree canopen intelhex jsonschema packaging patool psutil pyelftools pykwalify
              pylink-square pyserial pyyaml requests reuse semver setuptools tqdm west wheel
              colorama ply coverage pytest mypy junitparser python-dotenv protobuf
              pyocd tabulate natsort cbor python-can spdx-tools tkinter ecdsa
            ]
          );
        in
        {
          default = pkgs.mkShell {
            nativeBuildInputs = with pkgs; [
              protobuf rerun gz-jetty uv libuuid util-linux.dev tinyxml-2 zeromq cppzmq libsodium
              SDL2 gtk4.dev glib.dev cairo.dev pango.dev gdk-pixbuf.dev graphene.dev libadwaita
              pkgs.esptool
              (rust-bin.stable.latest.default.override { extensions = [ "rust-src" "rust-analyzer" "clippy" "rustfmt" ]; })
              ccache cmake dfu-util dtc file gcc gcovr git gperf libusb1 ncurses ninja pkg-config
              unzip wget which xz pythonEnv west2nixPkg.west2nix
              mesa-demos xvfb-run imagemagick xorg-server rsyslog gnumake
              libGL libglvnd mesa wayland vulkan-loader
            ];

            shellHook = ''
              echo "--- HomeRobot (Zephyr + Rust) Nix Environment (Gazebo Jetty 10) ---"
              
              export PYTHONPATH="${gz-jetty}/lib/python:$PYTHONPATH"
              export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath [ pkgs.libglvnd pkgs.vulkan-loader pkgs.libGL pkgs.gtk4 pkgs.glib pkgs.graphene pkgs.pango pkgs.cairo pkgs.gdk-pixbuf pkgs.libadwaita pkgs.SDL2 ]}:${gz-jetty}/lib:$LD_LIBRARY_PATH"
              export LD_LIBRARY_PATH="/run/opengl-driver/lib:/run/opengl-driver-32/lib:$LD_LIBRARY_PATH"
              
              export LIBGL_DRIVERS_PATH="/run/opengl-driver/lib/dri"
              export __EGL_VENDOR_LIBRARY_FILENAMES="/run/opengl-driver/share/glvnd/egl_vendor.d/50_mesa.json"
              
              [ -d "$LIBGL_DRIVERS_PATH" ] || export LIBGL_DRIVERS_PATH="${pkgs.mesa}/lib/dri"
              [ -f "$__EGL_VENDOR_LIBRARY_FILENAMES" ] || export __EGL_VENDOR_LIBRARY_FILENAMES="${pkgs.mesa}/share/glvnd/egl_vendor.d/50_mesa.json"

              export GZ_PARTITION=homerobot_sim
              export GZ_IP=127.0.0.1

              GZ_QT_PLUGIN_PATH=""
              GZ_QML_IMPORT_PATH=""
              for dep in $(nix-store -qR "${gz-jetty}" 2>/dev/null); do
                if [ -d "$dep/lib/qt-6/plugins" ]; then
                  GZ_QT_PLUGIN_PATH="$dep/lib/qt-6/plugins:$GZ_QT_PLUGIN_PATH"
                fi
                if [ -d "$dep/lib/qt-6/qml" ]; then
                  GZ_QML_IMPORT_PATH="$dep/lib/qt-6/qml:$GZ_QML_IMPORT_PATH"
                fi
              done
              
              export QT_PLUGIN_PATH="$GZ_QT_PLUGIN_PATH"
              export QML_IMPORT_PATH="$GZ_QML_IMPORT_PATH"
              export QML2_IMPORT_PATH="$GZ_QML_IMPORT_PATH"
              
              if [ -n "$WAYLAND_DISPLAY" ]; then
                 echo "  [INFO] Wayland detected ($WAYLAND_DISPLAY). Using native Wayland/EGL stack."
                 export QT_QPA_PLATFORM="xcb"
                 export GDK_BACKEND="x11"
                 export QT_QPA_PLATFORMTHEME="fusion"
                 export XDG_SESSION_TYPE="x11"
              fi

              export OGRE2_RESOURCE_PATH="${pkgs.ogre-next}/lib/OGRE-Next"
              export GZ_SIM_RESOURCE_PATH="${pkgs.ogre-next}/share/OGRE-Next/Media:$GZ_SIM_RESOURCE_PATH"
              export OGRE_MEDIA_PATH="${pkgs.ogre-next}/share/OGRE-Next/Media"

              if [ -d "$PWD/zephyrproject/zephyr" ]; then
                export ZEPHYR_BASE="$PWD/zephyrproject/zephyr"
              fi

              [ -f .env ] && { set -o allexport; source .env; set +o allexport; }

              export CCACHE_DIR="$PWD/.ccache"
              export PATH="${pkgs.ccache}/lib/ccache:$PATH"
              export CMAKE_C_COMPILER_LAUNCHER=ccache
              export CMAKE_CXX_COMPILER_LAUNCHER=ccache

              export GZ_SIM_RESOURCE_PATH="$PWD/simulation:$GZ_SIM_RESOURCE_PATH"

              ${hookContent}

              # --- Local Rsyslog Daemon Management ---
              mkdir -p "$PWD/logs/rsyslog"
              mkdir -p "$PWD/logs/remote"

              cat <<'RSLOG' > "$PWD/logs/rsyslog.conf"
module(load="imudp")
input(type="imudp" port="1514")

module(load="imtcp")
input(type="imtcp" port="1514")

global(workDirectory="$PWD/logs/rsyslog")

template(name="RemoteLogs" type="string"
  string="$PWD/logs/remote/%HOSTNAME%/%$YEAR%-%$MONTH%-%$DAY%.log"
)

*.* ?RemoteLogs
*.* @127.0.0.1:5140
RSLOG

              if [[ $- == *i* ]]; then
                  _HR_COUNTER="$PWD/logs/rsyslog/shell_count"
                  _hr_n=$(cat "$_HR_COUNTER" 2>/dev/null || echo 0)
                  echo $((_hr_n + 1)) > "$_HR_COUNTER"

                  _HR_RUNNING=0
                  if [ -f "$PWD/logs/rsyslog/rsyslogd.pid" ]; then
                      _HR_PID=$(cat "$PWD/logs/rsyslog/rsyslogd.pid" 2>/dev/null)
                      if [ -n "$_HR_PID" ] && kill -0 "$_HR_PID" 2>/dev/null; then
                          echo "  [INFO] Local rsyslogd is already running (PID $_HR_PID)."
                          _HR_RUNNING=1
                      fi
                  fi

                  if [ $_HR_RUNNING -eq 0 ]; then
                      echo "  [INFO] Starting local Rsyslog daemon on port 1514..."
                      rsyslogd -i "$PWD/logs/rsyslog/rsyslogd.pid" -f "$PWD/logs/rsyslog.conf"
                  fi

                  trap '
                      _HR_COUNTER="$PWD/logs/rsyslog/shell_count"
                      _hr_n=$(cat "$_HR_COUNTER" 2>/dev/null || echo 1)
                      _hr_n=$((_hr_n - 1))
                      [ $_hr_n -lt 0 ] && _hr_n=0
                      echo $_hr_n > "$_HR_COUNTER"
                      if [ $_hr_n -eq 0 ]; then
                          if [ -f "$PWD/logs/rsyslog/rsyslogd.pid" ]; then
                              _HR_PID=$(cat "$PWD/logs/rsyslog/rsyslogd.pid" 2>/dev/null)
                              if [ -n "$_HR_PID" ]; then
                                  echo "Stopping local rsyslogd (PID $_HR_PID)..."
                                  kill "$_HR_PID" 2>/dev/null || true
                              fi
                          fi
                      fi
                  ' EXIT
              fi

              echo "  [✓] Environment ready. 'west' and Gazebo Jetty are available."
            '';
          };
        }
      );
    };
}
