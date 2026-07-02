# Sources paths
SERVER_DIR = server
ZEPHYR_APP_DIR = robot
ZEPHYR_BOARD_S3 = esp32s3_devkitc/esp32s3/procpu
ZEPHYR_BOARD_C6 = esp32c6_devkitc/esp32c6/hpcore
ZEPHYR_BOARD_SIM = native_sim/native/64
# Build paths

ZEPHYR_BUILD_DIR = build/sim

# Hardware variations
FLASH ?= 16M
SNIPPET = espressif-flash-$(FLASH)

ESP_DEVICE = /dev/ttyACM0
# Tools
CARGO = cargo
WEST = west

EXTRA_CMAKE_ARGS =
ifeq ($(POWER_SUPPLY), 1)
  EXTRA_CMAKE_ARGS += -Dapp_DISABLE_BATTERY_CHECK=y
endif
ifeq ($(POWER_SUPPLY), y)
  EXTRA_CMAKE_ARGS += -Dapp_DISABLE_BATTERY_CHECK=y
endif
ifeq ($(POWER_SUPPLY), ON)
  EXTRA_CMAKE_ARGS += -Dapp_DISABLE_BATTERY_CHECK=y
endif

.PHONY: all server build-s3 build-c6 build-sim all-sim build-lidar-debug flash monitor snapshot-logs clean proto help ota-flash ota-confirm ci



all: proto server build-c6 ## Build everything (proto, server, and main C6 firmware)

all-sim: proto server build-sim ## Build all components required for simulation (proto, server, and sim firmware)

help: ## Show this help message
	@echo "HomeRobot Build System - Available Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

proto: ## Generate Python Protobuf bindings
	@echo "Generating Protobuf bindings..."
	protoc --python_out=proto/ -Iproto/ proto/messages.proto

ci: ## Run server CI (clippy + tests) on the build machine
	./tools/ci.sh

server: ## Build the Rust control server/dashboard
	@echo "Building server with cargo..."
	$(CARGO) build --manifest-path $(SERVER_DIR)/Cargo.toml

build-s3: ## Build robot firmware for ESP32-S3
	@echo "Building Zephyr app with west for $(ZEPHYR_BOARD_S3)..."
	$(WEST) build -p -b $(ZEPHYR_BOARD_S3) -d $(ZEPHYR_BUILD_DIR) $(ZEPHYR_APP_DIR)

build-lidar-debug: ## Build standalone Lidar test app for ESP32-C6
	@echo "Building Lidar Debug app with west for $(ZEPHYR_BOARD_C6)..."
	$(WEST) build -p -b $(ZEPHYR_BOARD_C6) -S $(SNIPPET) -d $(ZEPHYR_BUILD_DIR) zephyr-port/lidar_debug_app -- -DCONFIG_ESPTOOLPY_FLASHSIZE_$(FLASH)B=y

build-c6: ## Build main robot firmware for ESP32-C6 (includes MCUboot via sysbuild)
	@echo "Building Zephyr app with west for $(ZEPHYR_BOARD_C6) with $(FLASH) flash using sysbuild..."
	$(WEST) build -p --sysbuild -b $(ZEPHYR_BOARD_C6) -S $(SNIPPET) -d $(ZEPHYR_BUILD_DIR) $(ZEPHYR_APP_DIR) -Dmcuboot_EXTRA_CONF_FILE=$(PWD)/robot/sysbuild/mcuboot.conf -Dapp_CONFIG_ESPTOOLPY_FLASHSIZE_$(FLASH)B=y $(EXTRA_CMAKE_ARGS)

build-sim: ## Build robot firmware for native_sim (Gazebo)
	@echo "Building Zephyr app for simulation ($(ZEPHYR_BOARD_SIM))..."
	@export ZEPHYR_BASE=$$(pwd)/zephyrproject/zephyr; \
	$(WEST) build -p -b $(ZEPHYR_BOARD_SIM) -d $(ZEPHYR_BUILD_DIR)/sim $(ZEPHYR_APP_DIR) -- -DKCONFIG_WERROR=OFF -DCONF_FILE="prj.conf;boards/native_sim.conf"

FLASH_BAUD   ?= 460800
FLASH_BEFORE ?= default-reset

MCUBOOT_BIN  = $(ZEPHYR_BUILD_DIR)/mcuboot/zephyr/zephyr.bin
APP_BIN      = $(ZEPHYR_BUILD_DIR)/robot/zephyr/zephyr.signed.bin

flash: ## Flash MCUboot + app to the ESP32 (use FLASH_BEFORE=no-reset after manual BOOT+EN)
	@echo "Flashing MCUboot + app to $(ESP_DEVICE) at $(FLASH_BAUD) bps..."
	esptool --chip esp32c6 --port $(ESP_DEVICE) --baud $(FLASH_BAUD) \
	  --before $(FLASH_BEFORE) --after hard-reset \
	  write-flash --flash-mode dio --flash-freq 40m --flash-size 16MB \
	  0x0     $(MCUBOOT_BIN) \
	  0x20000 $(APP_BIN)

ROBOT_IP ?= 192.168.199.123
MCUMGR ?= $(shell which mcumgr 2>/dev/null || echo $(HOME)/go/bin/mcumgr)

ota-flash: ## Flash the signed app image to the robot wirelessly over Wi-Fi, test it, and reboot
	@echo "Uploading signed image to robot at $(ROBOT_IP)..."
	$(MCUMGR) --conntype udp --connstring="[$(ROBOT_IP)]:1337" image upload build/sim/robot/zephyr/zephyr.signed.bin
	@echo "Comparing image hashes..."
	@HASH0=$$($(MCUMGR) --conntype udp --connstring="[$(ROBOT_IP)]:1337" image list | grep -A 4 "slot=0" | grep "hash:" | awk '{print $$2}'); \
	HASH1=$$($(MCUMGR) --conntype udp --connstring="[$(ROBOT_IP)]:1337" image list | grep -A 4 "slot=1" | grep "hash:" | awk '{print $$2}'); \
	if [ -z "$$HASH1" ]; then \
	  echo "Error: Could not retrieve Slot 1 image hash. Make sure the upload completed successfully."; \
	  exit 1; \
	fi; \
	if [ "$$HASH0" = "$$HASH1" ]; then \
	  echo "New image hash is identical to the currently running image ($$HASH0)."; \
	  echo "No swap or reboot is necessary."; \
	else \
	  echo "Marking new image in Slot 1 ($$HASH1) as pending test..."; \
	  $(MCUMGR) --conntype udp --connstring="[$(ROBOT_IP)]:1337" image test $$HASH1; \
	  echo "Rebooting robot..."; \
	  $(MCUMGR) --conntype udp --connstring="[$(ROBOT_IP)]:1337" reset; \
	fi


ota-confirm: ## Confirm the booted OTA image to make it permanent (retries if the robot is rebooting)
	@echo "Waiting for the robot at $(ROBOT_IP) to boot and respond..."
	@for i in $$(seq 1 12); do \
	  if $(MCUMGR) --conntype udp --connstring="[$(ROBOT_IP)]:1337" --timeout 2.5 --tries 1 image list > /dev/null 2>&1; then \
	    echo "Robot is online! Confirming the new image..."; \
	    HASH=$$($(MCUMGR) --conntype udp --connstring="[$(ROBOT_IP)]:1337" image list | grep -A 4 "slot=0" | grep "hash:" | awk '{print $$2}'); \
	    if [ -n "$$HASH" ]; then \
	      $(MCUMGR) --conntype udp --connstring="[$(ROBOT_IP)]:1337" image confirm $$HASH; \
	      exit 0; \
	    else \
	      echo "Error: Could not retrieve Slot 0 hash to confirm."; \
	      exit 1; \
	    fi; \
	  fi; \
	  echo "  Retrying in 2 seconds... ($$i/12)"; \
	  sleep 2; \
	done; \
	echo "Error: Robot did not respond within the timeout period."; \
	exit 1



test: ## Run verification tests
	@echo "Running Server unit tests..."
	$(CARGO) test --manifest-path $(SERVER_DIR)/Cargo.toml
	@echo "Running Regression tests..."
	python3 tools/regression_test.py

clean: ## Remove build artifacts
	@echo "Cleaning server..."
	$(CARGO) clean --manifest-path $(SERVER_DIR)/Cargo.toml
	@echo "Cleaning Zephyr build..."
	rm -rf $(ZEPHYR_BUILD_DIR)
