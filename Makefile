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

.PHONY: all server build-s3 build-c6 build-sim all-sim build-lidar-debug flash monitor snapshot-logs clean proto help

all: proto server build-c6 ## Build everything (proto, server, and main C6 firmware)

all-sim: proto server build-sim ## Build all components required for simulation (proto, server, and sim firmware)

help: ## Show this help message
	@echo "HomeRobot Build System - Available Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'

proto: ## Generate Python Protobuf bindings
	@echo "Generating Protobuf bindings..."
	protoc --python_out=proto/ -Iproto/ proto/messages.proto

server: ## Build the Rust control server/dashboard
	@echo "Building server with cargo..."
	$(CARGO) build --manifest-path $(SERVER_DIR)/Cargo.toml

build-s3: ## Build robot firmware for ESP32-S3
	@echo "Building Zephyr app with west for $(ZEPHYR_BOARD_S3)..."
	$(WEST) build -p -b $(ZEPHYR_BOARD_S3) -d $(ZEPHYR_BUILD_DIR) $(ZEPHYR_APP_DIR)

build-lidar-debug: ## Build standalone Lidar test app for ESP32-C6
	@echo "Building Lidar Debug app with west for $(ZEPHYR_BOARD_C6)..."
	$(WEST) build -p -b $(ZEPHYR_BOARD_C6) -S $(SNIPPET) -d $(ZEPHYR_BUILD_DIR) zephyr-port/lidar_debug_app -- -DCONFIG_ESPTOOLPY_FLASHSIZE_$(FLASH)B=y

build-c6: ## Build main robot firmware for ESP32-C6
	@echo "Building Zephyr app with west for $(ZEPHYR_BOARD_C6) with $(FLASH) flash..."
	$(WEST) build -p -b $(ZEPHYR_BOARD_C6) -S $(SNIPPET) -d $(ZEPHYR_BUILD_DIR) $(ZEPHYR_APP_DIR) -- -DCONFIG_ESPTOOLPY_FLASHSIZE_$(FLASH)B=y

build-sim: ## Build robot firmware for native_sim (Gazebo)
	@echo "Building Zephyr app for simulation ($(ZEPHYR_BOARD_SIM))..."
	@export ZEPHYR_BASE=$$(pwd)/zephyrproject/zephyr; \
	$(WEST) build -p -b $(ZEPHYR_BOARD_SIM) -d $(ZEPHYR_BUILD_DIR)/sim $(ZEPHYR_APP_DIR) -- -DKCONFIG_WERROR=OFF -DCONF_FILE="prj.conf;boards/native_sim.conf"

flash: ## Flash the current build to the ESP32
	@echo "Flashing Zephyr app to $(ESP_DEVICE)..."
	$(WEST) flash -d $(ZEPHYR_BUILD_DIR) --esp-device $(ESP_DEVICE)

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
