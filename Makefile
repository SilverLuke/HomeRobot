# Paths
SERVER_DIR = server
ZEPHYR_APP_DIR = zephyr-port/robot_app
ZEPHYR_BOARD_S3 = esp32s3_devkitc/esp32s3/procpu
ZEPHYR_BOARD_C6 = esp32c6_devkitc/esp32c6/hpcore
ZEPHYR_BUILD_DIR = build-zephyr
ESP_DEVICE = /dev/ttyACM0
# Tools
CARGO = cargo
WEST = west

.PHONY: all server zephyr build-s3 build-c6 flash monitor snapshot-logs clean

all: server build-s3

server:
	@echo "Building server with cargo..."
	$(CARGO) build --manifest-path $(SERVER_DIR)/Cargo.toml

zephyr: build-s3

build-s3:
	@echo "Building Zephyr app with west for $(ZEPHYR_BOARD_S3)..."
	$(WEST) build -p -b $(ZEPHYR_BOARD_S3) -d $(ZEPHYR_BUILD_DIR) $(ZEPHYR_APP_DIR)

build-c6:
	@echo "Building Zephyr app with west for $(ZEPHYR_BOARD_C6)..."
	$(WEST) build -p -b $(ZEPHYR_BOARD_C6) -d $(ZEPHYR_BUILD_DIR) $(ZEPHYR_APP_DIR)

flash:
	@echo "Flashing Zephyr app to $(ESP_DEVICE)..."
	$(WEST) flash -d $(ZEPHYR_BUILD_DIR) --esp-device $(ESP_DEVICE)

monitor:
	@echo "Starting west monitor on $(ESP_DEVICE)..."
	$(WEST) monitor -p $(ESP_DEVICE)

snapshot-logs:
	@echo "Capturing $(or $(DURATION),5) seconds of logs from $(ESP_DEVICE)..."
	./tools/read_esp32_logs.sh $(or $(DURATION),5) $(ESP_DEVICE)

clean:
	@echo "Cleaning server..."
	$(CARGO) clean --manifest-path $(SERVER_DIR)/Cargo.toml
	@echo "Cleaning Zephyr build..."
	rm -rf $(ZEPHYR_BUILD_DIR)
