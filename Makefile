# Paths
SERVER_DIR = server
ZEPHYR_APP_DIR = zephyr-port/zephyr_led_app
ZEPHYR_BOARD = esp32s3_devkitc/esp32s3/procpu
ZEPHYR_BUILD_DIR = build-zephyr
ESP_DEVICE = /dev/ttyACM0
# Tools
CARGO = cargo
WEST = west

.PHONY: all server zephyr flash clean

all: server zephyr

server:
	@echo "Building server with cargo..."
	$(CARGO) build --manifest-path $(SERVER_DIR)/Cargo.toml

zephyr:
	@echo "Building Zephyr app with west for $(ZEPHYR_BOARD)..."
	$(WEST) build -p -b $(ZEPHYR_BOARD) -d $(ZEPHYR_BUILD_DIR) $(ZEPHYR_APP_DIR)

flash:
	@echo "Flashing Zephyr app to $(ZEPHYR_BOARD)..."
	$(WEST) flash -d $(ZEPHYR_BUILD_DIR) --esp-device $(ESP_DEVICE)

monitor:
	@echo "Starting west monitor on $(ESP_DEVICE)..."
	$(WEST) monitor -p $(ESP_DEVICE)

clean:
	@echo "Cleaning server..."
	$(CARGO) clean --manifest-path $(SERVER_DIR)/Cargo.toml
	@echo "Cleaning Zephyr build..."
	rm -rf $(ZEPHYR_BUILD_DIR)
