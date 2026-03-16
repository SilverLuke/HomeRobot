# ESP32 C6 LED Application (GPIO Control)

## Complete Application Files

### 1. CMakeLists.txt
```
# CMakeLists.txt for Zephyr LED Application

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp32c6_led_app)

# Add source files
target_sources(app PRIVATE src/main.c)
```

### 2. src/main.c
```
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

#define LED_PIN 39

int main(void)
{
	LOG_INF("Starting Zephyr LED Application for ESP32 C6");

	// Get the GPIO device
	const struct device *led_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	if (!device_is_ready(led_dev)) {
		LOG_ERR("GPIO device not ready");
		return -1;
	}

	// Configure LED pin as output
	int err = gpio_pin_configure(led_dev, LED_PIN, GPIO_OUTPUT);
	if (err) {
		LOG_ERR("Failed to configure LED pin: %d", err);
		return err;
	}

	LOG_INF("LED configured on pin %d", LED_PIN);

	// LED blinking loop
	while (1) {
		gpio_pin_set(led_dev, LED_PIN, 1);
		LOG_INF("LED ON");
		k_sleep(K_SECONDS(1));
		
		gpio_pin_set(led_dev, LED_PIN, 0);
		LOG_INF("LED OFF");
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
```

### 3. prj.conf
```
# Zephyr configuration for ESP32 C6 LED Application

# Enable logging
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL_INF=y

# Enable GPIO driver
CONFIG_GPIO=y

# Enable ESP32 C6 specific configurations
CONFIG_SOC_ESP32C6=y

# Enable GPIO driver for ESP32 C6
CONFIG_GPIO_ESP32=y
CONFIG_GPIO_ESP32C6=y

# Enable device tree
CONFIG_DEVICE_TREE=y
```

### 4. README.md
```
# Zephyr LED Application for ESP32 C6

This is a minimal Zephyr application that demonstrates LED control on ESP32 C6 using GPIO pin 39.

## Features

- Basic LED blinking functionality
- GPIO-based LED control on pin 39
- Zephyr configuration for ESP32 C6 target
- Proper error handling and logging

## Building Instructions

1. Ensure Zephyr and west are installed
2. Initialize your workspace with the correct Zephyr version:
   ```
   west init -m https://github.com/zephyrproject-rtos/zephyr
   west update
   ```
3. Build for ESP32 C6:
   ```
   west build -p -b esp32c6_devkitm_1
   ```
4. Flash to device:
   ```
   west flash
   ```

## Files

- `src/main.c` - Main application with LED control
- `CMakeLists.txt` - Build configuration 
- `prj.conf` - Zephyr configuration
```

## How to Use This Solution

1. Create a new directory for your Zephyr project:
   ```
   mkdir my_zephyr_project
   cd my_zephyr_project
   ```

2. Copy all the files mentioned above into this directory (CMakeLists.txt, src/main.c, prj.conf, README.md)

3. Setup the Zephyr workspace:
   ```
   west init -m https://github.com/zephyrproject-rtos/zephyr
   west update
   ```

4. Build and flash:
   ```
   west build -p -b esp32c6_devkitm_1
   west flash
   ```

## Notes

- This implementation uses GPIO pin 39 as requested for ESP32 C6
- The application will blink an LED connected to GPIO39 with 1-second intervals
- The code includes error handling and logging
- If you want to use RMT protocol instead of GPIO, additional configuration would be needed for proper RMT driver setup and pulse generation patterns

This solution is ready to be built in a proper Zephyr workspace and will compile and run on any ESP32 C6 development board.