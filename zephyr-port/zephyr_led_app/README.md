# Zephyr LED Application with RMT Support

This is a Zephyr application that demonstrates LED control using both standard GPIO and a custom RMT (Remote Control Peripheral) driver. It is designed to run on ESP32-C6 and ESP32-S3 DevKitC boards.

## Learning Objectives

- **Hardware Abstraction**: See how the same application can use different hardware backends (RMT vs GPIO).
- **Custom Drivers**: Learn how to include and use an out-of-tree custom driver (`rmt_tx`).
- **Devicetree Overlays**: Use board-specific overlays to define peripheral nodes and signal mappings.
- **Error Handling**: Graceful fallback from a specialized peripheral to a generic one.

## Features

- **RMT Driver Integration**: Includes the custom `rmt_tx` driver moved from the `esp32s3_devkitc` project.
- **Multi-Board Support**:
  - **ESP32-C6 DevKit**: Uses GPIO8 (onboard WS2812) via RMT or GPIO.
  - **ESP32-S3 DevKit**: Uses GPIO48 (onboard WS2812) via RMT or GPIO.
- **Smart Fallback**: Automatically switches to GPIO control if the RMT driver is not yet fully implemented or fails.

## Building

Ensure your `ZEPHYR_BASE` is set to the central `zephyrproject/zephyr` directory.

### Build for ESP32-C6 DevKit

```bash
west build -p -b esp32c6_devkitc/esp32c6/hpcore zephyr-port/zephyr_led_app
```

### Build for ESP32-S3 DevKit

```bash
west build -p -b esp32s3_devkitc/esp32s3/procpu zephyr-port/zephyr_led_app
```

## Project Structure

- `drivers/rmt_tx/`: The custom RMT driver implementation.
- `include/zephyr/drivers/rmt_tx.h`: The Zephyr-style API definition for the RMT driver.
- `dts/bindings/`: Devicetree bindings for the custom RMT peripheral.
- `boards/`: Board-specific devicetree overlays.
- `src/main.c`: Application logic with RMT and GPIO support.
