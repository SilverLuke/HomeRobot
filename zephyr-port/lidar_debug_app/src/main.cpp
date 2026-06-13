#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define LIDAR_UART DT_ALIAS(lidar_uart)
#define LIDAR_EN   DT_ALIAS(lidar_en)

static const struct device *uart_dev = DEVICE_DT_GET(LIDAR_UART);
static const struct gpio_dt_spec motor_gpio = GPIO_DT_SPEC_GET(LIDAR_EN, gpios);

int main(void) {
    printk("\n--- RPLIDAR Minimal Debug App ---\n");

    if (!device_is_ready(uart_dev)) {
        printk("Error: UART device not ready\n");
        return 1;
    }

    if (!gpio_is_ready_dt(&motor_gpio)) {
        printk("Error: Motor GPIO not ready\n");
        return 1;
    }

    gpio_pin_configure_dt(&motor_gpio, GPIO_OUTPUT_ACTIVE);
    printk("LiDAR Motor ENABLED\n");

    // Give it time to spin up
    k_msleep(1000);

    // RESET Command
    /*
    printk("Sending RESET (0xA5 0x40)...\n");
    uart_poll_out(uart_dev, 0xA5);
    uart_poll_out(uart_dev, 0x40);

    // Wait for reset banner
    k_msleep(2000);
    */

    // Flush any power-up/reset noise
    uint8_t dummy;
    int flushed = 0;
    while (uart_poll_in(uart_dev, &dummy) == 0) {
        flushed++;
    }
    printk("Flushed %d bytes of initial noise\n", flushed);

    // SCAN Command
    printk("Sending SCAN (0xA5 0x20)...\n");
    uart_poll_out(uart_dev, 0xA5);
    uart_poll_out(uart_dev, 0x20);

    printk("Entering read loop. Printing all bytes received (hex). Resending SCAN every 5s.\n");

    uint32_t total_received = 0;
    int64_t last_cmd_sent = k_uptime_get();

    while (1) {
        uint8_t rx_byte;
        if (uart_poll_in(uart_dev, &rx_byte) == 0) {
            printk("0x%02x ", rx_byte);
            total_received++;
            if (total_received % 16 == 0) {
                printk("\n");
            }
        }

        if (k_uptime_get() - last_cmd_sent > 5000) {
            printk("\n[Periodic] Resending SCAN (0xA5 0x20)...\n");
            uart_poll_out(uart_dev, 0xA5);
            uart_poll_out(uart_dev, 0x20);
            last_cmd_sent = k_uptime_get();
        }

        k_yield();
    }
}
