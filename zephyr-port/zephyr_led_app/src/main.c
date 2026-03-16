#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rmt_tx.h>
#include <stdio.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>

LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

/* Get the RMT channel node from alias or specific label */
#define RMT_LED_NODE DT_NODELABEL(rmt_led)

/* WS2812 timings assuming an 80MHz RMT clock (12.5ns per tick) */
#define T0H 32  // 400ns
#define T0L 68  // 850ns
#define T1H 64  // 800ns
#define T1L 36  // 450ns

void set_ws2812_color(struct rmt_symbol *symbols, uint8_t r, uint8_t g, uint8_t b) {
    // WS2812 expects GRB order
    uint32_t color = (g << 16) | (r << 8) | b;
    for (int i = 0; i < 24; i++) {
        int bit = (color >> (23 - i)) & 1;
        symbols[i].level0 = 1;
        symbols[i].level1 = 0;
        if (bit) {
            symbols[i].duration0 = T1H;
            symbols[i].duration1 = T1L;
        } else {
            symbols[i].duration0 = T0H;
            symbols[i].duration1 = T0L;
        }
    }
}

int main(void)
{
    // Wait for the USB console to stabilize
    k_sleep(K_MSEC(500));
    printk("\n\n*** APPLICATION START ***\n\n");
    LOG_INF("Starting Zephyr LED Blinker");

    const struct device *rmt_dev = NULL;

#if DT_NODE_HAS_STATUS(RMT_LED_NODE, okay)
    rmt_dev = DEVICE_DT_GET(RMT_LED_NODE);
#endif

    if (!device_is_ready(rmt_dev)) {
        LOG_ERR("RMT Device not ready");
        return -1;
    }

    struct rmt_symbol symbols[24];

    // The dynamic signal routing and GPIO scanning logic has been removed as per the request to blink a single LED.
    // It is assumed that the RMT_LED_NODE is properly configured in the device tree to control the target LED.

    while (1) {
        // Assuming max color value is 100 based on previous code. 50% power is 50.
        // Blink Red (1s period, 50% power)
				LOG_INF("LED blink sequence - RED");
        set_ws2812_color(symbols, 50, 0, 0);
        rmt_tx_transmit(rmt_dev, symbols, 24, K_MSEC(500));
        k_sleep(K_MSEC(500)); // LED ON for 1s
        set_ws2812_color(symbols, 0, 0, 0); // Turn off
        rmt_tx_transmit(rmt_dev, symbols, 24, K_MSEC(500));
        k_sleep(K_MSEC(500)); // LED OFF for 1s

        // Blink Green (1s period, 50% power)
				LOG_INF("LED blink sequence - GREEN");
        set_ws2812_color(symbols, 0, 50, 0);
        rmt_tx_transmit(rmt_dev, symbols, 24, K_MSEC(500));
        k_sleep(K_MSEC(500));
        set_ws2812_color(symbols, 0, 0, 0); // Turn off
        rmt_tx_transmit(rmt_dev, symbols, 24, K_MSEC(500));
        k_sleep(K_MSEC(500));

        // Blink Blue (1s period, 50% power)
				LOG_INF("LED blink sequence - BLUE");
        set_ws2812_color(symbols, 0, 0, 50);
        rmt_tx_transmit(rmt_dev, symbols, 24, K_MSEC(500));
        k_sleep(K_MSEC(500));
        set_ws2812_color(symbols, 0, 0, 0); // Turn off
        rmt_tx_transmit(rmt_dev, symbols, 24, K_MSEC(500));
        k_sleep(K_MSEC(500));
    }

    return 0;
}
