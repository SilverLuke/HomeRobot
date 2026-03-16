#include <zephyr/ztest.h>
#include <stdint.h>
#include <stddef.h>

// Mock the rmt_symbol struct for host-based testing on native_posix,
// as this platform does not support the RMT peripheral.
#ifndef RMT_SYMBOL_STRUCT_DEFINED
#define RMT_SYMBOL_STRUCT_DEFINED
struct rmt_symbol {
	uint16_t duration0 : 15;
	uint16_t level0 : 1;
	uint16_t duration1 : 15;
	uint16_t level1 : 1;
};
#endif

/* Definitions from main.c for WS2812 timings */
#define T0H 24  // 300ns
#define T0L 72  // 900ns
#define T1H 72  // 900ns
#define T1L 24  // 300ns

/* The function under test, copied from main.c */
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

ZTEST_SUITE(ws2812_color_suite, NULL, NULL, NULL, NULL, NULL);

ZTEST(ws2812_color_suite, test_set_black)
{
    struct rmt_symbol symbols[24];
    set_ws2812_color(symbols, 0, 0, 0);

    for (int i = 0; i < 24; i++) {
        zassert_equal(symbols[i].duration0, T0H, "Bit %d: T0H mismatch for black", i);
        zassert_equal(symbols[i].duration1, T0L, "Bit %d: T0L mismatch for black", i);
    }
}

ZTEST(ws2812_color_suite, test_set_white)
{
    struct rmt_symbol symbols[24];
    set_ws2812_color(symbols, 255, 255, 255);

    for (int i = 0; i < 24; i++) {
        zassert_equal(symbols[i].duration0, T1H, "Bit %d: T1H mismatch for white", i);
        zassert_equal(symbols[i].duration1, T1L, "Bit %d: T1L mismatch for white", i);
    }
}

ZTEST(ws2812_color_suite, test_set_red)
{
    struct rmt_symbol symbols[24];
    set_ws2812_color(symbols, 255, 0, 0); // r=255, g=0, b=0

    // GRB order: g (bits 0-7), r (bits 8-15), b (bits 16-23)
    for (int i = 0; i < 8; i++) { // Green bits
        zassert_equal(symbols[i].duration0, T0H, "Green Bit %d: T0H mismatch for red", i);
    }
    for (int i = 8; i < 16; i++) { // Red bits
        zassert_equal(symbols[i].duration0, T1H, "Red Bit %d: T1H mismatch for red", i);
    }
    for (int i = 16; i < 24; i++) { // Blue bits
        zassert_equal(symbols[i].duration0, T0H, "Blue Bit %d: T0H mismatch for red", i);
    }
}

ZTEST(ws2812_color_suite, test_set_green)
{
    struct rmt_symbol symbols[24];
    set_ws2812_color(symbols, 0, 255, 0); // r=0, g=255, b=0

    // GRB order: g (bits 0-7), r (bits 8-15), b (bits 16-23)
    for (int i = 0; i < 8; i++) { // Green bits
        zassert_equal(symbols[i].duration0, T1H, "Green Bit %d: T1H mismatch for green", i);
    }
    for (int i = 8; i < 16; i++) { // Red bits
        zassert_equal(symbols[i].duration0, T0H, "Red Bit %d: T0H mismatch for green", i);
    }
    for (int i = 16; i < 24; i++) { // Blue bits
        zassert_equal(symbols[i].duration0, T0H, "Blue Bit %d: T0H mismatch for green", i);
    }
}

ZTEST(ws2812_color_suite, test_set_blue)
{
    struct rmt_symbol symbols[24];
    set_ws2812_color(symbols, 0, 0, 255); // r=0, g=0, b=255

    // GRB order: g (bits 0-7), r (bits 8-15), b (bits 16-23)
    for (int i = 0; i < 16; i++) { // Green and Red bits
        zassert_equal(symbols[i].duration0, T0H, "GR Bit %d: T0H mismatch for blue", i);
    }
    for (int i = 16; i < 24; i++) { // Blue bits
        zassert_equal(symbols[i].duration0, T1H, "Blue Bit %d: T1H mismatch for blue", i);
    }
}
