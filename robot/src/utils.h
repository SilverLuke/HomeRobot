#pragma once

#include <cstdint>
#include <WString.h>

#define LED_BRIGHTNESS RGB_BRIGHTNESS

#define LED_RED     RGB_BRIGHTNESS, 0,              0
#define LED_GREEN   0,              RGB_BRIGHTNESS, 0
#define LED_BLUE    0,              0,              RGB_BRIGHTNESS
#define LED_YELLOW  RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0
#define LED_PURPLE  RGB_BRIGHTNESS, 0,              RGB_BRIGHTNESS
#define LED_CYAN    0,              RGB_BRIGHTNESS, RGB_BRIGHTNESS
#define LED_WHITE   RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS
#define LED_OFF     0,              0,              0

/**
 * @brief Set the LED to blink with a specific sequence, end then turn off the
 * LED
 * @param[in] sequence Is a string in a morse like format - and . if - blink
 * longer than .
 * @param[in] red Red color brightness
 * @param[in] green Green color brightness
 * @param[in] blue Blue color brightness
 */
void led_blink(String sequence, uint8_t red, uint8_t green, uint8_t blue);
