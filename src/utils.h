#pragma once

#include <cstdint>
#include <WString.h>

#define LED_BRIGHTNESS RGB_BRIGHTNESS

#define RED     RGB_BRIGHTNESS, 0,              0
#define GREEN   0,              RGB_BRIGHTNESS, 0
#define BLUE    0,              0,              RGB_BRIGHTNESS
#define YELLOW  RGB_BRIGHTNESS, RGB_BRIGHTNESS, 0
#define PURPLE  RGB_BRIGHTNESS, 0,              RGB_BRIGHTNESS
#define CYAN    0,              RGB_BRIGHTNESS, RGB_BRIGHTNESS
#define WHITE   RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS

// Sequence is a string in a morse like format - and . if - blink longer than . than power off the led
void led_blink(String seq, uint8_t red, uint8_t green, uint8_t blue);
