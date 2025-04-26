#include "../utils/utils.h"

#include <Arduino.h>

#define LED_TIMING 250

void led_blink(String seq, uint8_t red, uint8_t green, uint8_t blue) {
  // If the sequence is empty, return immediately
  if (seq.length() == 0) {
    return;
  }

  for (int i = 0; i < seq.length(); i++) {
    char c = seq[i];

    // Blink the LED based on the character
    switch (c) {
      case '.':
        neopixelWrite(RGB_BUILTIN, red, green, blue);
        delay(LED_TIMING);
        break;
      case '-':
        neopixelWrite(RGB_BUILTIN, red, green, blue);
        delay(LED_TIMING * 2);
        break;
      default:
        break;
    }
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
    delay(LED_TIMING);
  }
}

union float_conversions {
  float f;
  uint32_t i;
};

float net2hostFloat(uint32_t net_float) {
  float_conversions val{};
  val.i = ntohl(net_float);
  return val.f;
}

uint32_t host2netFloat(const float host_float) {
  float_conversions val{};
  val.f = host_float;
  val.i = htonl(val.i);

  return val.i;
}
