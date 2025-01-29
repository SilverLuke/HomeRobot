#pragma once

#include <Arduino.h>


#define LED_TIMING 500
// Sequence is a string in a morse like format - and . if - blink longer than . than power off the led
void led_blink(String seq, uint8_t red, uint8_t green, uint8_t blue) {

  // If the sequence is empty, return immediately
  if (seq.length() == 0) {
    return;
  }
  for(int i = 0; i < seq.length(); i++){
    char c = seq[i];

    // Blink the LED based on the character
    switch (c) {
      case '.':
        neopixelWrite(RGB_BUILTIN, red, green, blue);
        delay(LED_TIMING);
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
        break;
      case '-':
        neopixelWrite(RGB_BUILTIN, red, green, blue);
        delay(LED_TIMING * 2);
        neopixelWrite(RGB_BUILTIN, 0, 0, 0);
        break;
      default:;
    }
  }
}
