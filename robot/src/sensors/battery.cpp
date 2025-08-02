#include "sensors/battery.h"

#include <Arduino.h>
#include <Elog.h>

#include <cstdint>

#include "definitions.h"

#define ADC_BATTERY_PIN 2
// Values in the ADC range 0-4095
#define BATTERY_MAX 2990
#define BATTERY_MIN 2250  // TODO find a better value
#define BATTERY_NOT_CONNECTED 800
// Values in mV
#define BATTERY_MAX_VOLTAGE 16500

long battery_level() {
  int adcValue = analogRead(ADC_BATTERY_PIN);
  return map(adcValue, BATTERY_MIN, BATTERY_MAX, 0, 100);
}

long battery_voltage() {
  int adcValue = analogRead(ADC_BATTERY_PIN);
  return map(adcValue, 0, BATTERY_MAX, 0, BATTERY_MAX_VOLTAGE);
}

uint16_t battery_raw() {
  return analogRead(ADC_BATTERY_PIN);
}

uint8_t init_battery() {
  pinMode(ADC_BATTERY_PIN, INPUT);
  analogReadResolution(12);

  if (battery_raw() < BATTERY_NOT_CONNECTED) {
    return 1;
  }
  return 0;
  
}

void show_battery() {
  Logger.info(MAIN_LOGGER, "Battery: %i \%, %d mV, %i raw", battery_level(), battery_voltage(), battery_raw());
}