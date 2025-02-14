#pragma once
#include <cstdint>


/**
* @brief Get the battery level in percentage, from 0 to 100
*/
long battery_level();

/**
* @brief Get the raw value of the battery, from 0 to 4095
*/
uint16_t battery_raw();

/**
* @brief Initialize the battery sensor, return 1 if the battery is not connected
*/
uint8_t init_battery();