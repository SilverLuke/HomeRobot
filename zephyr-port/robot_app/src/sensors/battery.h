#pragma once

#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

class Battery {
public:
    Battery(const struct device* adc_dev, uint8_t channel);

    bool init();
    
    /**
     * @brief Read raw ADC value
     */
    int32_t read_raw();

    /**
     * @brief Get battery voltage in millivolts
     */
    uint32_t get_voltage_mv();

    /**
     * @brief Get battery percentage (0-100)
     */
    uint32_t get_percentage();

private:
    const struct device* adc_dev_;
    uint8_t channel_;
    struct adc_channel_cfg channel_cfg_;
    
    static constexpr uint32_t BATTERY_MAX_VOLTAGE = 16800; // 4S Li-ion fully charged
    static constexpr uint32_t BATTERY_MIN_VOLTAGE = 12000; // 4S Li-ion discharged (3.0V per cell)
    
    // Voltage divider: R1=100k, R2=10k -> V_adc = V_bat * (10 / (100+10)) = V_bat / 11
    // ESP32-C6 ADC range is 0-3.3V (with 1/4 gain it's roughly 0-1.1V * 4 = 4.4V max)
    // Actually gain 1/4 on ESP32 means 0dB to 11dB attenuation.
    // Let's assume standard divider and reference.
    static constexpr float VOLTAGE_DIVIDER_RATIO = 11.0f; 
};
