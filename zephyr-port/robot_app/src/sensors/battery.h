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
    
    // Voltage divider: Adjusted based on 15V telemetry (Raw 2453 -> ~2.63V at ADC)
    // 15.0 / 2.63 = 5.7
    static constexpr float VOLTAGE_DIVIDER_RATIO = 5.7f; 
};
