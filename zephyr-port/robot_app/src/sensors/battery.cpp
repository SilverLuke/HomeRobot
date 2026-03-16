#include "battery.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

Battery::Battery(const struct device* adc_dev, uint8_t channel)
    : adc_dev_(adc_dev), channel_(channel) {
}

bool Battery::init() {
    if (!device_is_ready(adc_dev_)) {
        LOG_ERR("ADC device not ready");
        return false;
    }

    channel_cfg_ = {
        .gain = ADC_GAIN_1_4,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = channel_,
    };

    int err = adc_channel_setup(adc_dev_, &channel_cfg_);
    if (err) {
        LOG_ERR("Failed to setup ADC channel (err %d)", err);
        return false;
    }

    return true;
}

int32_t Battery::read_raw() {
    int16_t sample_buffer[1];
    struct adc_sequence sequence = {
        .channels = BIT(channel_),
        .buffer = sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution = 12,
    };

    int err = adc_read(adc_dev_, &sequence);
    if (err) {
        LOG_ERR("ADC read failed (err %d)", err);
        return -1;
    }

    return sample_buffer[0];
}

uint32_t Battery::get_voltage_mv() {
    int32_t raw = read_raw();
    if (raw < 0) return 0;

    // Convert raw to millivolts (reference is ~1100mV, 12-bit is 4095)
    // with 1/4 gain -> 1100 * 4 = 4400mV max
    int32_t mv = raw;
    adc_raw_to_millivolts(adc_ref_internal(adc_dev_), ADC_GAIN_1_4, 12, &mv);
    
    // Applying the voltage divider ratio
    return (uint32_t)((float)mv * VOLTAGE_DIVIDER_RATIO);
}

uint32_t Battery::get_percentage() {
    uint32_t mv = get_voltage_mv();
    if (mv >= BATTERY_MAX_VOLTAGE) return 100;
    if (mv <= BATTERY_MIN_VOLTAGE) return 0;

    return (mv - BATTERY_MIN_VOLTAGE) * 100 / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE);
}
