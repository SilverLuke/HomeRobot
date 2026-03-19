#include "encoders.h"
#include <zephyr/logging/log.h>
#include <soc/pcnt_struct.h>

LOG_MODULE_REGISTER(encoders, LOG_LEVEL_INF);

Encoders::Encoders(const struct device* pcnt_unit_dev, uint8_t unit_idx)
    : dev_(pcnt_unit_dev), unit_idx_(unit_idx) {
}

bool Encoders::init() {
    if (!device_is_ready(dev_)) {
        LOG_ERR("Encoder device %s not ready", dev_->name);
        return false;
    }

    // Ensure the PCNT peripheral clock is enabled and the unit is not paused.
    // Bit 16 is the clock gate for the whole PCNT module in C6.
    PCNT.ctrl.val |= (1 << 16); 
    
    // Clear the reset and pause bits for this unit in the CTRL register.
    // Bit (unit*2) is reset, bit (unit*2 + 1) is pause.
    PCNT.ctrl.val &= ~(1 << (unit_idx_ * 2));
    PCNT.ctrl.val &= ~(1 << (unit_idx_ * 2 + 1));

    reset();
    LOG_INF("Encoder %s (unit %d) initialized and reset", dev_->name, unit_idx_);
    return true;
}

int32_t Encoders::get_ticks() {
    struct sensor_value val;
    // We try the sensor API first as it's the most correct way in Zephyr.
    int err = sensor_sample_fetch(dev_);
    if (err == 0) {
        // Many ESP32 drivers use SENSOR_CHAN_ROTATION for pulse counters.
        err = sensor_channel_get(dev_, SENSOR_CHAN_ROTATION, &val);
        if (err == 0) {
            return (int32_t)val.val1;
        }
    }

    // Fallback: direct register access if the sensor API is not supported or fails.
    // For ESP32-C6, the counter registers are at base + 0x30 + unit_idx * 4.
    // According to the SVD, the CNT register is exactly at this offset.
    return (int32_t)(int16_t)PCNT.cnt_unit[unit_idx_].pulse_cnt;
}

void Encoders::reset() {
    // Reset the pulse counter for this unit using ESP32 CTRL register.
    // Bit (unit*2) is the reset bit.
    PCNT.ctrl.val |= (1 << (unit_idx_ * 2));
    k_busy_wait(10);
    PCNT.ctrl.val &= ~(1 << (unit_idx_ * 2));
}
