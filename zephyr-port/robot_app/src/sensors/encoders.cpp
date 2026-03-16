#include "encoders.h"
#include <zephyr/logging/log.h>
#include <soc/pcnt_struct.h>

LOG_MODULE_REGISTER(encoders, LOG_LEVEL_INF);

Encoders::Encoders(const struct device* pcnt_dev, uint8_t unit_idx)
    : dev_(pcnt_dev), unit_idx_(unit_idx) {
}

bool Encoders::init() {
    if (!device_is_ready(dev_)) {
        LOG_ERR("PCNT device not ready");
        return false;
    }
    return true;
}

int32_t Encoders::get_ticks() {
    // Directly read from the ESP32 PCNT hardware registers for the specific unit.
    // This allows reading from any unit, bypassing the Zephyr sensor driver's
    // limitation which often defaults to unit 0.
    return (int32_t)(int16_t)PCNT.cnt_unit[unit_idx_].pulse_cnt;
}

void Encoders::reset() {
    // Reset the pulse counter for this unit using ESP32 registers.
    // Pulse counter reset bit for unit U is at bit (U * 2) in PCNT_CTRL_REG.
    PCNT.ctrl.val |= (1 << (unit_idx_ * 2)); // Set reset bit
    k_busy_wait(10);                         // Short wait
    PCNT.ctrl.val &= ~(1 << (unit_idx_ * 2)); // Clear reset bit
}
