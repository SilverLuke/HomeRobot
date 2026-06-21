#include "encoders.h"
#include "pcnt_reader.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include "../bridge/gazebo_bridge.h"
#endif

LOG_MODULE_REGISTER(encoders, LOG_LEVEL_INF);

Encoders::Encoders(const struct device* dev, uint8_t unit_idx)
    : dev_(dev), unit_idx_(unit_idx) {
}

bool Encoders::init() {
#if !defined(CONFIG_BOARD_NATIVE_SIM)
    if (dev_ == nullptr || !device_is_ready(dev_)) {
        LOG_ERR("PCNT device is NULL or not ready");
        return false;
    }
    
    // Manually start the specific unit
    pcnt_init_unit(unit_idx_);
#endif
    
    LOG_INF("Encoders initialized on PCNT device %p, Unit %d", dev_, unit_idx_);
    return true;
}

int32_t Encoders::get_total_ticks() {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    total_ticks_ = GazeboBridge::get_virtual_ticks(unit_idx_);
#else
    // Call the C wrapper to read the hardware register
    int16_t count = pcnt_get_unit_count(unit_idx_);
    total_ticks_ = (int32_t)count;
#endif
    return total_ticks_;
}

void Encoders::reset() {
    total_ticks_ = 0;
}
