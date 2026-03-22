#include "encoders.h"
#include <zephyr/logging/log.h>
#include <soc/pcnt_struct.h>

LOG_MODULE_REGISTER(encoders, LOG_LEVEL_INF);

// ESP32-C6 specific PCNT register offsets based on Technical Reference Manual
#define PCNT_CTRL_REG_OFF          0x60
#define PCNT_U0_CNT_REG_OFF        0x30 // Unit 0 count is at 0x30, Unit 1 at 0x34...
#define PCNT_BASE_ADDR             0x60012000

Encoders::Encoders(const struct device* pcnt_unit_dev, uint8_t unit_idx)
    : dev_(pcnt_unit_dev), unit_idx_(unit_idx) {
}

bool Encoders::init() {
    if (!device_is_ready(dev_)) {
        printk("ENCODER ERROR: Device %s not ready\n", dev_->name);
        return false;
    }

    // Direct Register Access for initialization (ESP32-C6)
    volatile uint32_t* pcnt_ctrl = (volatile uint32_t*)(PCNT_BASE_ADDR + PCNT_CTRL_REG_OFF);
    
    // Bit 16: Module Clock Enable
    *pcnt_ctrl |= (1 << 16); 
    
    // Bits [0, 2, 4, 6]: Reset for Units 0-3
    // Bits [1, 3, 5, 7]: Pause for Units 0-3
    *pcnt_ctrl &= ~(1 << (unit_idx_ * 2));     // Clear Reset
    *pcnt_ctrl &= ~(1 << (unit_idx_ * 2 + 1)); // Clear Pause

    reset();
    printk("ENCODER: %s (unit %d) initialized\n", dev_->name, unit_idx_);
    return true;
}

int32_t Encoders::get_ticks() {
    struct sensor_value val;
    int err = sensor_sample_fetch(dev_);
    if (err == 0) {
        err = sensor_channel_get(dev_, SENSOR_CHAN_ROTATION, &val);
        if (err == 0) {
            return (int32_t)val.val1;
        }
    }

    // Explicit Fallback for ESP32-C6
    volatile uint32_t* cnt_reg = (volatile uint32_t*)(PCNT_BASE_ADDR + PCNT_U0_CNT_REG_OFF + (unit_idx_ * 4));
    int32_t ticks = (int32_t)(int16_t)(*cnt_reg & 0xFFFF);
    
    static uint32_t last_print = 0;
    if (k_uptime_get_32() - last_print > 1000) {
        printk("ENCODER %d ticks: %d\n", unit_idx_, ticks);
        last_print = k_uptime_get_32();
    }
    
    return ticks;
}

void Encoders::reset() {
    volatile uint32_t* pcnt_ctrl = (volatile uint32_t*)(PCNT_BASE_ADDR + PCNT_CTRL_REG_OFF);
    
    // Toggle Reset bit
    *pcnt_ctrl |= (1 << (unit_idx_ * 2));
    k_busy_wait(10);
    *pcnt_ctrl &= ~(1 << (unit_idx_ * 2));
}
