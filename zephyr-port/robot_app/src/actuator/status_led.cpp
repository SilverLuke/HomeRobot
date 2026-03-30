#include "status_led.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(StatusLed, LOG_LEVEL_INF);

#define RMT_LED_NODE DT_ALIAS(status_led)

/* WS2812 timings assuming an 80MHz RMT clock (12.5ns per tick) */
#define T0H 32  // 400ns
#define T0L 68  // 850ns
#define T1H 64  // 800ns
#define T1L 36  // 450ns

StatusLed::StatusLed() : 
    rmt_dev_(nullptr), 
    current_status_(RobotStatus::NO_WIFI),
    last_toggle_ms_(0),
    blink_state_(false),
    r_(0), g_(0), b_(0) 
{}

bool StatusLed::init() {
    rmt_dev_ = DEVICE_DT_GET(RMT_LED_NODE);
    if (!device_is_ready(rmt_dev_)) {
        LOG_ERR( "RMT Device for LED not ready");
        return false;
    }
    LOG_INF( "Status LED: Initialized on RMT device %p", rmt_dev_);
    
    // Initial flash to confirm physical connectivity
    set_color(50, 50, 50); // White
    k_msleep(50);
    
    set_status(RobotStatus::NO_WIFI);
    return true;
}

void StatusLed::set_status(RobotStatus status) {
    current_status_ = status;
    update();
}

void StatusLed::set_color(uint8_t r, uint8_t g, uint8_t b) {
    r_ = r; g_ = g; b_ = b;
    // WS2812 expects GRB order
    uint32_t color = (g << 16) | (r << 8) | b;
    for (int i = 0; i < 24; i++) {
        int bit = (color >> (23 - i)) & 1;
        symbols_[i].level0 = 1;
        symbols_[i].level1 = 0;
        if (bit) {
            symbols_[i].duration0 = T1H;
            symbols_[i].duration1 = T1L;
        } else {
            symbols_[i].duration0 = T0H;
            symbols_[i].duration1 = T0L;
        }
    }
    write_led();
}

void StatusLed::write_led() {
    if (rmt_dev_) {
        int ret = rmt_tx_transmit(rmt_dev_, symbols_, 24, K_MSEC(100));
        if (ret < 0) {
            LOG_ERR( "Status LED: Transmit failed (%d)", ret);
        } else {
            LOG_DBG( "Status LED: Transmit success (R:%d G:%d B:%d)", r_, g_, b_);
        }
    }
}

void StatusLed::update() {
    uint32_t now = k_uptime_get_32();
    uint8_t target_r = 0, target_g = 0, target_b = 0;

    switch (current_status_) {
        case RobotStatus::NO_WIFI:
            target_r = 50; target_g = 0; target_b = 0; // Fixed Red
            break;
        case RobotStatus::WIFI_ONLY:
            if (now - last_toggle_ms_ >= 500) {
                blink_state_ = !blink_state_;
                last_toggle_ms_ = now;
            }
            if (blink_state_) {
                target_r = 50; target_g = 0; target_b = 0; // Red
            } else {
                target_r = 0; target_g = 0; target_b = 0;  // Off
            }
            break;
        case RobotStatus::CONNECTED:
            target_r = 0; target_g = 50; target_b = 0; // Fixed Green
            break;
    }

    if (target_r != r_ || target_g != g_ || target_b != b_) {
        set_color(target_r, target_g, target_b);
    }
}
