#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/rmt_tx.h>

enum class RobotStatus {
    NO_WIFI,        // Fixed Red
    WIFI_ONLY,      // Blinking Red
    CONNECTED       // Green
};

class StatusLed {
public:
    StatusLed();
    bool init();
    void set_status(RobotStatus status);
    void update(); // Should be called periodically or run in a thread

private:
    void set_color(uint8_t r, uint8_t g, uint8_t b);
    void write_led();

    const struct device* rmt_dev_;
    struct rmt_symbol symbols_[24];
    RobotStatus current_status_;
    uint32_t last_toggle_ms_;
    bool blink_state_;
    uint8_t r_, g_, b_;
};
