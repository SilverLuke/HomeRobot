#pragma once

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

class Encoders {
public:
    Encoders(const struct device* pcnt_dev, uint8_t unit_idx);

    bool init();
    
    /**
     * @brief Get the current raw tick count from the encoder.
     */
    int32_t get_ticks();

    /**
     * @brief Reset the encoder tick count to zero.
     */
    void reset();

private:
    const struct device* dev_;
    uint8_t unit_idx_;
};
