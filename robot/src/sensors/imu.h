#pragma once

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

class Imu {
public:
    Imu(const struct device* dev);

    bool init();
    bool update();

    void get_accel(float& x, float& y, float& z) const;
    void get_gyro(float& x, float& y, float& z) const;

private:
    const struct device* dev_;
    struct sensor_value accel_[3];
    struct sensor_value gyro_[3];
};
