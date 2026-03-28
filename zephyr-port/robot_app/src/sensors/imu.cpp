#include "imu.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

Imu::Imu(const struct device* dev) : dev_(dev) {
}

bool Imu::init() {
    if (!device_is_ready(dev_)) {
        LOG_ERR( "IMU device not ready");
        return false;
    }
    return true;
}

bool Imu::update() {
    if (sensor_sample_fetch(dev_) < 0) {
        LOG_ERR( "Sensor sample update error");
        return false;
    }

    sensor_channel_get(dev_, SENSOR_CHAN_ACCEL_XYZ, accel_);
    sensor_channel_get(dev_, SENSOR_CHAN_GYRO_XYZ, gyro_);
    return true;
}

void Imu::get_accel(float& x, float& y, float& z) const {
    x = (float)sensor_value_to_double(&accel_[0]);
    y = (float)sensor_value_to_double(&accel_[1]);
    z = (float)sensor_value_to_double(&accel_[2]);
}

void Imu::get_gyro(float& x, float& y, float& z) const {
    x = (float)sensor_value_to_double(&gyro_[0]);
    y = (float)sensor_value_to_double(&gyro_[1]);
    z = (float)sensor_value_to_double(&gyro_[2]);
}
