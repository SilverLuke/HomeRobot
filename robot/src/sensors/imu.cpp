#include "imu.h"
#include <zephyr/logging/log.h>
#include <string.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include "../bridge/gazebo_bridge.h"
#endif

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

Imu::Imu(const struct device* dev) : dev_(dev) {
    memset(accel_, 0, sizeof(accel_));
    memset(gyro_, 0, sizeof(gyro_));
}

bool Imu::init() {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    LOG_INF("IMU (Virtual) is ready.");
    return true;
#else
    if (dev_ == nullptr) {
        LOG_ERR("IMU device pointer is NULL! Check devicetree alias 'imu'");
        return false;
    }
    if (!device_is_ready(dev_)) {
        LOG_ERR("IMU device '%s' is NOT READY. Check I2C wiring and power.", dev_->name);
        return false;
    }
    LOG_INF("IMU device '%s' is ready.", dev_->name);
    return true;
#endif
}

bool Imu::update() {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    float v_accel[3] = {0.0f};
    float v_gyro[3] = {0.0f};
    GazeboBridge::get_virtual_imu(v_accel, v_gyro);
    for (int i = 0; i < 3; i++) {
        sensor_value_from_double(&accel_[i], v_accel[i]);
        sensor_value_from_double(&gyro_[i], v_gyro[i]);
    }
    return true;
#else
    if (dev_ == nullptr || !device_is_ready(dev_)) {
        return false;
    }
    int ret = sensor_sample_fetch(dev_);
    if (ret < 0) {
        LOG_ERR("IMU fetch failed: error %d. Check if sensor is connected to I2C.", ret);
        return false;
    }

    sensor_channel_get(dev_, SENSOR_CHAN_ACCEL_XYZ, accel_);
    sensor_channel_get(dev_, SENSOR_CHAN_GYRO_XYZ, gyro_);
    return true;
#endif
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
