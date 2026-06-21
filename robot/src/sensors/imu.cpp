#include "imu.h"
#include <zephyr/logging/log.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include "../bridge/gazebo_bridge.h"
#endif

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

Imu::Imu(const struct device* dev) : dev_(dev) {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    v_accel_[0] = v_accel_[1] = v_accel_[2] = 0.0f;
    v_gyro_[0] = v_gyro_[1] = v_gyro_[2] = 0.0f;
#endif
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
    GazeboBridge::get_virtual_imu(v_accel_, v_gyro_);
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
#if defined(CONFIG_BOARD_NATIVE_SIM)
    x = v_accel_[0];
    y = v_accel_[1];
    z = v_accel_[2];
#else
    x = (float)sensor_value_to_double(&accel_[0]);
    y = (float)sensor_value_to_double(&accel_[1]);
    z = (float)sensor_value_to_double(&accel_[2]);
#endif
}

void Imu::get_gyro(float& x, float& y, float& z) const {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    x = v_gyro_[0];
    y = v_gyro_[1];
    z = v_gyro_[2];
#else
    x = (float)sensor_value_to_double(&gyro_[0]);
    y = (float)sensor_value_to_double(&gyro_[1]);
    z = (float)sensor_value_to_double(&gyro_[2]);
#endif
}
