#pragma once

#include "../utils/utils.h"
#include "definitions.h"
#include "sensor.h"

/*
I2C Sensors
// BMI160    ACC + GYRO   On address: 0x69
// HMC5883L  MAGNETOMETER On address: 0x0D
// BMI160_HMC5883L
*/

#define IMU_ADDR 0x68

class IMU : public Sensor {
 private:
  BMI160 imu;
  size_t read_millis = 0;
  calData calib{};
  AccelData accelData{};
  GyroData gyroData{};
  MagData magData{};

 public:
  IMU();
  ~IMU() override = default;
  void startReading() override {};
  void read() override;

  uint32_t getMillis() override { return read_millis; }
  SendPacketType getPacketType() override { return SendPacketType::TX_IMU; }
  uint16_t getDataSize() override {
    return imu.hasMagnetometer() ? 6 * sizeof(float) : 9 * sizeof(float);
  }

  int32_t serialize(uint8_t* buffer, size_t buffer_size) override;

  void print();
  void getAccel(AccelData* data) const { *data = accelData; }
  void getGyro(GyroData* data) const { *data = gyroData; }
  void getMag(MagData* data) const { *data = magData; }
};
