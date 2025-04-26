#include "imu.h"

#include "../utils/utils.h"
#include "Wire.h"
#include "definitions.h"

IMU::IMU() {
  log_i("\t - INIT IMU");
  int err = 1;
  int old_error = 0;
  while (err != 0) {
    err = imu.init(calib, IMU_ADDR);
    if (err != 0) {
      if (err != old_error) {
        log_e("Error initializing IMU: %i", err);
        old_error = err;
      }
      delay(1000);
    }
  }
  log_i("\t OK");
}

void IMU::read() {
  this->read_millis = millis();
  imu.update();
  imu.getAccel(&accelData);
  imu.getGyro(&gyroData);
  if (imu.hasMagnetometer()) {
    imu.getMag(&magData);
    // filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ,
    // IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX,
    // IMUMag.magY, IMUMag.magZ);
  } else {
    // filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ,
    // IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  }
}

int32_t IMU::serialize(uint8_t* buffer, const size_t buffer_size) {
  // Go to the first empty byte
  auto* data = reinterpret_cast<uint32_t*>(buffer);
  // Start serialize imu sensor
  data[0] = host2netFloat(accelData.accelX);
  data[1] = host2netFloat(accelData.accelY);
  data[2] = host2netFloat(accelData.accelZ);
  data[3] = host2netFloat(gyroData.gyroX);
  data[4] = host2netFloat(gyroData.gyroY);
  data[5] = host2netFloat(gyroData.gyroZ);

  if (imu.hasMagnetometer()) {
    data[6] = host2netFloat(magData.magX);
    data[7] = host2netFloat(magData.magY);
    data[8] = host2netFloat(magData.magZ);
  }
  return this->getDataSize();
}

void IMU::print() {
  Serial.printf("IMU State (last read: %lu ms):\n", read_millis);
  Serial.printf("Accelerometer (m/s²):\n");
  Serial.printf("\tX: %.2f\n\tY: %.2f\n\tZ: %.2f\n", accelData.accelX,
                accelData.accelY, accelData.accelZ);

  Serial.printf("Gyroscope (rad/s):\n");
  Serial.printf("\tX: %.2f\n\tY: %.2f\n\tZ: %.2f\n", gyroData.gyroX,
                gyroData.gyroY, gyroData.gyroZ);

  if (imu.hasMagnetometer()) {
    Serial.printf("Magnetometer (µT):\n");
    Serial.printf("\tX: %.2f\n\tY: %.2f\n\tZ: %.2f\n", magData.magX,
                  magData.magY, magData.magZ);
  }
}