#include "definitions.h"
#include "imu.h"

void init_i2c() {
  log_d("INIT I2C");

  Wire.begin(I2C_SDA, I2C_SCL);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(30000);
  log_i("Timeout");
#endif
}

void init_imu() {
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

void read_sensors() {
  imu.update();
  imu.getAccel(&IMUAccel);
  imu.getGyro(&IMUGyro);
  if (imu.hasMagnetometer()) {
    imu.getMag(&IMUMag);
    // filter.update(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ,
    // IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX,
    // IMUMag.magY, IMUMag.magZ);
  } else {
    // filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ,
    // IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  }
}


void print_sensors() {
  Serial.print("IMU Accel: ");
  Serial.print(IMUAccel.accelX);
  Serial.print(", ");
  Serial.print(IMUAccel.accelY);
  Serial.print(", ");
  Serial.println(IMUAccel.accelZ);

  Serial.print("IMU Gyro: ");
  Serial.print(IMUGyro.gyroX);
  Serial.print(", ");
  Serial.print(IMUGyro.gyroY);
  Serial.print(", ");
  Serial.println(IMUGyro.gyroZ);
  // Serial.print("\tMX: ");
  // Serial.print(IMUMag.magX);
  // Serial.print("\tMY: ");
  // Serial.print(IMUMag.magY);
  // Serial.print("\tMZ: ");
  // log_i(IMUMag.magZ);

  // Serial.print("QW: ");
  // Serial.print(filter.getQuatW());
  // Serial.print("\tQX: ");
  // Serial.print(filter.getQuatX());
  // Serial.print("\tQY: ");
  // Serial.print(filter.getQuatY());
  // Serial.print("\tQZ: ");
  // log_i(filter.getQuatZ());
}
