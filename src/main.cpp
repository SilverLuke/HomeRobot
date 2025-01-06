#include "definitions.h"

int target = 360;  // target are the degrees of rotation

int state = 0;
int lidar_state = 0;

int oldPosition = -9999;

int oldRead = -120;

void robot_run() {
  // motor_sx.set_target(target);
  motor_sx.start();
  int motor_pos = motor_sx.get_position();
  if (motor_pos != oldPosition) {
    oldPosition = motor_pos;
    log_i("%d", motor_pos);
  }

  delay(20);
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


void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(100);

  log_i("###   INIT START   ###");

  init_i2c();
  init_imu();
  init_lidar();
  // init_motors();
  log_i("###   INIT DONE   ###");
}

void loop() {
  // String cmd = Serial.readString();
  char cmd = Serial.read();

  switch (cmd) {
    case 's':
      // case 'stop':
      log_i("Stop all");
      motor_dx.turn_off();
      motor_sx.turn_off();
      state = 0;
      break;
    case 'g':
      // case 'start':
      log_i("Start all");
      motor_sx.set_position(0);
      motor_sx.set_target(target);
      state = 1;
      break;
    case 'r':
      // case 'sensor':
      log_i("Stop all");
      motor_dx.turn_off();
      motor_sx.turn_off();
      state = 2;
      break;
    case 'p':
      read_sensors();
      print_sensors();
      break;
    case 'l':
      digitalWrite(LDS_MOTOR_PWM_PIN, HIGH);
      lidar_state = 1;
      break;
    case 'k':
      digitalWrite(LDS_MOTOR_PWM_PIN, LOW);
      lidar_state = 0;
    default:
      break;
  }

  if (lidar_state != 0) {
    lidar->loop();
  }

  if (state == 0) {
    delay(100);
  } else if (state == 1) {
    robot_run();
  } else if (state == 2) {
    int read_enc = digitalRead(ENCODER_SX_A);
    if (oldRead != read_enc) {
      log_i("%d", oldRead);
      oldRead = read_enc;
    }
    read_sensors();
  }
}
