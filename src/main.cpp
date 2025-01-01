
#include "definitions.h"

int target = 360;  // target are the degrees of rotation

int state = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("###   INIT START   ###");

  init_i2c();
  init_imu();
  init_serial_lidar();
  init_lidar();
  init_motors();

  Serial.println("###   INIT DONE   ###");
}

int oldPosition = -9999;

void robot_run() {
  // motor_sx.set_target(target);
  motor_sx.start();
  int motor_pos = motor_sx.get_position();
  if (motor_pos != oldPosition) {
    oldPosition = motor_pos;
    Serial.println(motor_pos);
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
 IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ, IMUMag.magX, IMUMag.magY,
 IMUMag.magZ);
  } else {
    // filter.updateIMU(IMUGyro.gyroX, IMUGyro.gyroY, IMUGyro.gyroZ,
 IMUAccel.accelX, IMUAccel.accelY, IMUAccel.accelZ);
  }
}

void print_sensor() {
  Serial.print("\tMX: ");
  Serial.print(IMUMag.magX);
  Serial.print("\tMY: ");
  Serial.print(IMUMag.magY);
  Serial.print("\tMZ: ");
  Serial.println(IMUMag.magZ);

  //	Serial.print("QW: ");
  //	Serial.print(filter.getQuatW());
  //	Serial.print("\tQX: ");
  //	Serial.print(filter.getQuatX());
  //	Serial.print("\tQY: ");
  //	Serial.print(filter.getQuatY());
  //	Serial.print("\tQZ: ");
  //	Serial.println(filter.getQuatZ());
}

int oldRead = -120;

void loop() {
  // String cmd = Serial.readString();
  char cmd = Serial.read();

  switch (cmd) {
    case 's':
      // case 'stop':
      Serial.println("Stop all");
      motor_dx.turn_off();
      motor_sx.turn_off();
      state = 0;
      break;
    case 'g':
      // case 'start':
      Serial.println("Start all");
      motor_sx.set_position(0);
      motor_sx.set_target(target);
      state = 1;
      break;
    case 'r':
      // case 'sensor':
      Serial.println("Stop all");
      motor_dx.turn_off();
      motor_sx.turn_off();
      state = 2;
      break;
    default:
      break;
  }

  if (state == 0) {
    delay(100);
  } else if (state == 1) {
    robot_run();
  } else if (state == 2) {
    int read_enc = digitalRead(ENCODER_SX_A);
    if (oldRead != read_enc) {
      Serial.println(oldRead);
      oldRead = read_enc;
    }
    read_sensors();
  }
}

/// LIDAR PROGRAM

// Copyright 2023-2024 REMAKE.AI, KAIA.AI, MAKERSPET.COM

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

// http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ESP32
#error This example runs on ESP32
#endif

#include <LDS.h>
#include <LDS_RPLIDAR_A1.h>

const uint8_t LDS_MOTOR_PWM_PIN = 15;  // LiDAR motor speed control using PWM
#define LDS_MOTOR_PWM_FREQ 10000
#define LDS_MOTOR_PWM_BITS 11
#define LDS_MOTOR_PWM_CHANNEL \
  2  // ESP32 PWM channel for LiDAR motor speed control

HardwareSerial LidarSerial(UART_NUM_0);  // TX 17, RX 16
LDS_RPLIDAR_A1 lidar;

void setup() {
  // pinMode(15, OUTPUT);
  Serial.begin(115200);

  Serial.print("LiDAR model ");
  Serial.println(lidar.getModelName());

  Serial.print("LiDAR RX buffer size ");            // default 128 hw + 256 sw
  Serial.print(LidarSerial.setRxBufferSize(1024));  // must be before .begin()

  uint32_t baud_rate = lidar.getSerialBaudRate();
  Serial.print(", baud rate ");
  Serial.println(baud_rate);

  LidarSerial.begin(baud_rate, SERIAL_8N1);  // Use default GPIO TX 17, RX 16
  // Assign TX, RX pins
  // LidarSerial.begin(baud_rate, SERIAL_8N1, rxPin, txPin);
  // Details
  // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.h
  // Tutorial https://www.youtube.com/watch?v=eUPAoP7xC7A

  while (LidarSerial.read() >= 0);

  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setPacketCallback(lidar_packet_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.setMotorPinCallback(lidar_motor_pin_callback);
  lidar.init();

  LDS::result_t result = lidar.start();
  Serial.print("LiDAR start() result: ");
  Serial.println(lidar.resultCodeToString(result));

  if (result < 0) Serial.println("Is the LiDAR connected to ESP32?");
}

int lidar_serial_read_callback() {
  return LidarSerial.read();
  //  int ch = LidarSerial.read();
  //  if (ch != -1)
  //    Serial.println(ch);
  //  return ch;
}

size_t lidar_serial_write_callback(const uint8_t* buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

void lidar_scan_point_callback(float angle_deg, float distance_mm,
                               float quality, bool scan_completed) {
  static int i = 0;

  if ((i++ % 20 == 0) || scan_completed) {
    Serial.print(i);
    Serial.print(' ');
    Serial.print(distance_mm);
    Serial.print(' ');
    Serial.print(angle_deg);
    if (scan_completed)
      Serial.println('*');
    else
      Serial.println();
  }
}

void lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("LiDAR info ");
  Serial.print(lidar.infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("LiDAR error ");
  Serial.print(lidar.resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int pin = LDS_MOTOR_PWM_PIN;

  if (value <= -3) {  // LDS::DIR_INPUT) {
    // Configure pin direction
    if (value == LDS::DIR_OUTPUT_PWM) {
      // ledcSetup(LDS_MOTOR_PWM_CHANNEL, LDS_MOTOR_PWM_FREQ,
      // LDS_MOTOR_PWM_BITS); ledcAttachPin(pin, LDS_MOTOR_PWM_CHANNEL);
    } else
      pinMode(pin, (value == LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }

  if (value < LDS::VALUE_PWM)  // set constant output
    digitalWrite(pin, (value == LDS::VALUE_HIGH) ? HIGH : LOW);
  else {  // set PWM duty cycle
    int pwm_value = ((1 << LDS_MOTOR_PWM_BITS) - 1) * value;
    ledcWrite(LDS_MOTOR_PWM_CHANNEL, pwm_value);
  }
}

void lidar_packet_callback(uint8_t* packet, uint16_t length,
                           bool scan_completed) {
  return;
}

void loop() {
  // digitalWrite(15, HIGH);
  lidar.loop();
}
