//
// Created by luca on 29/01/25.
//

#include "definitions.h"
#include "sensors.h"
#include "protocol.h"

HomeRobotPacket home_robot_packet{};


void lidar_info_callback(LDS::info_t code, String info) {
  Serial.print("LiDAR info ");
  Serial.print(lidar->infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.print("LiDAR error ");
  Serial.print(lidar->resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}


int lidar_serial_read_callback() {
  return LidarSerial.read();
}

size_t lidar_serial_write_callback(const uint8_t* buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

void lidar_scan_point_callback(
    float angle_deg,
    float distance_mm,
    float quality,
    bool scan_completed
) {
  static float total = 0.;
  static float old = 0.;
  static int mesures = 0;
  static unsigned long time = millis();

  float diff = angle_deg - old;
  total += diff;
  old = angle_deg;
  mesures++;
  if (scan_completed) {
    Serial.println("SPC TDeg: " + String(total) + " AVGDEG: " + String(total / (mesures)) + " mesures:" + String(mesures) + " " + String(millis() - time));
    mesures = 0;
    total = 0.;
    old = 0.;
    time = millis();
  }
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int pin = LDS_MOTOR_PWM_PIN;
  if (value <= -3) {  // LDS::DIR_INPUT) {
    // Configure pin direction
    if (value == LDS::DIR_OUTPUT_PWM) {
      // ledcSetup(LDS_MOTOR_PWM_CHANNEL, LDS_MOTOR_PWM_FREQ,
      // LDS_MOTOR_PWM_BITS); ledcAttachPin(pin, LDS_MOTOR_PWM_CHANNEL);
    }
    else {
      pinMode(pin, (value == LDS::DIR_INPUT) ? INPUT : OUTPUT);
    }
    return;
  }

  if (value < LDS::VALUE_PWM)  // set constant output
    digitalWrite(pin, (value == LDS::VALUE_HIGH) ? HIGH : LOW);
  else {  // set PWM duty cycle
    int pwm_value = ((1 << LDS_MOTOR_PWM_BITS) - 1) * value;
    ledcWrite(LDS_MOTOR_PWM_CHANNEL, pwm_value);
  }
}

/**
 * This is the base function packet is a pointer to a struct
 * @param packet
 * @param length
 * @param scan_completed
 */
void lidar_packet_callback(uint8_t* packet, uint16_t length, bool scan_completed) {
  static int sequence = 0;
  sequence++;

  home_robot_packet.millis = millis();
  home_robot_packet.type = SENSOR_LIDAR;
  home_robot_packet.size = length;
  home_robot_packet.data = packet;

  protocol->writePacket(home_robot_packet);

  if (scan_completed) {
    Serial.println("PC sended: " + String(sequence));
    sequence = 0;
  }
}

void init_lidar() {
  lidar = new LDS_RPLIDAR_A1();
  // lidar->setScanPointCallback(lidar_scan_point_callback);
  lidar->setPacketCallback(lidar_packet_callback);
  lidar->setSerialWriteCallback(lidar_serial_write_callback);
  lidar->setSerialReadCallback(lidar_serial_read_callback);
  lidar->setMotorPinCallback(lidar_motor_pin_callback);

  pinMode(LDS_MOTOR_PWM_PIN, OUTPUT);
  digitalWrite(LDS_MOTOR_PWM_PIN, LOW);

  log_i("Init LidarSerial");
  Serial.print("LiDAR model ");
  Serial.println(lidar->getModelName());
  Serial.print("LiDAR RX buffer size ");            // default 128 hw + 256 sw
  Serial.print(LidarSerial.setRxBufferSize(1024));  // must be before .begin()
  uint32_t baud_rate = lidar->getSerialBaudRate();
  Serial.print(", baud rate ");
  Serial.println(baud_rate);

  LidarSerial.begin(baud_rate, SERIAL_8N1, 7, 6);  // Use default GPIO TX 17, RX 16
  // Assign TX, RX pins
  // LidarSerial.begin(baud_rate, SERIAL_8N1, rxPin, txPin);
  // Details https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.h
  // Tutorial https://www.youtube.com/watch?v=eUPAoP7xC7A
  while (LidarSerial.read() >= 0);

  lidar->init();
}

void lidar_start() {

  LDS::result_t result = LDS::ERROR_TIMEOUT;
  while (result != LDS::RESULT_OK) {
    result = lidar->start();
    String status = lidar->resultCodeToString(result);

    Serial.print("\rLiDAR start() result: ");
    Serial.print(status);
    Serial.print(" ");
    Serial.println(result);

    if (result != 0) {
      log_e("Is the LiDAR connected to ESP32?");
      led_blink("..", RGB_BRIGHTNESS, 0,0);
      delay(500);
    }
  }
  lidar->setScanTargetFreqHz(1.0f);
}
