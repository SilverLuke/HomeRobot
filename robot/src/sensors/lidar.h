#pragma once

#include "HardwareSerial.h"
#include "LDS.h"
#include "LDS_RPLIDAR_A1.h"
#include "cppQueue.h"
#include "sensor.h"
/*
LIDAR
*/
// LiDAR motor speed control using PWM
#define LDS_MOTOR_PWM_PIN 15
#define LDS_MOTOR_PWM_FREQ 10000
#define LDS_MOTOR_PWM_BITS 11
#define LDS_MOTOR_PWM_CHANNEL \
  2  // ESP32 PWM channel for LiDAR motor speed control
#define LDS_UART_NUM_0 1

#define LIDAR_SERIAL_RX_PIN 7
#define LIDAR_SERIAL_TX_PIN 6
#define LIDAR_MOTOR_PIN 15

#define MAX_LIDAR_BUFFER_SIZE 900
#define MAX_LIDAR_FULL_READINGS (5+1)   // A full read has 180 points in it. So 900 keep 5 full reads

struct ArrayPoint {
  uint32_t read_millis;
  uint16_t start;
  uint16_t end;
};

class Lidar : public Sensor, public LDS_RPLIDAR_A1 {
public:
  Lidar();
  ~Lidar() override;

  String name() override { return "Lidar"; }
  void read() override { this->loop(); }
  void startReading() override;
  void stopReading() override;
  int32_t serialize(uint8_t* buffer, size_t max_size) override;
  uint32_t getMillis() override;
  SendPacketType getPacketType() override { return SendPacketType::TX_LIDAR; };
  uint16_t getDataSize() override;

  using LDS_RPLIDAR_A1::getCurrentScanFreqHz;
  using LDS_RPLIDAR_A1::getModelName;
  using LDS_RPLIDAR_A1::getSamplingRateHz;
  using LDS_RPLIDAR_A1::getSerialBaudRate;
  using LDS_RPLIDAR_A1::isActive;
  using LDS_RPLIDAR_A1::setScanTargetFreqHz;

private:
  // Callback methods
  static void infoCallback(LDS::info_t code, const String& info);
  static void errorCallback(LDS::result_t code, const String& aux_info);
  static int serialReadCallback();
  static size_t serialWriteCallback(const uint8_t* buffer, size_t length);
  static void scanPointCallback(float angle_deg, float distance_mm,
                                float quality, bool scan_completed);
  static void motorPinCallback(float value, LDS::lds_pin_t lidar_pin);
  static void packetCallback(uint8_t* packet, uint16_t length,
                             bool scan_completed);

  // Member variables
  HardwareSerial* lidarSerial;

  // For callback handling
  static Lidar* instance;

  /**
   * @brief Lidar circular buffer used to store the data from the LiDAR.
   */

  cppQueue* buffer;
  cppQueue* full_read_size;
  cppQueue* read_millis;

};