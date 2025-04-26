#include "sensors/lidar.h"

#include "../utils/utils.h"
#include "LDS_RPLIDAR_A1.h"
#include "communication/protocol.h"
#include "definitions.h"

Lidar* Lidar::instance = nullptr;

#define LIDAR_POINT_SIZE  sizeof(node_info_t)

Lidar::Lidar() : LDS_RPLIDAR_A1() {
  instance = this;

  buffer = new cppQueue(LIDAR_POINT_SIZE, MAX_LIDAR_BUFFER_SIZE, FIFO, false);
  full_read_size = new cppQueue(sizeof(size_t), MAX_LIDAR_FULL_READINGS, FIFO, false);
  read_millis = new cppQueue(sizeof(uint64_t), MAX_LIDAR_FULL_READINGS, FIFO, false);

  uint64_t time = millis();
  read_millis->push(&time);


  lidarSerial = new HardwareSerial(LDS_UART_NUM_0);

  // lidar->setScanPointCallback(lidar_scan_point_callback);
  this->setPacketCallback(packetCallback);
  this->setSerialWriteCallback(serialWriteCallback);
  this->setSerialReadCallback(serialReadCallback);
  this->setMotorPinCallback(motorPinCallback);

  pinMode(LIDAR_MOTOR_PIN, OUTPUT);
  digitalWrite(LIDAR_MOTOR_PIN, LOW);

  log_i("Init LidarSerial");
  Serial.print("LiDAR model ");
  Serial.println(this->LDS_RPLIDAR_A1::getModelName());
  Serial.print("LiDAR RX buffer size ");
  Serial.print(lidarSerial->setRxBufferSize(1024));

  const uint32_t baud_rate = this->LDS_RPLIDAR_A1::getSerialBaudRate();
  Serial.print(", baud rate ");
  Serial.println(baud_rate);

  lidarSerial->begin(baud_rate, SERIAL_8N1, LIDAR_SERIAL_RX_PIN,
                     LIDAR_SERIAL_TX_PIN);
  // Assign TX, RX pins
  // LidarSerial.begin(baud_rate, SERIAL_8N1, rxPin, txPin);
  // Details
  // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/HardwareSerial.h
  // Tutorial https://www.youtube.com/watch?v=eUPAoP7xC7A
  while (lidarSerial->read() >= 0) {
  }

  this->LDS_RPLIDAR_A1::init();
}

Lidar::~Lidar() {
  instance = nullptr;
}

void Lidar::startReading() {
  LDS::result_t result = LDS::ERROR_TIMEOUT;
  while (result != LDS::RESULT_OK) {
    result = LDS_RPLIDAR_A1::start();
    String status = this->resultCodeToString(result);

    Serial.print("\rLiDAR start() result: ");
    Serial.print(status);
    Serial.print(" ");
    Serial.println(result);

    if (result != 0) {
      log_e("Is the LiDAR connected to ESP32?");
      led_blink("..", LED_RED);
      delay(500);
    }
  }
  this->setScanTargetFreqHz(1.0f);
}

// Static callback implementations
void Lidar::infoCallback(const LDS::info_t code, const String& info) {
  Serial.print("LiDAR info ");
  Serial.print(instance->infoCodeToString(code));
  Serial.print(": ");
  Serial.println(info);
}

void Lidar::errorCallback(const LDS::result_t code, const String& aux_info) {
  Serial.print("LiDAR error ");
  Serial.print(instance->resultCodeToString(code));
  Serial.print(": ");
  Serial.println(aux_info);
}

int Lidar::serialReadCallback() { return instance->lidarSerial->read(); }

size_t Lidar::serialWriteCallback(const uint8_t* buffer, const size_t length) {
  return instance->lidarSerial->write(buffer, length);
}

void Lidar::motorPinCallback(float value, LDS::lds_pin_t lidar_pin) {
  const auto cast_value = static_cast<int32_t>(value);
  if (cast_value <= -3) {  // LDS::DIR_INPUT) {
    // Configure pin direction
    if (cast_value == LDS::DIR_OUTPUT_PWM) {
      // ledcSetup(LDS_MOTOR_PWM_CHANNEL, LDS_MOTOR_PWM_FREQ,
      // LDS_MOTOR_PWM_BITS); ledcAttachPin(pin, LDS_MOTOR_PWM_CHANNEL);
    } else {
      pinMode(LIDAR_MOTOR_PIN, (value == LDS::DIR_INPUT) ? INPUT : OUTPUT);
    }
    return;
  }

  if (value < LDS::VALUE_PWM)
    digitalWrite(LIDAR_MOTOR_PIN, (value == LDS::VALUE_HIGH) ? HIGH : LOW);
  else {
    int pwm_value = ((1 << LDS_MOTOR_PWM_BITS) - 1) * cast_value;
    ledcWrite(LDS_MOTOR_PWM_CHANNEL, pwm_value);
  }
}

// For each point in the scan already processed data, not used.
void Lidar::scanPointCallback(float angle_deg, float distance_mm, float quality,
                              bool scan_completed) {
  static float total = 0.;
  static float old = 0.;
  static int mesures = 0;
  static unsigned long time = millis();

  float diff = angle_deg - old;
  total += diff;
  old = angle_deg;
  mesures++;
  if (scan_completed) {
    Serial.println("SPC TDeg: " + String(total) +
                   " AVGDEG: " + String(total / static_cast<float>(mesures)) +
                   " mesures:" + String(mesures) + " " +
                   String(millis() - time));
    mesures = 0;
    total = 0.;
    old = 0.;
    time = millis();
  }
}

/**
 * @brief For each sensor read, like lidar_scan_point_callback but raw
 * datastructure
 * @param[in] packet   Raw data from the LiDAR, see RPLIDAR A1 @ref node_info
 * @param[in] length   For RPLIDAR A1 should be 8 + 16 + 16 = 40 byte
 * @param[in] scan_completed
 */
void Lidar::packetCallback(uint8_t* packet, const uint16_t length,
                           const bool scan_completed) {
  static int total_size = 0;

  instance->buffer->push(packet);

  total_size += 1;

  if (scan_completed) {
    instance->full_read_size->push(&total_size);
    total_size = 0;

    // Put the millis for the next read
    uint32_t time = millis();
    instance->read_millis->push(&time);

    Serial.println("Scan end: total size " + String(total_size));
  }
}

uint32_t Lidar::getMillis() {
  uint32_t time;
  this->read_millis->peek(&time);
  return time;
}

uint16_t Lidar::getDataSize() {
  uint16_t data_size;
  this->full_read_size->peek(&data_size);
  return data_size * LIDAR_POINT_SIZE;
}

int32_t Lidar::serialize(uint8_t* buffer, size_t max_size) {
  uint16_t items;
  this->full_read_size->pop(&items);
  uint32_t time;
  this->read_millis->pop(&time);

  if (items * LIDAR_POINT_SIZE > max_size) {
    return -1;
  }

  for (uint8_t* i = buffer ; i < buffer + (items * LIDAR_POINT_SIZE); i += LIDAR_POINT_SIZE) {
    this->buffer->pop(i);
  }

  return items * LIDAR_POINT_SIZE;
}
