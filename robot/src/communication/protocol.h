//
// Created by luca on 26/01/25.
//

/**
 * PROTOCOL:
 * ESP32 -> PC
 * MILLIS() - SENSOR ID - SIZE - DATA
 *
 * PACKET ID:
 *   - LIDAR
 *   - IMU
 *   - ENCODER MOTOR
 *   - BATTERY
 *
 * PC -> ESP32
 * MILLIS() ? - ACTION TYPE - SIZE - DATA
 *
 * ACTION TYPE:
 * - MOTOR
 * - LIDAR
 * - STOP ALL
 */
/**
 * bool: 1
 * char: 1
 * short: 2
 * int: 4
 * long: 4
 * unsigned long: 4
 * long long: 8
 * float: 4
 * double: 8
 * long double: 16
 * void*: 4
 * unit8_t*: 4
 */
#pragma once
#include <WiFi.h>

#include <cstdint>

#include "sensors/sensor.h"

#define RX_BUFFER_SIZE 128
#define RX_MAX_DATA    128
#define TX_BUFFER_SIZE 4096

/**
 * @struct HomeRobotPacket
 * @brief This struct is used to send data from the ESP32 to the PC
 * @var HomeRobotPacket::sequence_millis: when the packet was created, used to
 * sync the data between the ESP32 and the PC
 * @var HomeRobotPacket::type: the type of the packet, used to identify the data
 * @var HomeRobotPacket::size: the size of the data in bytes
 * @var HomeRobotPacket::data: the data itself
 */
struct HomeRobotHeader {
  unsigned long sequence_millis;
  PacketType type;
  uint16_t size;
} __attribute__((packed));

struct HomeRobotPacket {
  HomeRobotHeader header;
  uint8_t* data = new uint8_t[RX_MAX_DATA];
} __attribute__((packed));

class Protocol {
 private:
  WiFiClient server_connection;
  // Transmission stuff
  uint8_t* tx_buffer = new uint8_t[TX_BUFFER_SIZE];

  // Reception
  uint32_t rx_latest_millis = 0;
  size_t rx_used = 0;
  uint8_t* rx_buffer = new uint8_t[RX_BUFFER_SIZE];

  // 5 -> Lidar, 2 motors, imu, battery.
  Sensor* sensors[5] = {};
  uint8_t sensors_count = 0;
  bool read_header = false;

  /**
   * @brief Parse the rx_buffer to build the header.
   */
  void ParseHeader();

 public:
  HomeRobotPacket receive_packet;

  Protocol();
  // Utils
  void connect();
  void HardRestart();
  void SoftRestart();
  bool isConnected();
  void ShowStatus();

  // Send stuff
  size_t SendPacket(const HomeRobotPacket& packet);

  /**
   * @brief Should be called every cycle, read the wifi and allocate in a buffer.
   * Receive_packet will be good for only one cycle.
   * @return True if received a new packet from the server; false if no packet is ready.
   */
  bool ReceivePacket();

  // Sensors
  void AddSensor(Sensor* sensor);
  void SendSensors();
  void Loop();

  // Static functions
  static size_t HeaderSize();
  static int32_t GenerateHeader(
    uint8_t* buffer,
    size_t max_size,
    uint32_t millis,
    SendPacketType type,
    uint16_t size
  );
};
