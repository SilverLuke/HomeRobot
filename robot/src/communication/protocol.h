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
#include <cstdint>
#include <WiFi.h>

// Maximum size of the packet use 1500 because it's the maximum size of the
// packet MTU
#define MAX_LIDAR_BUFFER_SIZE 900
#define MAX_PACKET_SIZE 128

// Used from ESP32 -> PC
enum SendPacketType : uint8_t {
  /**
   * Maybe cache some points in the buffer and send a big packet with more
   * points in it.
   * LIDAR:
   * ANGLE (0, 360),
   * DISTANCE (0, 8000),
   * QUALITY (0, 255)
   */
  TX_LIDAR = 0,

  /**
   * IMU:
   * ACC_X, ACC_Y, ACC_Z,
   * GYRO_X, GYRO_Y, GYRO_Z,
   * MAG_X, MAG_Y, MAG_Z       // NOT USED
   */
  TX_IMU = 1,

  /**
   * BATTERY LEVEL:
   *    PERCENTAGE (0, 100),
   *    RAW (0, 4095)
   */
  TX_BATTERY = 2,

  /**
   * MOTOR DX ENCODER,
   * MOTOR SX ENCODER
   */
  TX_ENCODER_MOTORS = 4,

  /**
   * LIDAR SPEED,
   * MDX:
   *    KP,
   *    KI,
   *    KD,
   *    MAX_SPEED,
   * MSX:
   *    KP,
   *    KI,
   *    KD,
   *    MAX_SPEED
   */
  TX_CONFIG = 8,

  /**
   *
   */
  TX_ECHO = 16,
};

// Used from PC -> ESP32
enum ReceivePacketType : uint8_t {
  /**
   * DX:
   *  - int8_t POWER (-100, 100),
   *  - float  ANGLE (0, +inf)
   * SX:
   *  - int8_t POWER (-100, 100),
   *  - float  ANGLE (0, +inf)
   *
   * Send DX: 0,0 && SX: 0,0 to stop the robot
   */
  RX_MOTOR_MOVE = 0,

  /**
   * DX:
   * - float KP
   * - float KI
   * - float KD
   * - uint8_t MAX_SPEED
   * SX:
   * - float KP
   * - float KI
   * - float KD
   * - uint8_t MAX_SPEED
   *
   * Change the PID constants and the max speed of the motors
   */
  RX_MOTOR_CONFIG = 1,

  /**
   * Scan frequency of the LiDAR
   * Hz: (0, 256) -> then mapped to (0., 10.) float
   */
  RX_LIDAR_MOTOR = 2,

  /**
   * Stop all the motors and the LiDAR
   */
  RX_STOP_ALL = 4,

  /**
   * Request a TX packet, used for echo/ping
   */
  RX_REQUEST = 8,
};

union PacketType {
  SendPacketType send;
  ReceivePacketType receive;
};

/**
 * @struct HomeRobotPacket
 * @brief This struct is used to send data from the ESP32 to the PC
 * @var HomeRobotPacket::sequence_millis: when the packet was created, used to
 * sync the data between the ESP32 and the PC
 * @var HomeRobotPacket::type: the type of the packet, used to identify the data
 * @var HomeRobotPacket::size: the size of the data in bytes
 * @var HomeRobotPacket::data: the data itself
 */
struct HomeRobotPacket {
  unsigned long sequence_millis;
  PacketType type;
  uint16_t size;
  uint8_t* data;
} __attribute__((packed));

class Protocol {

  private:
    WiFiClient wifi_client;
    /**
     * @brief Lidar buffer used to store the data from the LiDAR.
     */
    unsigned long buffer_start_millis = 0;
    uint16_t lidar_buffer_end = Protocol::PrefixSize();
    uint8_t* lidar_buffer = new uint8_t[MAX_LIDAR_BUFFER_SIZE];

    /**
     * @brief Buffer used to store the non Lidar packets before sending it to the PC
     */
    uint8_t* packet_buffer = new uint8_t[MAX_PACKET_SIZE];
    uint8_t* receive_buffer = new uint8_t[MAX_PACKET_SIZE];

   public:
    HomeRobotPacket receive_packet{};

    Protocol();
    // Utils
    void connect();
    bool is_connected();
    void restart();
    static size_t PrefixSize();

    // Send stuff
    void AddLidarPacket(uint8_t* packet, uint16_t length);
    void SendLidarPacket();
    size_t SendPacket(const HomeRobotPacket& packet);

    // Receive stuff
    bool ReceivePacket();
};
