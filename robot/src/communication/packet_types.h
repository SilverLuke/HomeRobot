#pragma once

#include <cstdint>

// Used from ESP32 -> PC
enum class SendPacketType : uint8_t {
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
