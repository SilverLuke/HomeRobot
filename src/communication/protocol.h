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

#pragma once
#include <WiFi.h>
#include <cstdint>

#define MAX_PACKET_SIZE 128

// Used from ESP32 -> PC
enum Sensors : uint8_t{
  SENSOR_LIDAR = 0,
  SENSOR_IMU = 1,
  SENSOR_ENCODER_MOTOR = 2,
  SENSOR_BATTERY = 3,
};

// Used from PC -> ESP32
enum ActionType : uint8_t {
  ACTION_MOTOR = 0,
  ACTION_LIDAR = 1,
  ACTION_STOP_ALL = 2,
};

union PacketType {
  Sensors sensor_id;
  ActionType action_type;
};

struct HomeRobotPacket {
  unsigned long millis;
  Sensors type;
  uint16_t size;
  uint8_t* data;
}__attribute__((packed));


class Protocol {
    private:
      uint8_t* wifi_buffer = new uint8_t[MAX_PACKET_SIZE];
      WiFiClient client;

    public:
        Protocol();
        size_t writePacket(const HomeRobotPacket& packet);
        void readPacket(HomeRobotPacket& packet);
        void restart();
};
