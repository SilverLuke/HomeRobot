#include <cstdint>

#include "Arduino.h"
#include "Elog.h"
#include "actuator/motor.h"
#include "communication/protocol.h"
#include "sensors/imu.h"
#include "sensors/lidar.h"

enum ROBOT_STATE { IDLE = 0, STOP, LIDAR, WIFI_SOFT, WIFI_HARD, MOTOR_GO };

void print_esp_state() {
  Logger.info(MAIN_LOGGER, "\n=== Internal RAM Stats ===");
  Logger.info(MAIN_LOGGER, "Total Heap Size: %u bytes", ESP.getHeapSize());
  Logger.info(MAIN_LOGGER, "Free Heap: %u bytes", ESP.getFreeHeap());
  Logger.info(MAIN_LOGGER, "Lowest Free Heap: %u bytes", ESP.getMinFreeHeap());
  Logger.info(MAIN_LOGGER, "Largest Allocatable Block: %u bytes", ESP.getMaxAllocHeap());

  // Memory - PSRAM (if available)
  Logger.info(MAIN_LOGGER, "\n=== PSRAM Stats ===");
  Logger.info(MAIN_LOGGER, "Total PSRAM Size: %u bytes", ESP.getPsramSize());
  Logger.info(MAIN_LOGGER, "Free PSRAM: %u bytes", ESP.getFreePsram());
  Logger.info(MAIN_LOGGER, "Lowest Free PSRAM: %u bytes", ESP.getMinFreePsram());
  Logger.info(MAIN_LOGGER, "Largest Allocatable PSRAM Block: %u bytes", ESP.getMaxAllocPsram());

  // Chip Information
  Logger.info(MAIN_LOGGER, "\n=== Chip Information ===");
  Logger.info(MAIN_LOGGER, "Chip Model: %s", ESP.getChipModel());
  Logger.info(MAIN_LOGGER, "Chip Revision: %d", ESP.getChipRevision());
  Logger.info(MAIN_LOGGER, "Number of Cores: %d", ESP.getChipCores());
  Logger.info(MAIN_LOGGER, "CPU Frequency: %u MHz", ESP.getCpuFreqMHz());
  Logger.info(MAIN_LOGGER, "Cycle Count: %u", ESP.getCycleCount());

  // Software Versions
  Logger.info(MAIN_LOGGER, "\n=== Software Versions ===");
  Logger.info(MAIN_LOGGER, "SDK Version: %s", ESP.getSdkVersion());
  Logger.info(MAIN_LOGGER, "Core Version: %s", ESP.getCoreVersion());

  // Flash Information
  Logger.info(MAIN_LOGGER, "\n=== Flash Information ===");
  Logger.info(MAIN_LOGGER, "Flash Chip Size: %u bytes", ESP.getFlashChipSize());
  Logger.info(MAIN_LOGGER, "Flash Chip Speed: %u Hz", ESP.getFlashChipSpeed());
  Logger.info(MAIN_LOGGER, "Flash Chip Mode: %d", ESP.getFlashChipMode());

  // Sketch Information
  Logger.info(MAIN_LOGGER, "\n=== Sketch Information ===");
  Logger.info(MAIN_LOGGER, "Sketch Size: %u bytes", ESP.getSketchSize());
  Logger.info(MAIN_LOGGER, "Free Sketch Space: %u bytes", ESP.getFreeSketchSpace());
  Logger.info(MAIN_LOGGER, "Sketch MD5: %s", ESP.getSketchMD5().c_str());

  // Hardware ID
  Logger.info(MAIN_LOGGER, "\n=== Hardware Information ===");
  const uint64_t macAddress12 = ESP.getEfuseMac();
  Logger.info(MAIN_LOGGER, "MAC Address: %012llX", macAddress12);

  Logger.info(MAIN_LOGGER, "\n=== Variable Sizes ===");
  Logger.info(MAIN_LOGGER, "bool: %d", sizeof(bool));
  Logger.info(MAIN_LOGGER, "char: %d", sizeof(char));
  Logger.info(MAIN_LOGGER, "short: %d", sizeof(short));
  Logger.info(MAIN_LOGGER, "int: %d", sizeof(int));
  Logger.info(MAIN_LOGGER, "long: %d", sizeof(long));
  Logger.info(MAIN_LOGGER, "unsigned long: %d", sizeof(unsigned long));
  Logger.info(MAIN_LOGGER, "long long: %d", sizeof(long long));
  Logger.info(MAIN_LOGGER, "float: %d", sizeof(float));
  Logger.info(MAIN_LOGGER, "double: %d", sizeof(double));
  Logger.info(MAIN_LOGGER, "long double: %d", sizeof(long double));
  Logger.info(MAIN_LOGGER, "void*: %d", sizeof(void*));
  Logger.info(MAIN_LOGGER, "unit8_t*: %d", sizeof(uint8_t*));
  /*
  bool: 1
  char: 1
  short: 2
  int: 4
  long: 4
  unsigned long: 4
  long long: 8
  float: 4
  double: 8
  long double: 16
  void*: 4
  unit8_t*: 4
  */
}
ROBOT_STATE serial_commands() {
  ROBOT_STATE state = IDLE;
  char cmd = Serial.read();
  switch (cmd) {
    case 'r':
      // Memory - Internal RAM
      print_esp_state();
      delay(100);
      ESP.restart();
      state = IDLE;
      break;
    case 's':
      // case 'stop'
      Logger.info(MAIN_LOGGER, "Serial cmd: Stop");
      state = STOP;
      break;
    case 'g':
      // case 'go'
      Logger.info(MAIN_LOGGER, "Serial cmd: Motor go");
      state = MOTOR_GO;
      break;
    case 'l':
      // START LIDAR
      Logger.info(MAIN_LOGGER, "Serial cmd: Lidar start");
      state = LIDAR;
      break;
    case 'w':
      Logger.info(MAIN_LOGGER, "Serial cmd: Reset wifi soft");
      state = WIFI_SOFT;
      break;
    case 'W':
      Logger.info(MAIN_LOGGER, "Serial cmd: Reset wifi hard");
      state = WIFI_HARD;
      break;
    default:
      break;
  }
  return state;
}

ROBOT_STATE wifi_commands(Protocol* protocol, Lidar* lidar, IMU* imu) {
  Serial.println("Waiting for commands");
  if (!protocol->ReceivePacket()) {
    return IDLE;
  }

  switch (protocol->receive_packet.header.type.receive) {
    case RX_MOTOR_MOVE:
      Logger.info(MAIN_LOGGER, "Received motor move");
      motor_move(protocol->receive_packet.data);
      break;
    case RX_MOTOR_CONFIG:
      Logger.info(MAIN_LOGGER, "Received motor config");
      motor_config(protocol->receive_packet.data);
      break;
    case RX_LIDAR_MOTOR:
      Logger.info(MAIN_LOGGER, "Received Lidar start");
      float hz;
      memcpy(&hz, protocol->receive_packet.data, sizeof(float));
      lidar->setScanTargetFreqHz(hz);
      break;
    case RX_STOP_ALL:
      Logger.info(MAIN_LOGGER, "Received stop all");
      motor_sx->turn_off();
      motor_dx->turn_off();
      lidar->stopReading();
      break;
    case RX_REQUEST:
      Logger.info(MAIN_LOGGER, "Received request");
      protocol->receive_packet.header.sequence_millis = millis();
      protocol->SendPacket(protocol->receive_packet);
      break;
    default:
      Logger.error(MAIN_LOGGER, "Unknown cmd");
      break;
  }
  return IDLE;
}

void apply_state(ROBOT_STATE state, Lidar* lidar, IMU* imu) {
  switch (state) {
    case STOP:
      motor_sx->turn_off();
      motor_dx->turn_off();
      // lidar->stopReading();
      break;
    // case 2:
    //   imu->print();
    //
    //   int32_t read_enc = encoder_sx.read();
    //   if (oldRead != read_enc) {
    //     Logger.info(MAIN_LOGGER, "DX: %d", oldRead);
    //     oldRead = read_enc;
    //   }
    //   Logger.info(MAIN_LOGGER, "SX: %d", encoder_sx.read());
    //   set_refresh(FAST_REFRESH_HZ);
    //   break;
    case MOTOR_GO:
      motor_sx->set_position(0);
      motor_dx->set_position(0);
      motor_sx->set_target(360);
      motor_dx->set_target(360);
      motor_sx->turn_on();
      motor_dx->turn_on();
      break;
    case LIDAR:
      // START LIDAR
      Logger.info(MAIN_LOGGER, "Start LiDAR by serial command");
      if (!lidar->isActive()) {
        lidar->startReading();
      }
      break;

    case WIFI_HARD:
      protocol->ShowStatus();
      protocol->HardRestart();
      break;

    case WIFI_SOFT:
      protocol->ShowStatus();
      if (WiFi.status() != WL_CONNECTED) {
        init_wifi();
      }
      protocol->SoftRestart();
      break;
    default:
      // lidar->loop();
      break;
  }
  //motor_sx->loop();
  //motor_dx->loop();
}