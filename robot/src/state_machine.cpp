#include <cstdint>

#include "Arduino.h"
#include "Elog.h"
#include "actuator/motor.h"
#include "communication/protocol.h"
#include "sensors/imu.h"
#include "sensors/lidar.h"

enum ROBOT_STATE { IDLE = 0, STOP, LIDAR, WIFI_SOFT, WIFI_HARD, MOTOR_GO };

void print_esp_state() {
  Serial.println("\n=== Internal RAM Stats ===");
  Serial.printf("Total Heap Size: %u bytes\n", ESP.getHeapSize());
  Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("Lowest Free Heap: %u bytes\n", ESP.getMinFreeHeap());
  Serial.printf("Largest Allocatable Block: %u bytes\n", ESP.getMaxAllocHeap());

  // Memory - PSRAM (if available)
  Serial.println("\n=== PSRAM Stats ===");
  Serial.printf("Total PSRAM Size: %u bytes\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());
  Serial.printf("Lowest Free PSRAM: %u bytes\n", ESP.getMinFreePsram());
  Serial.printf("Largest Allocatable PSRAM Block: %u bytes\n",
                ESP.getMaxAllocPsram());

  // Chip Information
  Serial.println("\n=== Chip Information ===");
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("Number of Cores: %d\n", ESP.getChipCores());
  Serial.printf("CPU Frequency: %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Cycle Count: %u\n", ESP.getCycleCount());

  // Software Versions
  Serial.println("\n=== Software Versions ===");
  Serial.printf("SDK Version: %s\n", ESP.getSdkVersion());
  Serial.printf("Core Version: %s\n", ESP.getCoreVersion());

  // Flash Information
  Serial.println("\n=== Flash Information ===");
  Serial.printf("Flash Chip Size: %u bytes\n", ESP.getFlashChipSize());
  Serial.printf("Flash Chip Speed: %u Hz\n", ESP.getFlashChipSpeed());
  Serial.printf("Flash Chip Mode: %d\n", ESP.getFlashChipMode());

  // Sketch Information
  Serial.println("\n=== Sketch Information ===");
  Serial.printf("Sketch Size: %u bytes\n", ESP.getSketchSize());
  Serial.printf("Free Sketch Space: %u bytes\n", ESP.getFreeSketchSpace());
  Serial.printf("Sketch MD5: %s\n", ESP.getSketchMD5().c_str());

  // Hardware ID
  Serial.println("\n=== Hardware Information ===");
  const uint64_t macAddress12 = ESP.getEfuseMac();
  Serial.printf("MAC Address: %012llX\n", macAddress12);

  Serial.println("\n=== Variable Sizes ===");
  Serial.printf("bool: %d\n", sizeof(bool));
  Serial.printf("char: %d\n", sizeof(char));
  Serial.printf("short: %d\n", sizeof(short));
  Serial.printf("int: %d\n", sizeof(int));
  Serial.printf("long: %d\n", sizeof(long));
  Serial.printf("unsigned long: %d\n", sizeof(unsigned long));
  Serial.printf("long long: %d\n", sizeof(long long));
  Serial.printf("float: %d\n", sizeof(float));
  Serial.printf("double: %d\n", sizeof(double));
  Serial.printf("long double: %d\n", sizeof(long double));
  Serial.printf("void*: %d\n", sizeof(void*));
  Serial.printf("unit8_t*: %d\n", sizeof(uint8_t*));
  /*
  *bool: 1
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
      log_i("Serial cmd: Stop");
      state = STOP;
      break;
    case 'g':
      // case 'go'
      log_i("Serial cmd: Motor go");
      state = MOTOR_GO;
      break;
    case 'l':
      // START LIDAR
      log_i("Serial cmd: Lidar start");
      state = LIDAR;
      break;
    case 'w':
      log_i("Serial cmd: Reset wifi soft");
      state = WIFI_SOFT;
      break;
    case 'W':
      log_i("Serial cmd: Reset wifi hard");
      state = WIFI_HARD;
      break;
    default:
      break;
  }
  return state;
}

ROBOT_STATE wifi_commands(Protocol* protocol, Lidar* lidar, IMU* imu) {
  if (!protocol->ReceivePacket()) {
    return IDLE;
  }

  switch (protocol->receive_packet.header.type.receive) {
    case RX_MOTOR_MOVE:
      motor_move(protocol->receive_packet.data);
      break;
    case RX_MOTOR_CONFIG:
      motor_config(protocol->receive_packet.data);
      break;
    case RX_LIDAR_MOTOR:
      float hz;
      memcpy(&hz, protocol->receive_packet.data, sizeof(float));
      lidar->setScanTargetFreqHz(hz);
      break;
    case RX_STOP_ALL:
      motor_sx->turn_off();
      motor_dx->turn_off();
      lidar->stopReading();
      break;
    case RX_REQUEST:
      protocol->receive_packet.header.sequence_millis = millis();
      protocol->SendPacket(protocol->receive_packet);
      break;
    default:
      log_e("Unknown cmd");
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
    //     log_i("DX: %d", oldRead);
    //     oldRead = read_enc;
    //   }
    //   log_i("SX: %d", encoder_sx.read());
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
      log_i("Start LiDAR by serial command");
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
  motor_sx->loop();
  motor_dx->loop();
}