#include "definitions.h"
#include "sensors/battery.h"
#include "sensors/imu.h"
#include "sensors/lidar.h"
#include "utils/utils.h"
int state = 0;
int lidar_state = 0;

int target = 360;  // target are the degrees of rotation
int32_t oldPosition = -999;
int32_t oldRead = -120;

// 1/150 = 0.006 seconds -> 6 millis  150 is the number of points that the lidar
// can read
#define FAST_REFRESH_HZ 500
#define SLOW_REFRESH_HZ 1000

unsigned long old_millis = 0;
unsigned long targetRefresh = -1;

#define MAX_SERIAL_RETRY 5
#define SEND_BATTERY_STATUS_SERIAL 10000

Lidar* lidar;
IMU* imu;
Sensor* sensor[2];

void set_refresh(unsigned long refresh) {
  if (refresh < targetRefresh) {
    targetRefresh = refresh;
  }
}

void setup() {
  neopixelWrite(RGB_BUILTIN, LED_OFF);
  led_blink(".", LED_WHITE);

  uint8_t retry = 0;
  Serial.begin(115200);
  while (!Serial || retry < MAX_SERIAL_RETRY) {
    retry++;
    delay(1000);
  }

  if (!Serial) {
    led_blink(".", LED_CYAN);
  }

  while (init_battery() != 0 || battery_level() < 5) {
    log_e("Battery not connected or battery level is low. Waiting 5 seconds");
    serial_show_battery();
    led_blink("-", LED_PURPLE);
    delay(5000);
  }

  log_i("###   INIT START   ###");
  init_wifi();
  init_server_connection();
  // init_motors();
  init_i2c();
  lidar = new Lidar();
  imu = new IMU();

  log_i("###   INIT DONE   ###");
  led_blink("--", LED_GREEN);
  old_millis = millis();

  sensor[0] = lidar;
  sensor[1] = imu;
}

void show_battery() {
  // Do it only once every 10 seconds
  static unsigned long lastBatteryRead = 0;
  if (millis() - lastBatteryRead >= SEND_BATTERY_STATUS_SERIAL) {
    lastBatteryRead = millis();
    serial_show_battery();
  }
}

void serial_commands() {
  char cmd = Serial.read();
  switch (cmd) {
    case 's':
      // case 'stop'
      log_i("Stop all by serial command");
      state = 1;
      break;
    case 'g':
      // case 'go'
      log_i("Start motor sx by serial command");
      motor_sx.set_position(0);
      motor_sx.set_target(target);
      state = 2;
      break;
    case 'l':
      // START LIDAR
      log_i("Start LiDAR by serial command");
      if (lidar->isActive()) {
        break;
      }
      lidar->startReading();
      set_refresh(FAST_REFRESH_HZ);
      break;
    case 'w':
      Serial.println("CONNECTING to WIFI by serial command");
      if (WiFi.status() != WL_CONNECTED) {
        init_wifi();
      }
      protocol->restart();
      break;
    default:
      break;
  }

  switch (state) {
    case 1:
      motor_sx.turn_off();
      motor_dx.turn_off();
      lidar->stopReading();
      set_refresh(SLOW_REFRESH_HZ);
      break;
    case 2:
      imu->print();

      int32_t read_enc = encoder_sx.read();
      if (oldRead != read_enc) {
        log_i("DX: %d", oldRead);
        oldRead = read_enc;
      }
      log_i("SX: %d", encoder_sx.read());
      set_refresh(FAST_REFRESH_HZ);
      break;
  }
}

void motor_move(const uint8_t* data) {
  const int8_t dx_power = static_cast<int8_t>(data[0]);

  uint32_t raw_dx_angle;
  memcpy(&raw_dx_angle, &data[1], sizeof(uint32_t));
  float dx_angle = net2hostFloat(raw_dx_angle);

  const int8_t sx_power = static_cast<int8_t>(data[5]);

  uint32_t raw_sx_angle;
  memcpy(&raw_sx_angle, &data[6], sizeof(uint32_t));
  float sx_angle = net2hostFloat(raw_sx_angle);

  if (dx_power == 0) {
    motor_dx.turn_off();
  } else {
    motor_dx.set_target(100.0f * dx_power);
  }

  if (sx_power == 0) {
    motor_sx.turn_off();
  } else {
    motor_sx.set_target(100.0f * sx_power);
  }
  log_d("DX: %d, %f   SX: %d, %f", dx_power, dx_angle, sx_power, sx_angle);
}

uint8_t extract_motor_config(const uint8_t* data, float* kp, float* ki,
                             float* kd, uint8_t* upper_limit, uint8_t offset) {
  memcpy(kp, data, sizeof(float));
  offset += sizeof(float);
  memcpy(ki, data + offset, sizeof(float));
  offset += sizeof(float);
  memcpy(kd, data + offset, sizeof(float));
  offset += sizeof(float);
  memcpy(upper_limit, data + offset, sizeof(uint8_t));
  offset += sizeof(uint8_t);
  return offset;
}

void motor_config(const uint8_t* data) {
  float kp, ki, kd;
  uint8_t upper_limit;
  const uint8_t offset =
      extract_motor_config(data, &kp, &ki, &kd, &upper_limit, 0);
  motor_dx.set_config(kp, ki, kd);
  motor_dx.limit(50, upper_limit);

  extract_motor_config(data, &kp, &ki, &kd, &upper_limit, offset);
  motor_sx.set_config(kp, ki, kd);
  motor_sx.limit(50, upper_limit);
}

void wifi_commands() {
  if (!protocol->ReceivePacket()) {
    return;
  }

  switch (protocol->receive_packet.type.receive) {
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
      motor_sx.turn_off();
      motor_dx.turn_off();
      lidar->stopReading();
      break;
    case RX_REQUEST:
      protocol->receive_packet.sequence_millis = millis();
      protocol->SendPacket(protocol->receive_packet);
      break;
    default:
      log_e("Unknown cmd");
      break;
  }
}

void loop() {
  show_battery();

  if (Serial.available()) {
    serial_commands();
  }

  if (protocol->isConnected()) {
    wifi_commands();
    set_refresh(FAST_REFRESH_HZ);
  } else {
    set_refresh(SLOW_REFRESH_HZ);
  }

  lidar->loop();
  motor_sx.loop();
  motor_dx.loop();

  const unsigned long current_millis = millis();
  if (current_millis - old_millis < targetRefresh) {
    // Serial.println(current_millis);
    // Serial.println(old_millis);
    // Serial.println("DELAY " + String(targetRefresh - (current_millis -
    // old_millis)));
    delay(targetRefresh - (current_millis - old_millis));
  }
  old_millis = current_millis;
}
