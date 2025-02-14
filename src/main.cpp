#include "definitions.h"
#include "utils.h"
#include "sensors/lidar.h"
#include "sensors/battery.h"
#include "sensors/imu.h"

int state = 0;
int lidar_state = 0;

int target = 360;  // target are the degrees of rotation
int oldPosition = -9999;
int oldRead = -120;


void robot_run() {
  // motor_sx.set_target(target);
  motor_sx.start();
  int motor_pos = motor_sx.get_position();
  if (motor_pos != oldPosition) {
    oldPosition = motor_pos;
    log_i("%d", motor_pos);
  }

  delay(20);
}


void setup() {
  neopixelWrite(RGB_BUILTIN, 0,0,0);
  led_blink(".", WHITE);

  Serial.begin(115200);
  while (!Serial);
  delay(100);

  while (init_battery() != 0) {
    log_e("Battery not connected. Waiting 5 seconds");
    led_blink("-", PURPLE);
    delay(5000);
  }

  log_i("###   INIT START   ###");
  //init_wifi();
  //init_i2c();
  //init_imu();
  //init_lidar();
  init_motors();
  log_i("###   INIT DONE   ###");
  led_blink("--", GREEN);
  Serial.println("Use 'w' to connect to WiFi");
  Serial.println("Use 'l' to start LiDAR");
  Serial.println("Use 's' to Emergency Stop");
}

unsigned long lastBatteryRead = 0;
void show_battery() {
   // Do it only once every 10 seconds
  if (millis() - lastBatteryRead >= 10000) {
    lastBatteryRead = millis();
    Serial.print("Battery level: ");
    Serial.print(battery_level());
    Serial.print("% raw: ");
    Serial.println(battery_raw());
  }
}


void loop() {
  show_battery();
  char cmd = Serial.read();
  switch (cmd) {
    case 's':
      // case 'stop':
      log_i("Stop all");
      motor_sx.turn_off();
      motor_dx.turn_off();
      // lidar->stop();
      state = 0;
      lidar_state = 0;
      log_i("Stop all end");
      // delay(10);
      break;
    case 'g':
      // case 'start':
      log_i("Start all");
      motor_sx.set_position(0);
      motor_sx.set_target(target);
      state = 1;
      break;
    case 'r':
      // case 'sensor':
      log_i("Stop all");
      motor_dx.turn_off();
      motor_sx.turn_off();
      state = 2;
      break;
    case 'p':
      read_sensors();
      print_sensors();
      break;
    case 'l':
      // START LIDAR
      if (lidar_state != 0) {
        break;
      }
      lidar_start();
      lidar_state = 1;
      break;
    case 'w':
      Serial.println("CONNECTING to WIFI");
      if (WiFi.status() != WL_CONNECTED) {
        init_wifi();
      }
      protocol->restart();
      break;
    default:
      break;
  }

  if (lidar_state != 0) {
    lidar->loop();
  }

  switch (state) {
    case 0:
      delay(1000);
      break;
    case 1:
      robot_run();
      break;
    case 2:
      int read_enc = digitalRead(ENCODER_SX_A);
      if (oldRead != read_enc) {
        log_i("%d", oldRead);
        oldRead = read_enc;
      }
      read_sensors();
      break;
  }
}
