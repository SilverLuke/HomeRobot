#include <ArduinoOTA.h>
// #include <ESPmDNS.h>
// #include <NetworkUdp.h>
// #include <WiFi.h>
//
#include "definitions.h"
#include "sensors/battery.h"
#include "sensors/imu.h"
#include "sensors/lidar.h"
#include "state_machine.h"
#include "utils/utils.h"

#define MAX_SERIAL_RETRY 5
#define SEND_BATTERY_STATUS_SERIAL 10000

Lidar* lidar;
IMU* imu;

void show_status() {
  // Do it only once every 10 seconds
  static unsigned long lastStatusRead = 0;
  if (millis() - lastStatusRead >= SEND_BATTERY_STATUS_SERIAL) {
    lastStatusRead = millis();
    serial_show_battery();
    protocol->ShowStatus();
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

  // Port defaults to 3232
  ArduinoOTA.setPort(54321);
  ArduinoOTA.setTimeout(10);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("robot.luca.home.arpa");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
          type = "sketch";
        } else {  // U_SPIFFS
          type = "filesystem";
        }

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
          Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
          Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
          Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
          Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
          Serial.println("End Failed");
        }
      });

  ArduinoOTA.begin();

  init_server_connection();
  // init_motors();
  init_i2c();
  lidar = new Lidar();
  imu = new IMU();

  protocol->AddSensor(lidar);
  protocol->AddSensor(imu);

  log_i("###   INIT DONE   ###");
  led_blink("--", LED_GREEN);
  Serial.setDebugOutput(true);
}


ROBOT_STATE state = IDLE;
void loop() {
  state = IDLE;
  show_status();

  if (Serial.available()) {
    state = serial_commands();
  }

  if (protocol->isConnected()) {
    wifi_commands(protocol, lidar, imu);
  }


  apply_state(state, lidar, imu);
  motor_sx.loop();
  motor_dx.loop();

  protocol->Loop();

  ArduinoOTA.handle();
}
