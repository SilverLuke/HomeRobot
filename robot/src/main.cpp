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

#include <Elog.h>

#include "secrets-example.h"
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
  // Serial setup
  uint8_t retry = 0;
  Serial.begin(115200);
  while (!Serial || retry < MAX_SERIAL_RETRY) {
    retry++;
    delay(1000);
    Serial.begin(115200);
  }
  if (!Serial) {
    led_blink(".", LED_CYAN);
  }

  // Logger setup
  Logger.configureSyslog(wifi_server_host , 514, "esp32"); // syslog host, port and name of the esp32 device host name
  Logger.registerSerial(MAIN_LOGGER, ELOG_LEVEL_DEBUG, "Main");
  Logger.registerSyslog(MAIN_LOGGER, ELOG_LEVEL_DEBUG, ELOG_FAC_LOCAL0, "Main");
  Logger.registerSerial(PROTO_LOGGER, ELOG_LEVEL_DEBUG, "Protocol");
  Logger.registerSyslog(PROTO_LOGGER, ELOG_LEVEL_DEBUG, ELOG_FAC_LOCAL0, "Protocol");
  Logger.registerSerial(MOTOR_LOGGER, ELOG_LEVEL_DEBUG, "Motor");
  Logger.registerSyslog(MOTOR_LOGGER, ELOG_LEVEL_DEBUG, ELOG_FAC_LOCAL0, "Motor");
  Logger.registerSerial(LIDAR_LOGGER, ELOG_LEVEL_DEBUG, "Lidar");
  Logger.registerSyslog(LIDAR_LOGGER, ELOG_LEVEL_DEBUG, ELOG_FAC_LOCAL0, "Lidar");
  Logger.registerSerial(IMU_LOGGER, ELOG_LEVEL_DEBUG, "IMU");
  Logger.registerSyslog(IMU_LOGGER, ELOG_LEVEL_DEBUG, ELOG_FAC_LOCAL0, "IMU");

  while (init_battery() != 0 || battery_level() < 5) {
    Logger.warning(
        MAIN_LOGGER,
        "Battery not connected or battery level is low. Waiting 5 seconds");
    serial_show_battery();
    led_blink("-", LED_PURPLE);
    delay(5000);
  }

  init_wifi();

  // Port defaults to 3232
  ArduinoOTA.setPort(54321);
  ArduinoOTA.setTimeout(10000);

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
        Logger.info(MAIN_LOGGER, "Start updating %s", type.c_str());
      })
      .onEnd([]() { Logger.info(MAIN_LOGGER, "End"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Logger.debug(MAIN_LOGGER, "Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Logger.error(MAIN_LOGGER, "Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
          Logger.error(MAIN_LOGGER, "Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
          Logger.error(MAIN_LOGGER, "Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
          Logger.error(MAIN_LOGGER, "Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
          Logger.error(MAIN_LOGGER, "Receive Failed");
        } else if (error == OTA_END_ERROR) {
          Logger.error(MAIN_LOGGER, "End Failed");
        }
      });

  ArduinoOTA.begin();

  init_server_connection();
  init_motors();
  init_i2c();
  lidar = new Lidar();
  imu = new IMU();

  //protocol->AddSensor(lidar);
  //protocol->AddSensor(imu);

  led_blink("--", LED_GREEN);
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

  protocol->Loop();

  ArduinoOTA.handle();
  delay(100);
}