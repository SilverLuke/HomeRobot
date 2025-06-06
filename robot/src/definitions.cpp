#include "definitions.h"

#include <Elog.h>
#include <Wire.h>

#include "actuator/Motor_PID/Motor_PID.h"
#include "communication/protocol.h"
#include "secrets.h"

Encoder* encoder_sx;
Encoder* encoder_dx;
Motor* motor_sx;
Motor* motor_dx;

Protocol* protocol;

// Fix build error TODO investigate real solution
extern "C" void lwip_hook_ip6_input() {}

void init_wifi() {
  // Set static IP
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Logger.error(PROTO_LOGGER, "Failed to configure static IP");
  }

  WiFi.begin(wifi_ssid, wifi_password);

  int retries = 5;
  while (WiFi.status() != WL_CONNECTED && retries-- > 0) {
    Logger.info(PROTO_LOGGER, "Trying to connect to WiFi...");
    delay(2000);
  }
  if (WiFi.status() != WL_CONNECTED) {
    // TODO this could lead to boot loops
    Logger.error(PROTO_LOGGER, "Unable to connect to WiFi restarting ESP");
    ESP.restart();
  }

  Logger.info(PROTO_LOGGER, "Connected to WiFi");
}

void restart_wifi() {
  Logger.error(PROTO_LOGGER, "WiFi restart");
  WiFi.disconnect();
  WiFi.reconnect();
  sleep(1);
  init_wifi();
}

void init_i2c() {
  Logger.debug(MAIN_LOGGER, "INIT I2C");

  Wire.begin(I2C_SDA, I2C_SCL);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(30000);
  Logger.info(MAIN_LOGGER, "Timeout");
#endif
}
void init_server_connection() { protocol = new Protocol(); }

void init_motors() {
  Logger.info(MOTOR_LOGGER, "INIT MOTORS - START");
  encoder_sx = new Encoder(ENCODER_SX_A, ENCODER_SX_B);
  encoder_dx = new Encoder(ENCODER_DX_A, ENCODER_DX_B);

  motor_sx =
      new Motor("SX", encoder_sx, MOTOR_SX_FORWARD, MOTOR_SX_BACKWARD, MOTOR_SX_PWM,
                MOTOR_PWM_LOWER_LIMIT, MOTOR_PWM_UPPER_LIMIT);
  motor_dx =
      new Motor("DX", encoder_dx, MOTOR_DX_FORWARD, MOTOR_DX_BACKWARD, MOTOR_DX_PWM,
                MOTOR_PWM_LOWER_LIMIT, MOTOR_PWM_UPPER_LIMIT);

  // Logger.info(MAIN_LOGGER, "SX: %d", encoder_dx->get_interrupts());// SX: 2
  // Logger.info(MAIN_LOGGER, "DX: %d", encoder_dx->get_interrupts());// DX: 2

  motor_sx->init(KP, KI, KD);
  motor_dx->init(KP, KI, KD);
  Logger.info(MOTOR_LOGGER, "INIT MOTORS - END");
}