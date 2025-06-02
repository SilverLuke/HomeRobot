#include "definitions.h"

#include <Wire.h>

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
    Serial.println("Failed to configure static IP");
  }

  WiFi.begin(wifi_ssid, wifi_password);

  int retries = 5;
  while (WiFi.status() != WL_CONNECTED && retries-- > 0) {
    Serial.println("Trying to connect to WiFi...");
    delay(2000);
  }
  if (WiFi.status() != WL_CONNECTED) {
    // TODO this could lead to boot loops
    log_e("Unable to connect to WiFi restarting ESP");
    ESP.restart();
  }

  Serial.println("Connected to WiFi");
}

void restart_wifi() {
  log_e("WiFi restart");
  WiFi.disconnect();
  WiFi.reconnect();
  sleep(1);
  init_wifi();
}

void init_i2c() {
  log_d("INIT I2C");

  Wire.begin(I2C_SDA, I2C_SCL);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(30000);
  log_i("Timeout");
#endif
}
void init_server_connection() { protocol = new Protocol(); }

void init_motors() {
  log_i("INIT MOTORS - START");
  encoder_sx = new Encoder(ENCODER_SX_A, ENCODER_SX_B);
  encoder_dx = new Encoder(ENCODER_DX_A, ENCODER_DX_B);

  motor_sx =
      new Motor(encoder_sx, MOTOR_SX_FORWARD, MOTOR_SX_BACKWARD, MOTOR_SX_PWM,
                MOTOR_PWM_LOWER_LIMIT, MOTOR_PWM_UPPER_LIMIT);
  motor_dx =
      new Motor(encoder_dx, MOTOR_DX_FORWARD, MOTOR_DX_BACKWARD, MOTOR_DX_PWM,
                MOTOR_PWM_LOWER_LIMIT, MOTOR_PWM_UPPER_LIMIT);

  // log_i("SX: %d", encoder_dx->get_interrupts());// SX: 2
  // log_i("DX: %d", encoder_dx->get_interrupts());// DX: 2

  motor_sx->init(KP, KI, KD);
  motor_dx->init(KP, KI, KD);
  log_i("INIT MOTORS - END");
}