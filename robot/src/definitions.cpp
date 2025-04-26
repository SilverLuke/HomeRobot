#include "definitions.h"

#include "LDS_RPLIDAR_A1.h"
#include "communication/protocol.h"
#include "secrets.h"

Encoder encoder_sx = Encoder(ENCODER_SX_A, ENCODER_SX_B);
Encoder encoder_dx = Encoder(ENCODER_DX_A, ENCODER_DX_B);

Motor motor_sx =
    Motor(encoder_sx, MOTOR_SX_FORWARD, MOTOR_SX_BACKWARD, MOTOR_SX_PWM,
          MOTOR_PWM_LOWER_LIMIT, MOTOR_PWM_UPPER_LIMIT);
Motor motor_dx =
    Motor(encoder_dx, MOTOR_DX_FORWARD, MOTOR_DX_BACKWARD, MOTOR_DX_PWM,
          MOTOR_PWM_LOWER_LIMIT, MOTOR_PWM_UPPER_LIMIT);

Protocol *protocol;

// Fix build error TODO investigate real solution
extern "C" void lwip_hook_ip6_input() {}

void init_wifi() {
  // Set static IP
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure static IP");
  }

  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    delay(2000);
  }
  Serial.println("Connected to WiFi");
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
  log_i("SX: %d", encoder_dx.get_interrupts());
  log_i("DX: %d", encoder_dx.get_interrupts());

  motor_sx.init(KP, KI, KD);
  motor_dx.init(KP, KI, KD);
}