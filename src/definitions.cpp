#include <Arduino.h>
#include "LDS_RPLIDAR_A1.h"
#include "definitions.h"
#include "protocol.h"

#include "secrets.h"

int pwm_lower_limit = 10;    //full range 0 - 255
int pwm_upper_limit = 63;  //full range

Encoder encoder_sx = Encoder(ENCODER_SX_A, ENCODER_SX_B);
Encoder  encoder_dx = Encoder(ENCODER_DX_A, ENCODER_DX_B);

Motor motor_sx = Motor(encoder_sx, MOTOR_SX_FORWARD, MOTOR_SX_BACKWARD, MOTOR_SX_PWM, pwm_lower_limit, pwm_upper_limit);
Motor motor_dx = Motor(encoder_dx, MOTOR_DX_FORWARD, MOTOR_DX_BACKWARD, MOTOR_DX_PWM, pwm_lower_limit, pwm_upper_limit);

// MPU6050_QMC5883L imu;
// MPU6050 imu;  // 0x68
// QMC5883L imu;  //
BMI160 imu;  // 0x68

calData calib = { 0 };  //Calibration data
AccelData IMUAccel;    //Sensor data
GyroData IMUGyro;
MagData IMUMag;
HardwareSerial LidarSerial(LDS_UART_NUM_0);  // TX 17, RX 16
LDS *lidar;
Protocol *protocol;

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

void init_server_connection() {
    protocol = new Protocol();
}

void init_i2c() {
  log_d("INIT I2C");

  Wire.begin(I2C_SDA, I2C_SCL);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(30000);
  log_i("Timeout");
#endif
}

void init_imu() {
  log_i("\t - INIT IMU");
  int err = 1;
  int old_error = 0;
  while (err != 0) {
    err = imu.init(calib, IMU_ADDR);
    if (err != 0) {
      if (err != old_error) {
        log_e("Error initializing IMU: %i", err);
        old_error = err;
      }
      delay(1000);
    }
  }
  log_i("\t OK");
}

void init_motors() {
  log_i("%d", encoder_dx.get_interrupts());

  motor_sx.init(KP, KI, KD);
  motor_dx.init(KP, KI, KD);
}