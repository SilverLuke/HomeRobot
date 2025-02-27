#include "LDS_RPLIDAR_A1.h"
#include "definitions.h"
#include "communication/protocol.h"

#include "secrets.h"

Encoder encoder_sx = Encoder(ENCODER_SX_A, ENCODER_SX_B);
Encoder encoder_dx = Encoder(ENCODER_DX_A, ENCODER_DX_B);

Motor motor_sx = Motor(
  encoder_sx,
  MOTOR_SX_FORWARD,
  MOTOR_SX_BACKWARD,
  MOTOR_SX_PWM,
  MOTOR_PWM_LOWER_LIMIT,
  MOTOR_PWM_UPPER_LIMIT);
Motor motor_dx = Motor(
  encoder_dx,
  MOTOR_DX_FORWARD,
  MOTOR_DX_BACKWARD,
  MOTOR_DX_PWM,
  MOTOR_PWM_LOWER_LIMIT,
  MOTOR_PWM_UPPER_LIMIT);

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

void init_server_connection() {
    protocol = new Protocol();
}

void init_motors() {
  log_i("SX: %d", encoder_dx.get_interrupts());
  log_i("DX: %d", encoder_dx.get_interrupts());

  motor_sx.init(KP, KI, KD);
  motor_dx.init(KP, KI, KD);
}