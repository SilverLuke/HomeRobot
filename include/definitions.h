#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <Encoder.h>
#include <Motor_PID.h>
#include <FastIMU.h>
#include <LDS.h>
#include <LDS_RPLIDAR_A1.h>
#include <HardwareSerial.h>

// MOTOR SX
#define ENCODER_SX_A 20  // YELLOW from NXT motor encoder
#define ENCODER_SX_B 21  // WHITE from NXT motor encoder
#define MOTOR_SX_FORWARD  12  // in2 from driver to control direction
#define MOTOR_SX_BACKWARD 13   // in1 from driver to control direction
#define MOTOR_SX_PWM 0  // pwm to control speed pin 0 for full speed

// MOTOR DX
#define ENCODER_DX_A 22 // YELLOW from NXT motor encoder
#define ENCODER_DX_B 23 // WHITE from NXT motor encoder
#define MOTOR_DX_FORWARD  10  // in2 from driver pins to control direction
#define MOTOR_DX_BACKWARD 11  // in1 from driver pins to control direction
#define MOTOR_DX_PWM 0 // pwm to control speed pin 0 for full speed

// PID constants
#define KP 10.
#define KI 0.1
#define KD 0.3


/*
I2C Sensors
// BMI160    ACC + GYRO   On address: 0x69
// HMC5883L  MAGNETOMETER On address: 0x0D
// BMI160_HMC5883L
*/
#define I2C_SDA 6
#define I2C_SCL 7
#define IMU_ADDR 0x68

/*
LIDAR
*/
#define LDS_MOTOR_PWM_PIN  15 // LiDAR motor speed control using PWM
#define LDS_MOTOR_PWM_FREQ    10000
#define LDS_MOTOR_PWM_BITS    11
#define LDS_MOTOR_PWM_CHANNEL    2 // ESP32 PWM channel for LiDAR motor speed control
#define UART_NUM_0  0  // TODO check this 0

extern int pwm_lower_limit;
extern int pwm_upper_limit;

extern Encoder encoder_sx;
extern Encoder encoder_dx;
extern Motor motor_sx;
extern Motor motor_dx;
extern BMI160 imu;
extern calData calib;
extern AccelData IMUAccel;
extern GyroData IMUGyro;
extern MagData IMUMag;

extern HardwareSerial LidarSerial;
extern LDS_RPLIDAR_A1 lidar;

void init_i2c();
void init_imu();
void init_serial_lidar();
void init_lidar();
void init_motors();

#endif // DEFINITIONS_H