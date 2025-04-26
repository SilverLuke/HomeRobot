#pragma once

#include <Encoder.h>
#include <FastIMU.h>
#include <HardwareSerial.h>
#include <LDS.h>
#include <LDS_RPLIDAR_A1.h>
#include <WiFi.h>
#include <communication/protocol.h>

#include "Motor_PID.h"

// LED
#define RGB_BRIGHTNESS 64

// General motor settings
#define MOTOR_PWM_LOWER_LIMIT 64   // full range 0 - 255
#define MOTOR_PWM_UPPER_LIMIT 128  // full range 0 - 255

// MOTOR SX
#define ENCODER_SX_A 20       // YELLOW from NXT motor encoder
#define ENCODER_SX_B 21       // WHITE from NXT motor encoder
#define MOTOR_SX_FORWARD 19   // in2 from driver to control the direction
#define MOTOR_SX_BACKWARD 18  // in1 from driver to control the direction
#define MOTOR_SX_PWM 0        // pwm to control speed pin 0 for full speed

// MOTOR DX
#define ENCODER_DX_A 22       // YELLOW from NXT motor encoder
#define ENCODER_DX_B 23       // WHITE from NXT motor encoder
#define MOTOR_DX_FORWARD 10   // in2 from driver pins to control the direction
#define MOTOR_DX_BACKWARD 11  // in1 from driver pins to control the direction
#define MOTOR_DX_PWM 0        // pwm to control speed pin 0 for full speed

// PID constants
#define KP 10.0
#define KI 0.1
#define KD 0.3

// I2C pins
#define I2C_SDA 4
#define I2C_SCL 5

// Physical sizes
#define CENTER_TO_WHEEL 100  // millimeters TODO freecad check
#define CENTER_TO_LIDAR 100  // Millimeters TODO freecad check

extern int pwm_lower_limit;
extern int pwm_upper_limit;

extern Encoder encoder_sx;
extern Motor motor_sx;
extern Encoder encoder_dx;
extern Motor motor_dx;

extern Protocol *protocol;

void init_wifi();
void init_i2c();
void init_server_connection();
void init_motors();
