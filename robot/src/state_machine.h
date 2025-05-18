#ifndef ROBOT_H
#define ROBOT_H

#include "Arduino.h"
#include "actuator/motor.h"
#include "communication/protocol.h"
#include "sensors/imu.h"
#include "sensors/lidar.h"

// Constants
#define FAST_REFRESH_HZ 500
#define SLOW_REFRESH_HZ 1000

// Robot states enum
enum ROBOT_STATE {
    IDLE = 0,
    STOP,
    LIDAR,
    WIFI_HARD,
    MOTOR_GO
};

// Function declarations
void set_refresh(unsigned long refresh);
void do_refresh();
ROBOT_STATE serial_commands();
ROBOT_STATE wifi_commands(Protocol* protocol, Lidar* lidar, IMU* imu);
void apply_state(ROBOT_STATE state, Lidar* lidar, IMU* imu);

#endif // ROBOT_H