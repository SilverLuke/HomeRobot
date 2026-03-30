#pragma once

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>

#include "actuator/status_led.h"
#include "actuator/motor.h"
#include "sensors/lidar.h"
#include "sensors/battery.h"
#include "sensors/encoders.h"
#include "sensors/imu.h"
#include "communication/wifi_manager.h"
#include "communication/zephyr_net_client.h"
#include "communication/protobuf_handler.h"
#include "diagnostic.h"

enum class RobotState {
    POWER_CHECK,
    INITIALIZE_HARDWARE,
    WIFI_CONNECTING,
    SERVER_CONNECTING,
    OPERATIONAL
};

class Robot {
public:
    Robot();
    void setup();
    void loop();

private:
    // State machine
    RobotState state_ = RobotState::POWER_CHECK;
    void set_state(RobotState new_state);
    const char* state_to_string(RobotState state);
    void handle_power_check();
    void handle_initialize_hardware();
    void handle_wifi_connecting();
    void handle_server_connecting();
    void handle_operational();

    // Helpers
    void send_telemetry();
    void process_commands();
    void handle_server_message(homerobot_ServerToRobotMessage& msg);

    // Hardware Components
    StatusLed status_led_;
    Battery battery_;
    Lidar lidar_;
    Imu imu_;
    Encoders enc_sx_;
    Encoders enc_dx_;
    Motor motor_sx_;
    Motor motor_dx_;

    // Communication
    WifiManager& wifi_;
    ZephyrNetClient net_client_;
    ProtobufHandler proto_handler_;
    Diagnostic diagnostic_;

    // Configuration & State
    float motor_kp_ = 1.0f;
    float motor_ki_ = 0.01f;
    float motor_kd_ = 0.1f;

    uint32_t last_telemetry_ms_ = 0;
    uint32_t last_fast_telemetry_ms_ = 0;
    uint32_t last_imu_log_ms_ = 0;

    // DT Specs (passed to constructors in initializer list)
    static const struct device *const lidar_uart_dev;
    static const struct gpio_dt_spec lidar_en_gpio;
    static const struct device *const adc_dev;
    static const struct device *const imu_dev;
    static const struct pwm_dt_spec motor_sx_fwd;
    static const struct pwm_dt_spec motor_sx_bwd;
    static const struct pwm_dt_spec motor_dx_fwd;
    static const struct pwm_dt_spec motor_dx_bwd;
    static const struct device *const encoder_dev;
};
