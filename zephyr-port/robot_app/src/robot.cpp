#include "robot.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include "secrets.h"
#include "constants.h"

LOG_MODULE_REGISTER(robot, LOG_LEVEL_INF);

using namespace constants;

// Define static DT specs
const struct device *const Robot::lidar_uart_dev = DEVICE_DT_GET(DT_ALIAS(lidar_uart));
const struct gpio_dt_spec Robot::lidar_en_gpio = GPIO_DT_SPEC_GET(DT_ALIAS(lidar_en), gpios);
const struct device *const Robot::adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
const struct device *const Robot::imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));
const struct pwm_dt_spec Robot::motor_sx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_fwd_pwm));
const struct pwm_dt_spec Robot::motor_sx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_bwd_pwm));
const struct pwm_dt_spec Robot::motor_dx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_fwd_pwm));
const struct pwm_dt_spec Robot::motor_dx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_bwd_pwm));
const struct device *const Robot::encoder_dev = DEVICE_DT_GET(DT_ALIAS(encoder_sx));

Robot::Robot()
    : status_led_(),
      battery_(adc_dev, 2),
      lidar_(lidar_uart_dev, &lidar_en_gpio),
      imu_(imu_dev),
      enc_sx_(encoder_dev, 0),
      enc_dx_(encoder_dev, 1),
      motor_sx_("SX", &motor_sx_fwd, &motor_sx_bwd, &enc_sx_),
      motor_dx_("DX", &motor_dx_fwd, &motor_dx_bwd, &enc_dx_),
      wifi_(WifiManager::instance()),
      net_client_(),
      proto_handler_(net_client_),
      diagnostic_(motor_sx_, motor_dx_, imu_, battery_, status_led_) {
    LOG_DBG("Robot object constructed");
}

void Robot::setup() {
    LOG_INF("--- Robot State Machine Starting ---");
    status_led_.init();
    battery_.init();
    LOG_INF("Initial State: %s", state_to_string(state_));
}

void Robot::set_state(RobotState new_state) {
    if (state_ == new_state) return;
    LOG_INF("State Transition: %s -> %s", state_to_string(state_), state_to_string(new_state));
    state_ = new_state;
}

const char* Robot::state_to_string(RobotState state) {
    switch (state) {
        case RobotState::POWER_CHECK: return "POWER_CHECK";
        case RobotState::INITIALIZE_HARDWARE: return "INITIALIZE_HARDWARE";
        case RobotState::WIFI_CONNECTING: return "WIFI_CONNECTING";
        case RobotState::SERVER_CONNECTING: return "SERVER_CONNECTING";
        case RobotState::OPERATIONAL: return "OPERATIONAL";
        default: return "UNKNOWN";
    }
}

void Robot::loop() {
    switch (state_) {
        case RobotState::POWER_CHECK:
            handle_power_check();
            break;
        case RobotState::INITIALIZE_HARDWARE:
            handle_initialize_hardware();
            break;
        case RobotState::WIFI_CONNECTING:
            handle_wifi_connecting();
            break;
        case RobotState::SERVER_CONNECTING:
            handle_server_connecting();
            break;
        case RobotState::OPERATIONAL:
            handle_operational();
            break;
    }
    lidar_.loop(&proto_handler_);
    status_led_.update();
}

void Robot::handle_power_check() {
    uint32_t voltage = battery_.get_voltage_mv();
    if (voltage < 10000) {
        status_led_.set_status(RobotStatus::LOW_BATTERY);
        static uint32_t last_log = 0;
        if (k_uptime_get_32() - last_log > 2000) {
            LOG_WRN("LOW POWER: %u mV - Please turn on battery switch", voltage);
            last_log = k_uptime_get_32();
        }
    } else {
        LOG_INF("Power OK: %u mV", voltage);
        set_state(RobotState::INITIALIZE_HARDWARE);
    }
}

void Robot::handle_initialize_hardware() {
    LOG_INF("Initializing hardware components...");
    enc_sx_.init();
    enc_dx_.init();
    imu_.init();
    LOG_INF("Initializing Lidar...");
    lidar_.init();
    motor_sx_.init(motor_kp_, motor_ki_, motor_kd_);
    motor_dx_.init(motor_kp_, motor_ki_, motor_kd_);

    LOG_INF("Hardware ready. Starting Wi-Fi connection...");
    wifi_.connect(wifi_ssid, wifi_password);
    status_led_.set_status(RobotStatus::NO_WIFI);

    set_state(RobotState::WIFI_CONNECTING);
}

void Robot::handle_wifi_connecting() {
    if (wifi_.is_connected()) {
        LOG_INF("Wi-Fi connected.");
        set_state(RobotState::SERVER_CONNECTING);
    } else {
        status_led_.set_status(RobotStatus::NO_WIFI);
        // Wi-Fi manager handles retries automatically based on its implementation
    }
}

void Robot::handle_server_connecting() {
    if (!wifi_.is_connected()) {
        LOG_WRN("Wi-Fi lost while connecting to server.");
        set_state(RobotState::WIFI_CONNECTING);
        return;
    }

    status_led_.set_status(RobotStatus::WIFI_ONLY);
    LOG_INF("Connecting to server: %s:%d", wifi_server_host, wifi_server_port);

    if (net_client_.connect(wifi_server_host, wifi_server_port)) {
        LOG_INF("CONNECTED to server at %s:%d", wifi_server_host, wifi_server_port);
        status_led_.set_status(RobotStatus::CONNECTED);
        
        // Send current config on connection
        proto_handler_.send_robot_config(k_uptime_get_32(),
            motor_kp_, motor_ki_, motor_kd_, motor_kp_, motor_ki_, motor_kd_);
        
        set_state(RobotState::OPERATIONAL);
    } else {
        LOG_ERR("Server connection failed, retrying...");
        // Call telemetry even while connecting to see serial logs
        send_telemetry();
        k_sleep(K_MSEC(2000));
    }
}

void Robot::handle_operational() {
    if (!wifi_.is_connected()) {
        LOG_WRN("Wi-Fi lost, reconnecting...");
        motor_sx_.set_motor(BRAKE, 0);
        motor_dx_.set_motor(BRAKE, 0);
        set_state(RobotState::WIFI_CONNECTING);
        return;
    }

    if (!net_client_.connected()) {
        LOG_WRN("Server connection lost, reconnecting...");
        motor_sx_.set_motor(BRAKE, 0);
        motor_dx_.set_motor(BRAKE, 0);
        set_state(RobotState::SERVER_CONNECTING);
        return;
    }

    // 1. Process incoming commands
    process_commands();

    // 2. Periodic telemetry
    send_telemetry();
}

void Robot::process_commands() {
    homerobot_ServerToRobotMessage rx_msg;
    while (proto_handler_.receive_and_decode(rx_msg)) {
        handle_server_message(rx_msg);
    }
}

void Robot::handle_server_message(homerobot_ServerToRobotMessage& msg) {
    if (msg.which_payload == homerobot_ServerToRobotMessage_motor_move_tag) {
        int lp = (int)msg.payload.motor_move.left_power;
        int rp = (int)msg.payload.motor_move.right_power;
        float la = msg.payload.motor_move.left_angle;
        float ra = msg.payload.motor_move.right_angle;

        if (lp == 0) motor_sx_.set_motor(BRAKE, 0);
        else motor_sx_.set_motor(la >= 0 ? FORWARD : BACKWARD, (uint8_t)lp);

        if (rp == 0) motor_dx_.set_motor(BRAKE, 0);
        else motor_dx_.set_motor(ra >= 0 ? FORWARD : BACKWARD, (uint8_t)rp);
    }
    else if (msg.which_payload == homerobot_ServerToRobotMessage_motor_config_tag) {
        if (msg.payload.motor_config.has_left_motor) {
            motor_kp_ = msg.payload.motor_config.left_motor.kp;
            motor_ki_ = msg.payload.motor_config.left_motor.ki;
            motor_kd_ = msg.payload.motor_config.left_motor.kd;
            motor_sx_.init(motor_kp_, motor_ki_, motor_kd_);
            motor_dx_.init(motor_kp_, motor_ki_, motor_kd_);
            LOG_INF("PID UPDATED: P=%.3f I=%.3f D=%.3f", (double)motor_kp_, (double)motor_ki_, (double)motor_kd_);
        }
    }
    else if (msg.which_payload == homerobot_ServerToRobotMessage_rpc_request_tag) {
        diagnostic_.run_rpc();
        proto_handler_.send_rpc_response(k_uptime_get_32(), msg.payload.rpc_request.call_id, nullptr, 0);
    }
    else if (msg.which_payload == homerobot_ServerToRobotMessage_lidar_control_tag) {
        bool active = msg.payload.lidar_control.active;
        LOG_INF("RPC: Lidar control received: active=%s", active ? "true" : "false");
        if (active) lidar_.start();
        else lidar_.stop();
    }
    else if (msg.which_payload == homerobot_ServerToRobotMessage_stop_all_tag) {
        LOG_INF("RPC: Stop all received");
        motor_sx_.set_motor(BRAKE, 0);
        motor_dx_.set_motor(BRAKE, 0);
        lidar_.stop();
    }
}

void Robot::send_telemetry() {
    uint32_t now = k_uptime_get_32();

    // Fast telemetry (IMU & Encoders) - 100ms
    if (now - last_fast_telemetry_ms_ >= 100) {
        bool imu_ok = imu_.update();
        float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        
        if (imu_ok) {
            imu_.get_accel(ax, ay, az);
            imu_.get_gyro(gx, gy, gz);
            proto_handler_.send_imu_data(now, ax, ay, az, gx, gy, gz);
        }
        proto_handler_.send_encoders_data(now, enc_sx_.get_total_ticks(), enc_dx_.get_total_ticks());
        last_fast_telemetry_ms_ = now;

        // Serial log for IMU - 1000ms
        if (now - last_imu_log_ms_ >= 1000) {
            if (imu_ok) {
                LOG_INF("[IMU] Acc: X=%.2f Y=%.2f Z=%.2f | Gyro: X=%.2f Y=%.2f Z=%.2f", 
                               (double)ax, (double)ay, (double)az, (double)gx, (double)gy, (double)gz);
            } else {
                LOG_ERR("[IMU] ERROR: Failed to update sensor data");
            }
            last_imu_log_ms_ = now;
        }
    }

    // Slow telemetry (Battery) - 5000ms
    if (now - last_telemetry_ms_ >= 5000) {
        proto_handler_.send_battery_status(now, battery_.get_percentage(), 
                                           battery_.get_voltage_mv(), battery_.read_raw());
        last_telemetry_ms_ = now;
    }
}
