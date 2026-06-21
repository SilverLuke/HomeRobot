#include "robot.h"
#include <pb_decode.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>
#include "secrets.h"
#include "constants.h"

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include "bridge/gazebo_bridge.h"
#endif

LOG_MODULE_REGISTER(robot, LOG_LEVEL_INF);

using namespace constants;

#if !defined(CONFIG_BOARD_NATIVE_SIM)
// Define static DT specs
const struct device *const Robot::lidar_uart_dev = DEVICE_DT_GET(DT_ALIAS(lidar_uart));
const struct pwm_dt_spec Robot::lidar_pwm = PWM_DT_SPEC_GET(DT_ALIAS(lidar_pwm));
const struct device *const Robot::adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc0));
const struct device *const Robot::imu_dev = DEVICE_DT_GET(DT_ALIAS(imu));
const struct pwm_dt_spec Robot::motor_sx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_fwd_pwm));
const struct pwm_dt_spec Robot::motor_sx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_sx_bwd_pwm));
const struct pwm_dt_spec Robot::motor_dx_fwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_fwd_pwm));
const struct pwm_dt_spec Robot::motor_dx_bwd = PWM_DT_SPEC_GET(DT_ALIAS(motor_dx_bwd_pwm));
const struct device *const Robot::encoder_dev = DEVICE_DT_GET(DT_ALIAS(encoder_sx));
#else
// Dummy values for simulation
const struct device *const Robot::lidar_uart_dev = nullptr;
const struct pwm_dt_spec Robot::lidar_pwm = {0};
const struct device *const Robot::adc_dev = nullptr;
const struct device *const Robot::imu_dev = nullptr;
const struct pwm_dt_spec Robot::motor_sx_fwd = {0};
const struct pwm_dt_spec Robot::motor_sx_bwd = {0};
const struct pwm_dt_spec Robot::motor_dx_fwd = {0};
const struct pwm_dt_spec Robot::motor_dx_bwd = {0};
const struct device *const Robot::encoder_dev = nullptr;
#endif

Robot::Robot()
    : status_led_(),
      battery_(adc_dev, 2),
      lidar_(lidar_uart_dev, &lidar_pwm),
      imu_(imu_dev),
      enc_sx_(encoder_dev, 0),
      enc_dx_(encoder_dev, 1),
      motor_sx_("SX", &motor_sx_fwd, &motor_sx_bwd, &enc_sx_, true),
      motor_dx_("DX", &motor_dx_fwd, &motor_dx_bwd, &enc_dx_, false),
      wifi_(WifiManager::instance()),
      net_client_(),
      proto_handler_(net_client_),
      diagnostic_(motor_sx_, motor_dx_, imu_, battery_, status_led_) {
    LOG_DBG("Robot object constructed");
}

void Robot::setup() {
    LOG_INF("--- Robot State Machine Starting ---");
#if defined(CONFIG_BOARD_NATIVE_SIM)
    GazeboBridge::init();
    GazeboBridge::start();
#endif
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
 
    // Handle Lidar stop timeout on server disconnection
    if (state_ != RobotState::OPERATIONAL && !lidar_stopped_due_to_disconnect_) {
        if (disconnect_time_ms_ != 0 && k_uptime_get_32() - disconnect_time_ms_ >= 10000) {
            LOG_INF("Disconnected from server for >= 10s. Stopping Lidar...");
            lidar_.stop();
            lidar_stopped_due_to_disconnect_ = true;
        }
    }
 
    motor_sx_.loop();
    motor_dx_.loop();

    if (motion_active_) {
        if (motor_sx_.target_reached(false) && motor_dx_.target_reached(false)) {
            LOG_INF("ExecuteMotion: Target reached successfully");
            motor_sx_.target_reached(true);
            motor_dx_.target_reached(true);
            proto_handler_.send_rpc_response(k_uptime_get_32(), active_motion_call_id_, nullptr, 0);
            motion_active_ = false;
        } else if (k_uptime_get_32() - motion_start_time_ms_ > 10000) { // 10s timeout
            LOG_WRN("ExecuteMotion: Safety timeout exceeded");
            motor_sx_.target_reached(true);
            motor_dx_.target_reached(true);
            motor_sx_.set_motor(BRAKE, 0);
            motor_dx_.set_motor(BRAKE, 0);
            proto_handler_.send_rpc_response(k_uptime_get_32(), active_motion_call_id_, nullptr, 0, "Timeout");
            motion_active_ = false;
        }
    }

    lidar_.loop(&proto_handler_);
    status_led_.update();
}

void Robot::handle_power_check() {
    uint32_t voltage = battery_.get_voltage_mv();
#if defined(DISABLE_BATTERY_CHECK)
    LOG_INF("Power check bypassed (bench power supply mode). Voltage: %u mV", voltage);
#else
    if (voltage < 10000) {
        status_led_.set_status(RobotStatus::LOW_BATTERY);
        static uint32_t last_log = 0;
        if (k_uptime_get_32() - last_log > 2000) {
            LOG_WRN("LOW POWER: %u mV - Please turn on battery switch", voltage);
            last_log = k_uptime_get_32();
        }
        return;
    }
    LOG_INF("Power OK: %u mV", voltage);
#endif

#if defined(CONFIG_BOARD_NATIVE_SIM)
    set_state(RobotState::INITIALIZE_HARDWARE);
#else
    LOG_INF("Starting Wi-Fi connection early...");
    wifi_.connect(wifi_ssid, wifi_password);
    status_led_.set_status(RobotStatus::NO_WIFI);
    set_state(RobotState::WIFI_CONNECTING);
#endif
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
    hardware_initialized_ = true;

    LOG_INF("Hardware ready. Connecting to server...");
    set_state(RobotState::SERVER_CONNECTING);
}

void Robot::handle_wifi_connecting() {
    if (wifi_.is_connected()) {
        LOG_INF("Wi-Fi connected.");
        if (!hardware_initialized_) {
            set_state(RobotState::INITIALIZE_HARDWARE);
        } else {
            set_state(RobotState::SERVER_CONNECTING);
        }
    } else {
        status_led_.set_status(RobotStatus::NO_WIFI);
        
        static uint32_t last_connect_attempt = 0;
        uint32_t now = k_uptime_get_32();
        if (now - last_connect_attempt > 5000) {
            LOG_INF("Retrying Wi-Fi connection...");
            wifi_.connect(wifi_ssid, wifi_password);
            last_connect_attempt = now;
        }
    }
}

void Robot::handle_server_connecting() {
#if !defined(CONFIG_BOARD_NATIVE_SIM)
    if (!wifi_.is_connected()) {
        LOG_WRN("Wi-Fi lost while connecting to server.");
        set_state(RobotState::WIFI_CONNECTING);
        return;
    }
#endif

    status_led_.set_status(RobotStatus::WIFI_ONLY);
    
    const char* server_host = wifi_server_host;
#if defined(CONFIG_BOARD_NATIVE_SIM)
    server_host = "127.0.0.1";
#endif

    LOG_INF("Connecting to server: %s:%d", server_host, wifi_server_port);

    if (net_client_.connect(server_host, wifi_server_port)) {
        LOG_INF("CONNECTED to server at %s:%d", server_host, wifi_server_port);
        status_led_.set_status(RobotStatus::CONNECTED);
        
        // Small delay to ensure the network stack is ready for transmission
        k_msleep(100);
        
        // Send current config on connection
        if (net_client_.connected()) {
            proto_handler_.send_robot_config(k_uptime_get_32(),
                motor_kp_, motor_ki_, motor_kd_, motor_kp_, motor_ki_, motor_kd_, lidar_frequency_);
        }
        
        // Reset encoder baselines so the first telemetry packet sends 0 delta
        last_sent_enc_sx_ = enc_sx_.get_total_ticks();
        last_sent_enc_dx_ = enc_dx_.get_total_ticks();
        
        disconnect_time_ms_ = 0;
        lidar_stopped_due_to_disconnect_ = false;
        set_state(RobotState::OPERATIONAL);
    } else {
        LOG_ERR("Server connection failed, retrying...");
        // Call telemetry even while connecting to see serial logs
        send_telemetry();
        k_sleep(K_MSEC(2000));
    }
}

void Robot::handle_operational() {
#if !defined(CONFIG_BOARD_NATIVE_SIM)
    if (!wifi_.is_connected()) {
        LOG_WRN("Wi-Fi lost, reconnecting...");
        motor_sx_.set_motor(BRAKE, 0);
        motor_dx_.set_motor(BRAKE, 0);
        disconnect_time_ms_ = k_uptime_get_32();
        lidar_stopped_due_to_disconnect_ = false;
        set_state(RobotState::WIFI_CONNECTING);
        return;
    }
#endif

    if (!net_client_.connected()) {
        LOG_WRN("Server connection lost, reconnecting...");
        motor_sx_.set_motor(BRAKE, 0);
        motor_dx_.set_motor(BRAKE, 0);
        disconnect_time_ms_ = k_uptime_get_32();
        lidar_stopped_due_to_disconnect_ = false;
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
        
        LOG_INF("MOVE: L=%d (%.1f) R=%d (%.1f)", lp, (double)la, rp, (double)ra);

        if (lp == 0) motor_sx_.set_manual_power(BRAKE, 0);
        else motor_sx_.set_manual_power(la >= 0 ? FORWARD : BACKWARD, (uint8_t)lp);

        if (rp == 0) motor_dx_.set_manual_power(BRAKE, 0);
        else motor_dx_.set_manual_power(ra >= 0 ? FORWARD : BACKWARD, (uint8_t)rp);
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
        float freq = msg.payload.motor_config.lidar_frequency;
        if (freq >= 5.0f && freq <= 10.0f) {
            lidar_frequency_ = freq;
            lidar_.set_target_frequency(freq);
            LOG_INF("LIDAR FREQUENCY UPDATED: %.1f Hz", (double)lidar_frequency_);
        }
    }
    else if (msg.which_payload == homerobot_ServerToRobotMessage_rpc_request_tag) {
        if (strcmp(msg.payload.rpc_request.method, "ExecuteMotion") == 0) {
            homerobot_MotionRequest motion_req = homerobot_MotionRequest_init_default;
            pb_istream_t stream = pb_istream_from_buffer(
                msg.payload.rpc_request.payload.bytes,
                msg.payload.rpc_request.payload.size
            );
            if (pb_decode(&stream, homerobot_MotionRequest_fields, &motion_req)) {
                LOG_INF("RPC ExecuteMotion: Type=%d L=%d R=%d MaxPower=%u",
                        motion_req.type, motion_req.left_ticks, motion_req.right_ticks, motion_req.max_power);
                
                motor_sx_.config_set_limit(5, motion_req.max_power);
                motor_dx_.config_set_limit(5, motion_req.max_power);
                
                int32_t current_sx = motor_sx_.get_position();
                int32_t current_dx = motor_dx_.get_position();
                
                motor_sx_.set_target(current_sx + motion_req.left_ticks);
                motor_dx_.set_target(current_dx + motion_req.right_ticks);
                
                motion_active_ = true;
                active_motion_call_id_ = msg.payload.rpc_request.call_id;
                motion_start_time_ms_ = k_uptime_get_32();
            } else {
                LOG_ERR("Failed to decode MotionRequest");
                proto_handler_.send_rpc_response(k_uptime_get_32(), msg.payload.rpc_request.call_id, nullptr, 0, "Decode error");
            }
        } else {
            diagnostic_.run_rpc();
            proto_handler_.send_rpc_response(k_uptime_get_32(), msg.payload.rpc_request.call_id, nullptr, 0);
        }
    }
    else if (msg.which_payload == homerobot_ServerToRobotMessage_lidar_control_tag) {
        bool active = msg.payload.lidar_control.active;
        float freq = msg.payload.lidar_control.target_frequency_hz;
        if (freq >= 5.0f && freq <= 10.0f) {
            lidar_frequency_ = freq;
            lidar_.set_target_frequency(freq);
        }
        LOG_INF("RPC: Lidar control received: active=%s, freq=%.1f Hz", active ? "true" : "false", (double)lidar_frequency_);
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
        int32_t current_sx = enc_sx_.get_total_ticks();
        int32_t current_dx = enc_dx_.get_total_ticks();
        int32_t delta_sx = current_sx - last_sent_enc_sx_;
        int32_t delta_dx = current_dx - last_sent_enc_dx_;
        if (delta_sx != 0 || delta_dx != 0) {
            proto_handler_.send_encoders_data(now, delta_sx, delta_dx);
            last_sent_enc_sx_ = current_sx;
            last_sent_enc_dx_ = current_dx;
        }
        last_fast_telemetry_ms_ = now;

        // LiDAR Loop
        lidar_.loop(&proto_handler_);

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
