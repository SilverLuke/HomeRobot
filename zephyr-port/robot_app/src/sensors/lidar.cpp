#include "lidar.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lidar, LOG_LEVEL_DBG);

#define RESP_MEAS_CHECKBIT (0x1<<0)
#define RESP_MEAS_SYNCBIT (0x1<<0)
#define RESP_MEAS_QUALITY_SHIFT 2
#define RESP_MEAS_ANGLE_SHIFT 1

Lidar::Lidar(const struct device* uart_dev, const struct pwm_dt_spec* motor_pwm)
    : uart_dev_(uart_dev), motor_pwm_(motor_pwm), state_(IDLE), rx_idx_(0) {}

bool Lidar::init() {
    if (!device_is_ready(uart_dev_)) {
        LOG_ERR("UART device not ready");
        return false;
    }
    if (!pwm_is_ready_dt(motor_pwm_)) {
        LOG_ERR("Lidar Motor PWM device not ready");
        return false;
    }
    return true;
}

void Lidar::start() {
    LOG_INF("Starting Lidar...");
    set_motor_speed(5.0); // 5Hz
    k_msleep(500);
    
    // Send start scan command
    uint8_t cmd[] = { 0xA5, 0x20 };
    for (int i = 0; i < sizeof(cmd); i++) {
        uart_poll_out(uart_dev_, cmd[i]);
    }
    state_ = WAITING_HEADER;
    rx_idx_ = 0;
}

void Lidar::stop() {
    LOG_INF("Stopping Lidar...");
    uint8_t cmd[] = { 0xA5, 0x25 };
    for (int i = 0; i < sizeof(cmd); i++) {
        uart_poll_out(uart_dev_, cmd[i]);
    }
    set_motor_speed(0);
    state_ = IDLE;
}

void Lidar::set_motor_speed(float freq_hz) {
    uint32_t period = 1000000000 / 10000; // 10kHz
    uint32_t pulse = 0;
    if (freq_hz > 0) {
        // Simple proportional mapping or fixed for now
        pulse = (uint32_t)(period * 0.7); 
    }
    pwm_set_dt(motor_pwm_, period, pulse);
}

void Lidar::loop(ProtobufHandler& proto_handler) {
    if (state_ == IDLE) return;

    uint8_t byte;
    while (uart_poll_in(uart_dev_, &byte) == 0) {
        process_byte(byte, proto_handler);
    }
}

void Lidar::process_byte(uint8_t byte, ProtobufHandler& proto_handler) {
    switch (state_) {
        case WAITING_HEADER:
            // A1 response header is 7 bytes: A5 5A 05 00 00 40 81
            rx_buffer_[rx_idx_++] = byte;
            if (rx_idx_ == 7) {
                if (rx_buffer_[0] == 0xA5 && rx_buffer_[1] == 0x5A) {
                    LOG_INF("Lidar header received");
                    state_ = READING_DATA;
                } else {
                    LOG_WRN("Invalid Lidar header, retrying...");
                }
                rx_idx_ = 0;
            }
            break;

        case READING_DATA:
            rx_buffer_[rx_idx_++] = byte;
            if (rx_idx_ == 5) {
                // Check sync bits for byte 0 and 1
                bool b0_sync = (((rx_buffer_[0] >> 1) ^ rx_buffer_[0]) & 0x01);
                bool b1_check = (rx_buffer_[1] & RESP_MEAS_CHECKBIT);

                if (b0_sync && b1_check) {
                    handle_point(*(node_info_t*)rx_buffer_, proto_handler);
                } else {
                    LOG_WRN("Lidar sync error, resyncing...");
                    // Try to find sync by shifting? For now just reset
                }
                rx_idx_ = 0;
            }
            break;
        
        default:
            break;
    }
}

void Lidar::handle_point(const node_info_t& node, ProtobufHandler& proto_handler) {
    float distance_mm = node.distance_q2 * 0.25f;
    float angle_deg = (node.angle_q6_checkbit >> RESP_MEAS_ANGLE_SHIFT) * 0.015625f;
    uint8_t quality = (node.sync_quality >> RESP_MEAS_QUALITY_SHIFT);
    bool scan_completed = (node.sync_quality & RESP_MEAS_SYNCBIT);

    proto_handler.send_lidar_point(k_uptime_get_32(), distance_mm, angle_deg, quality, scan_completed);
}
