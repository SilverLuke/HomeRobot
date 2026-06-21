#include "lidar.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include "../bridge/gazebo_bridge.h"
#endif

LOG_MODULE_REGISTER(lidar, LOG_LEVEL_DBG);

Lidar::Lidar(const struct device* uart_dev, const struct pwm_dt_spec* motor_pwm)
    : uart_dev_(uart_dev), motor_pwm_(motor_pwm), rx_idx_(0) {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    state_ = State::READING_DATA;
#else
    state_ = State::IDLE;
#endif
}

bool Lidar::init() {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    LOG_INF("Lidar (Simulated) initialized.");
    return true;
#else
    if (!device_is_ready(uart_dev_)) {
        LOG_ERR("UART device for Lidar not ready");
        return false;
    }
 
    if (motor_pwm_ && !pwm_is_ready_dt(motor_pwm_)) {
        LOG_ERR("Motor PWM for Lidar not ready");
        return false;
    }
 
    LOG_INF("Lidar driver initialized");
    return true;
#endif
}

void Lidar::start() {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    LOG_INF("Starting Lidar (Simulated)...");
    state_ = State::READING_DATA;
    return;
#else
    printk("Lidar::start() called\n");
    LOG_INF("Starting Lidar...");
    enable_motor(true);
    
    rx_idx_ = 0;
    points_count_ = 0;
    total_bytes_read_ = 0;
    total_points_read_ = 0;
    sync_errors_ = 0;
    sync_locked_ = false;
    consecutive_valid_ = 0;
    consecutive_invalid_ = 0;
    last_sync_ms_ = 0;
    speed_integral_ = 0.0f;

    // Send stop command first to halt any active scan
    uint8_t stop_cmd[] = {CMD_SYNC_BYTE, CMD_STOP};
    for(int i=0; i<2; i++) uart_poll_out(uart_dev_, stop_cmd[i]);
    k_sleep(K_MSEC(100)); // wait for the lidar to process the stop command

    // Flush any leftover scan bytes (maximum 1024 to avoid infinite loops)
    uint8_t dummy;
    int flushed = 0;
    while (flushed < 1024 && uart_poll_in(uart_dev_, &dummy) == 0) {
        flushed++;
    }
    if (flushed > 0) printk("Flushed %d bytes of old scan data\n", flushed);

    // Reset lidar to clear any stuck state
    LOG_INF("Sending RESET command...");
    uint8_t reset_cmd[] = {CMD_SYNC_BYTE, CMD_RESET};
    for(int i=0; i<2; i++) uart_poll_out(uart_dev_, reset_cmd[i]);
    
    // Give it time to reboot
    k_sleep(K_MSEC(2000));

    // Flush welcome banner (maximum 2048 to avoid infinite loops)
    flushed = 0;
    while (flushed < 2048 && uart_poll_in(uart_dev_, &dummy) == 0) {
        flushed++;
    }
    if (flushed > 0) printk("Flushed %d bytes of welcome banner\n", flushed);

    // Send Scan command
    LOG_INF("Sending SCAN command...");
    uint8_t scan_cmd[] = {CMD_SYNC_BYTE, CMD_SCAN};
    for(int i=0; i<2; i++) {
        uart_poll_out(uart_dev_, scan_cmd[i]);
    }
    
    state_ = State::WAITING_HEADER;
    printk("Lidar started, state=WAITING_HEADER\n");
#endif
}

void Lidar::stop() {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    LOG_INF("Stopping Lidar (Simulated)...");
    state_ = State::IDLE;
    return;
#else
    printk("Lidar::stop() called\n");
    uint8_t stop_cmd[] = {CMD_SYNC_BYTE, CMD_STOP};
    for(int i=0; i<2; i++) uart_poll_out(uart_dev_, stop_cmd[i]);
    enable_motor(false);
    state_ = State::IDLE;
    rx_idx_ = 0;
    points_count_ = 0;
#endif
}

void Lidar::enable_motor(bool enable) {
#if !defined(CONFIG_BOARD_NATIVE_SIM)
    if (motor_pwm_) {
        uint32_t pulse = enable ? (uint32_t)(0.6f * (float)motor_pwm_->period) : 0;
        int ret = pwm_set_pulse_dt(motor_pwm_, pulse);
        if (ret < 0) LOG_ERR("Failed to set motor PWM: %d", ret);
        LOG_INF("Lidar motor %s", enable ? "ENABLED (60% PWM)" : "DISABLED");
    }
#endif
}
void Lidar::loop(ProtobufHandler* proto_handler) {
#if defined(CONFIG_BOARD_NATIVE_SIM)
    if (!proto_handler) return;

    homerobot_LidarScan scan = homerobot_LidarScan_init_default;
    if (GazeboBridge::get_virtual_lidar(&scan)) {
        LOG_DBG("Forwarding virtual Lidar scan (%d points)", scan.points_count);

        // Convert Nanopb LidarScan back to ProtobufHandler format for consistency
        // or just send it directly if possible. 
        // ProtobufHandler::send_lidar_scan expects an array of LidarPointData.

        static ProtobufHandler::LidarPointData pts[200];
        for (pb_size_t i = 0; i < scan.points_count; i++) {
            pts[i].angle_deg = scan.points[i].angle_deg;
            pts[i].distance_mm = scan.points[i].distance_mm;
            pts[i].quality = scan.points[i].quality;
            pts[i].scan_completed = scan.points[i].scan_completed;
        }

        proto_handler->send_lidar_scan(k_uptime_get_32(), pts, scan.points_count);
    }
#else
    if (uart_dev_ == nullptr || !device_is_ready(uart_dev_)) {
        return;
    }
    uint8_t rx_byte;

    int bytes_in_this_loop = 0;
    while (uart_poll_in(uart_dev_, &rx_byte) == 0) {
        total_bytes_read_++;
        bytes_in_this_loop++;
        process_byte(rx_byte, proto_handler);
        if (bytes_in_this_loop > 2048) break;
    }

    uint32_t now = k_uptime_get_32();
    if (now - last_log_ms_ >= 1000) {
        const char* state_str = "UNKNOWN";
        switch(state_) {
            case State::IDLE: state_str = "IDLE"; break;
            case State::WAITING_HEADER: state_str = "WAIT_HDR"; break;
            case State::READING_DATA: state_str = "READ_DATA"; break;
        }
        if (state_ != State::IDLE) {
            printk("[lidar] Status: State=%s, BytesRcv=%u, PtsRcv=%u, PtsRcvGood=%u, PtsRcvErrors=%u\n",
                   state_str, total_bytes_read_, total_points_read_ + sync_errors_, total_points_read_, sync_errors_);
        }
        last_log_ms_ = now;
    }
#endif
}

void Lidar::process_byte(uint8_t byte, ProtobufHandler* proto_handler) {
    switch (state_) {
        case State::IDLE:
            break;
        case State::WAITING_HEADER:
            rx_buffer_[rx_idx_++] = byte;
            if (rx_idx_ >= sizeof(ans_header_t)) {
                ans_header_t* header = (ans_header_t*)rx_buffer_;
                if (header->syncByte1 == ANS_SYNC_BYTE1 && 
                    header->syncByte2 == ANS_SYNC_BYTE2 &&
                    header->type == ANS_TYPE_MEAS) {
                    LOG_INF("Lidar: Received valid response header (type 0x%02x, size %u)", header->type, header->size());
                    state_ = State::READING_DATA;
                    rx_idx_ = 0;
                } else {
                    // Shift buffer and keep looking
                    memmove(rx_buffer_, rx_buffer_ + 1, rx_idx_ - 1);
                    rx_idx_--;
                }
            }
            break;
        case State::READING_DATA:
            rx_buffer_[rx_idx_++] = byte;

            if (rx_idx_ >= sizeof(node_info_t)) {
                uint8_t s = rx_buffer_[0] & 0x01;
                uint8_t s_inv = (rx_buffer_[0] & 0x02) >> 1;
                bool first_byte_ok = (s != s_inv);
                bool second_byte_ok = (rx_buffer_[1] & 0x01);

                if (first_byte_ok && second_byte_ok) {
                    consecutive_valid_++;
                    consecutive_invalid_ = 0;
                    if (!sync_locked_ && consecutive_valid_ >= 100) {
                        sync_locked_ = true;
                        LOG_INF("Sync lock ACQUIRED");
                    }

                    if (sync_locked_) {
                        node_info_t* node = (node_info_t*)rx_buffer_;
                        handle_point(*node, proto_handler);
                    }
                    rx_idx_ = 0;
                } else {
                    sync_errors_++;
                    consecutive_valid_ = 0;

                    if (sync_locked_) {
                        sync_locked_ = false;
                        consecutive_invalid_ = 0;
                        LOG_WRN("Sync lock LOST");
                    }

                    // Shift by 1 byte to search for start-of-packet alignment
                    memmove(rx_buffer_, rx_buffer_ + 1, rx_idx_ - 1);
                    rx_idx_--;
                }
            }
            break;
    }
}

void Lidar::handle_point(const node_info_t& node, ProtobufHandler* proto_handler) {
    if (!proto_handler) return;
 
    float angle_deg = (float)(node.angle_q6_checkbit >> RESP_MEAS_ANGLE_SHIFT) / 64.0f;
    float distance_mm = (float)node.distance_q2 / 4.0f;
    float quality = (float)(node.sync_quality >> RESP_MEAS_QUALITY_SHIFT);
    bool sync = (node.sync_quality & RESP_MEAS_SYNCBIT);
 
    if (sync) {
        uint32_t now = k_uptime_get_32();
        if (last_sync_ms_ != 0) {
            uint32_t diff = now - last_sync_ms_;
            if (diff > 0) {
                actual_frequency_ = 1000.0f / (float)diff;
 
                // PI closed-loop speed control
                float error = target_frequency_ - actual_frequency_;
                const float kp = 12.0f;
                const float ki = 1.5f;
                const float dt = (float)diff / 1000.0f;
 
                speed_integral_ += error * dt;
                // Clamp integral
                if (speed_integral_ > 80.0f) speed_integral_ = 80.0f;
                if (speed_integral_ < -80.0f) speed_integral_ = -80.0f;
 
                float ctrl_out = (kp * error) + (ki * speed_integral_);
                
                // Base feedforward PWM duty cycle (approximate)
                float base_pwm = 150.0f + (target_frequency_ - 5.0f) * 15.0f;
                float final_pwm = base_pwm + ctrl_out;
 
                if (final_pwm > 255.0f) final_pwm = 255.0f;
                if (final_pwm < 80.0f) final_pwm = 80.0f;
 
                #if !defined(CONFIG_BOARD_NATIVE_SIM)
                if (motor_pwm_) {
                    uint32_t pulse = (uint32_t)((final_pwm / 255.0f) * (float)motor_pwm_->period);
                    pwm_set_pulse_dt(motor_pwm_, pulse);
                }
                #endif
            }
        }
        last_sync_ms_ = now;
    }
 
    points_buffer_[points_count_].angle_deg = angle_deg;
    points_buffer_[points_count_].distance_mm = distance_mm;
    points_buffer_[points_count_].quality = (uint32_t)quality;
    points_buffer_[points_count_].scan_completed = sync;
    points_count_++;
    total_points_read_++;

    if (points_count_ >= BATCH_SIZE) {
        proto_handler->send_lidar_scan(k_uptime_get_32(), points_buffer_, points_count_);
        points_count_ = 0;
    }
}
