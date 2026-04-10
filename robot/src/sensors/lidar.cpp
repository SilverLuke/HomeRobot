#include "lidar.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

LOG_MODULE_REGISTER(lidar, LOG_LEVEL_INF);

Lidar::Lidar(const struct device* uart_dev, const struct gpio_dt_spec* motor_gpio)
    : uart_dev_(uart_dev), motor_gpio_(motor_gpio), state_(State::IDLE), rx_idx_(0) {
}

bool Lidar::init() {
    if (!device_is_ready(uart_dev_)) {
        LOG_ERR("UART device for Lidar not ready");
        return false;
    }

    if (motor_gpio_ && !gpio_is_ready_dt(motor_gpio_)) {
        LOG_ERR("Motor GPIO for Lidar not ready");
        return false;
    }

    if (motor_gpio_) {
        gpio_pin_configure_dt(motor_gpio_, GPIO_OUTPUT_INACTIVE);
    }

    LOG_INF("Lidar driver initialized");
    return true;
}

void Lidar::start() {
    printk("Lidar::start() called\n");
    LOG_INF("Starting Lidar...");
    enable_motor(true);
    
    rx_idx_ = 0;
    points_count_ = 0;
    total_bytes_read_ = 0;
    total_points_read_ = 0;

    // Reset lidar to clear any stuck state
    LOG_INF("Sending RESET command...");
    uint8_t reset_cmd[] = {CMD_SYNC_BYTE, CMD_RESET};
    for(int i=0; i<2; i++) uart_poll_out(uart_dev_, reset_cmd[i]);
    
    // Give it time to spin up and reset
    k_sleep(K_MSEC(2000));

    // Flush welcome banner
    uint8_t dummy;
    int flushed = 0;
    while (uart_poll_in(uart_dev_, &dummy) == 0) {
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
}

void Lidar::stop() {
    printk("Lidar::stop() called\n");
    uint8_t stop_cmd[] = {CMD_SYNC_BYTE, CMD_STOP};
    for(int i=0; i<2; i++) uart_poll_out(uart_dev_, stop_cmd[i]);
    enable_motor(false);
    state_ = State::IDLE;
    rx_idx_ = 0;
    points_count_ = 0;
}

void Lidar::enable_motor(bool enable) {
    if (motor_gpio_) {
        int ret = gpio_pin_set_dt(motor_gpio_, enable ? 1 : 0);
        if (ret < 0) LOG_ERR("Failed to set motor GPIO: %d", ret);
        LOG_INF("Lidar motor %s", enable ? "ENABLED" : "DISABLED");
        printk("Lidar motor %s\n", enable ? "ENABLED" : "DISABLED");
    }
}

void Lidar::loop(ProtobufHandler* proto_handler) {
    uint8_t rx_byte;
    int bytes_in_this_loop = 0;
    while (uart_poll_in(uart_dev_, &rx_byte) == 0) {
        total_bytes_read_++;
        bytes_in_this_loop++;
        process_byte(rx_byte, proto_handler);
        if (bytes_in_this_loop > 256) break;
    }
    k_msleep(1); // Small sleep to let other tasks run

    uint32_t now = k_uptime_get_32();
    if (now - last_log_ms_ >= 1000) {
        const char* state_str = "UNKNOWN";
        switch(state_) {
            case State::IDLE: state_str = "IDLE"; break;
            case State::WAITING_HEADER: state_str = "WAIT_HDR"; break;
            case State::READING_DATA: state_str = "READ_DATA"; break;
        }
        if (state_ != State::IDLE) {
            printk("[lidar] Status: State=%s, BytesRcv=%u, PtsRcv=%u\n", state_str, total_bytes_read_, total_points_read_);
        }
        last_log_ms_ = now;
    }
}

void Lidar::process_byte(uint8_t byte, ProtobufHandler* proto_handler) {
    switch (state_) {
        case State::IDLE:
            break;
        case State::WAITING_HEADER:
            rx_buffer_[rx_idx_++] = byte;
            if (rx_idx_ >= sizeof(ans_header_t)) {
                ans_header_t* header = (ans_header_t*)rx_buffer_;
                if (header->syncByte1 == ANS_SYNC_BYTE1 && header->syncByte2 == ANS_SYNC_BYTE2) {
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
            
            // Check first byte validity if we just started a packet
            if (rx_idx_ == 1) {
                uint8_t s = byte & 0x01;
                uint8_t s_inv = (byte & 0x02) >> 1;
                if (s == s_inv) {
                    // Invalid start byte (S and ~S must be different)
                    rx_idx_ = 0;
                    return;
                }
            }
            
            // Check second byte validity (Check bit must be 1)
            if (rx_idx_ == 2) {
                if (!(byte & 0x01)) {
                    // Invalid check bit, this byte is not the second byte of a packet.
                    // We might have missed the first byte.
                    // Reset and try to find a valid start byte.
                    rx_idx_ = 0;
                    return;
                }
            }

            if (rx_idx_ >= sizeof(node_info_t)) {
                node_info_t* node = (node_info_t*)rx_buffer_;
                handle_point(*node, proto_handler);
                rx_idx_ = 0;
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

    points_buffer_[points_count_].angle_deg = angle_deg;
    points_buffer_[points_count_].distance_mm = distance_mm;
    points_buffer_[points_count_].quality = (uint32_t)quality;
    points_buffer_[points_count_].scan_completed = sync;
    points_count_++;
    total_points_read_++;

    if (points_count_ >= BATCH_SIZE) {
        LOG_DBG("Sending lidar scan batch (%d points)", points_count_);
        proto_handler->send_lidar_scan(k_uptime_get_32(), points_buffer_, points_count_);
        points_count_ = 0;
    }
}
