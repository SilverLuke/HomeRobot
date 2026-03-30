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
    LOG_INF("Starting Lidar...");
    enable_motor(true);
    
    // Reset lidar to clear any stuck state
    LOG_INF("Sending RESET command...");
    uint8_t reset_cmd[] = {CMD_SYNC_BYTE, CMD_RESET};
    for(int i=0; i<2; i++) uart_poll_out(uart_dev_, reset_cmd[i]);
    
    // Give it time to spin up and reset
    k_sleep(K_MSEC(1000));

    // Send Scan command
    LOG_INF("Sending SCAN command...");
    uint8_t scan_cmd[] = {CMD_SYNC_BYTE, CMD_SCAN};
    for(int i=0; i<2; i++) {
        uart_poll_out(uart_dev_, scan_cmd[i]);
        LOG_DBG("Sent byte: 0x%02x", scan_cmd[i]);
    }
    
    state_ = State::WAITING_HEADER;
}

void Lidar::stop() {
    uint8_t stop_cmd[] = {CMD_SYNC_BYTE, CMD_STOP};
    for(int i=0; i<2; i++) uart_poll_out(uart_dev_, stop_cmd[i]);
    enable_motor(false);
    state_ = State::IDLE;
}

void Lidar::enable_motor(bool enable) {
    if (motor_gpio_) {
        gpio_pin_set_dt(motor_gpio_, enable ? 1 : 0);
        LOG_INF("Lidar motor %s", enable ? "ENABLED" : "DISABLED");
    }
}

void Lidar::loop(ProtobufHandler* proto_handler) {
    uint8_t rx_byte;
    while (uart_poll_in(uart_dev_, &rx_byte) == 0) {
        static bool first_byte = true;
        if (first_byte) {
            LOG_INF("Lidar: Receiving raw bytes...");
            first_byte = false;
        }
        process_byte(rx_byte, proto_handler);
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
                    LOG_INF("Lidar: Received valid response header (type 0x%02x)", header->type);
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
                node_info_t* node = (node_info_t*)rx_buffer_;
                handle_point(*node, proto_handler);
                rx_idx_ = 0;
            }
            break;
    }
}

void Lidar::handle_point(const node_info_t& node, ProtobufHandler* proto_handler) {
    if (!proto_handler) return;

    // Check bit (must be 1 for MEAS packets)
    if (!(node.angle_q6_checkbit & RESP_MEAS_CHECKBIT)) return;

    float angle_deg = (float)(node.angle_q6_checkbit >> RESP_MEAS_ANGLE_SHIFT) / 64.0f;
    float distance_mm = (float)node.distance_q2 / 4.0f;
    float quality = (float)(node.sync_quality >> RESP_MEAS_QUALITY_SHIFT);
    bool sync = (node.sync_quality & RESP_MEAS_SYNCBIT);

    points_buffer_[points_count_].angle_deg = angle_deg;
    points_buffer_[points_count_].distance_mm = distance_mm;
    points_buffer_[points_count_].quality = (uint32_t)quality;
    points_buffer_[points_count_].scan_completed = sync;
    points_count_++;

    if (points_count_ >= BATCH_SIZE) {
        LOG_DBG("Sending lidar scan batch (%d points)", points_count_);
        proto_handler->send_lidar_scan(k_uptime_get_32(), points_buffer_, points_count_);
        points_count_ = 0;
    }
}
