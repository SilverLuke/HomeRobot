#include "lidar.h"
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(lidar, LOG_LEVEL_DBG);

Lidar::Lidar(const struct device* uart_dev, const struct gpio_dt_spec* motor_gpio)
    : uart_dev_(uart_dev), motor_gpio_(motor_gpio), state_(IDLE), rx_idx_(0), points_count_(0) {}

bool Lidar::init() {
    if (!device_is_ready(uart_dev_)) {
        LOG_ERR("UART device not ready");
        return false;
    }
    if (!gpio_is_ready_dt(motor_gpio_)) {
        LOG_ERR("Lidar Motor GPIO device not ready");
        return false;
    }
    if (gpio_pin_configure_dt(motor_gpio_, GPIO_OUTPUT_INACTIVE) < 0) {
        LOG_ERR("Failed to configure Lidar Motor GPIO");
        return false;
    }
    return true;
}

void Lidar::start() {
    LOG_INF("Starting Lidar...");
    
    // Reset Lidar
    LOG_INF("Sending RESET command...");
    send_command(CMD_RESET);
    k_msleep(2000); // Wait for Lidar to reboot

    // Stop any current operation
    stop();
    k_msleep(500);

    // Flush RX
    uint8_t dummy;
    int flush_count = 0;
    while (uart_poll_in(uart_dev_, &dummy) == 0 && flush_count < 1000) {
        flush_count++;
    }

    device_info_t info;
    if (get_device_info(info, 1000) == 0) {
        LOG_INF("Lidar Model: %d, Firmware: %d.%d, Hardware: %d", 
                info.model, info.firmware_version >> 8, info.firmware_version & 0xFF, info.hardware_version);
    } else {
        LOG_WRN("Failed to get Lidar device info");
    }

    device_health_t health;
    if (get_health(health, 500) == 0) {
        LOG_INF("Lidar Health Status: %s", health.status == 0 ? "OK" : "Error");
    }

    LOG_INF("Enabling Lidar motor...");
    enable_motor(true);
    k_msleep(1000); // Give it time to spin up
    
    LOG_INF("Sending SCAN command...");
    if (send_command(CMD_SCAN) == 0) {
        ans_header_t header;
        if (wait_response_header(&header, 2000) == 0) {
            if (header.type == ANS_TYPE_MEAS) {
                LOG_INF("Lidar scanning started successfully");
                state_ = READING_DATA;
                rx_idx_ = 0;
            } else {
                LOG_ERR("Unexpected response type: 0x%02X", header.type);
            }
        } else {
            LOG_ERR("Failed to get scan response header");
        }
    } else {
        LOG_ERR("Failed to send scan command");
    }
}

void Lidar::stop() {
    LOG_INF("Stopping Lidar...");
    send_command(CMD_STOP);
    enable_motor(false);
    state_ = IDLE;
}

void Lidar::enable_motor(bool enable) {
    printk("Lidar motor %s\n", enable ? "ENABLED" : "DISABLED");
    gpio_pin_set_dt(motor_gpio_, enable ? 1 : 0);
}

void Lidar::loop(ProtobufHandler* proto_handler) {
    uint8_t byte;
    
    // Raw monitor EVERYTHING
    if (uart_poll_in(uart_dev_, &byte) == 0) {
        if (state_ == IDLE) {
            printk("[Lidar Raw RX] 0x%02x\n", byte);
        } else {
            process_byte(byte, proto_handler);
        }
    }
}

void Lidar::process_byte(uint8_t byte, ProtobufHandler* proto_handler) {
    rx_buffer_[rx_idx_++] = byte;
    
    if (state_ == READING_DATA) {
        if (rx_idx_ == sizeof(node_info_t)) {
            const node_info_t* node = reinterpret_cast<const node_info_t*>(rx_buffer_);
            
            // Check sync bits
            bool b0_sync = (((rx_buffer_[0] >> 1) ^ rx_buffer_[0]) & 0x01);
            bool b1_check = (rx_buffer_[1] & RESP_MEAS_CHECKBIT);

            if (b0_sync && b1_check) {
                handle_point(*node, proto_handler);
            } else {
                printk("Lidar sync error: 0x%02x 0x%02x\n", rx_buffer_[0], rx_buffer_[1]);
                memmove(rx_buffer_, rx_buffer_ + 1, rx_idx_ - 1);
                rx_idx_--;
                return;
            }
            rx_idx_ = 0;
        }
    }
}

void Lidar::handle_point(const node_info_t& node, ProtobufHandler* proto_handler) {
    float distance_mm = node.distance_q2 * 0.25f;
    float angle_deg = (node.angle_q6_checkbit >> RESP_MEAS_ANGLE_SHIFT) * 0.015625f;
    uint8_t quality = (node.sync_quality >> RESP_MEAS_QUALITY_SHIFT);
    bool scan_completed = (node.sync_quality & RESP_MEAS_SYNCBIT);

    if (distance_mm > 0) {
        points_buffer_[points_count_].distance_mm = distance_mm;
        points_buffer_[points_count_].angle_deg = angle_deg;
        points_buffer_[points_count_].quality = quality;
        points_buffer_[points_count_].scan_completed = scan_completed;
        points_count_++;
    }

    if (points_count_ >= BATCH_SIZE || (scan_completed && points_count_ > 0)) {
#ifdef CONFIG_NETWORKING
        if (proto_handler) {
            proto_handler->send_lidar_scan(k_uptime_get_32(), points_buffer_, points_count_);
        }
#else
        static uint32_t last_log = 0;
        if (k_uptime_get_32() - last_log > 2000) {
            printk("Lidar point sample: dist=%.2f, angle=%.2f, qual=%d\n", (double)distance_mm, (double)angle_deg, quality);
            last_log = k_uptime_get_32();
        }
#endif
        points_count_ = 0;
    }
}

int Lidar::send_command(uint8_t cmd, const void * payload, size_t payloadsize) {
    uint8_t checksum = 0;
    uint8_t full_cmd = cmd;

    if (payloadsize && payload)
        full_cmd |= CMDFLAG_HAS_PAYLOAD;

    printk("Lidar TX: 0xA5 0x%02x\n", full_cmd);
    uart_poll_out(uart_dev_, CMD_SYNC_BYTE);
    uart_poll_out(uart_dev_, full_cmd);

    if (full_cmd & CMDFLAG_HAS_PAYLOAD) {
        checksum ^= CMD_SYNC_BYTE;
        checksum ^= full_cmd;
        checksum ^= (payloadsize & 0xFF);

        for (size_t pos = 0; pos < payloadsize; ++pos)
            checksum ^= ((const uint8_t *)payload)[pos];

        uint8_t sizebyte = (uint8_t)payloadsize;
        uart_poll_out(uart_dev_, sizebyte);
        for (size_t pos = 0; pos < payloadsize; ++pos)
            uart_poll_out(uart_dev_, ((const uint8_t *)payload)[pos]);
        uart_poll_out(uart_dev_, checksum);
    }
    return 0;
}

int Lidar::wait_response_header(ans_header_t * header, uint32_t timeout_ms) {
    size_t recvPos = 0;
    uint32_t start_ms = k_uptime_get_32();
    uint8_t *headerBuffer = reinterpret_cast<uint8_t*>(header);

    while (k_uptime_get_32() - start_ms < timeout_ms) {
        uint8_t current_byte;
        if (uart_poll_in(uart_dev_, &current_byte) == 0) {
            printk("Lidar wait_header RX: 0x%02x\n", current_byte);
            switch (recvPos) {
                case 0:
                    if (current_byte != ANS_SYNC_BYTE1) continue;
                    break;
                case 1:
                    if (current_byte != ANS_SYNC_BYTE2) {
                        recvPos = 0;
                        if (current_byte == ANS_SYNC_BYTE1) recvPos = 1;
                        continue;
                    }
                    break;
            }
            headerBuffer[recvPos++] = current_byte;

            if (recvPos == sizeof(ans_header_t)) return 0;
        }
        k_yield();
    }
    return -ETIMEDOUT;
}

int Lidar::get_device_info(device_info_t & info, uint32_t timeout_ms) {
    LOG_INF("Requesting device info...");
    if (send_command(CMD_GET_DEV_INFO) != 0) return -EIO;

    ans_header_t header;
    if (wait_response_header(&header, timeout_ms) != 0) {
        LOG_WRN("Timeout waiting for device info header");
        return -ETIMEDOUT;
    }

    if (header.type != ANS_TYPE_DEV_INFO) {
        LOG_WRN("Unexpected header type for device info: 0x%02x", header.type);
        return -EINVAL;
    }

    size_t recvPos = 0;
    uint32_t start_ms = k_uptime_get_32();
    uint8_t *infobuf = reinterpret_cast<uint8_t*>(&info);

    while (k_uptime_get_32() - start_ms < timeout_ms) {
        uint8_t current_byte;
        if (uart_poll_in(uart_dev_, &current_byte) == 0) {
            printk("Lidar info RX: 0x%02x\n", current_byte);
            infobuf[recvPos++] = current_byte;
            if (recvPos == sizeof(device_info_t)) return 0;
        }
        k_yield();
    }
    return -ETIMEDOUT;
}

int Lidar::get_health(device_health_t & health, uint32_t timeout_ms) {
    LOG_INF("Requesting device health...");
    if (send_command(CMD_GET_DEV_HEALTH) != 0) return -EIO;

    ans_header_t header;
    if (wait_response_header(&header, timeout_ms) != 0) return -ETIMEDOUT;

    if (header.type != ANS_TYPE_DEV_HEALTH) return -EINVAL;

    size_t recvPos = 0;
    uint32_t start_ms = k_uptime_get_32();
    uint8_t *healthbuf = reinterpret_cast<uint8_t*>(&health);

    while (k_uptime_get_32() - start_ms < timeout_ms) {
        uint8_t current_byte;
        if (uart_poll_in(uart_dev_, &current_byte) == 0) {
            healthbuf[recvPos++] = current_byte;
            if (recvPos == sizeof(device_health_t)) return 0;
        }
        k_yield();
    }
    return -ETIMEDOUT;
}
