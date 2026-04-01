#pragma once

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "communication/protobuf_handler.h"
#include "constants.h"

class Lidar {
public:
    Lidar(const struct device* uart_dev, const struct gpio_dt_spec* motor_gpio);

    bool init();
    void loop(ProtobufHandler* proto_handler = nullptr);

    void start();
    void stop();

    void enable_motor(bool enable);

private:
    const struct device* uart_dev_;
    const struct gpio_dt_spec* motor_gpio_;
    
    enum State {
        IDLE,
        WAITING_HEADER,
        READING_DATA
    } state_;

    uint8_t rx_buffer_[1024];
    size_t rx_idx_;

    void process_byte(uint8_t byte, ProtobufHandler* proto_handler);
    
    // RPLIDAR A1 protocol constants
    static constexpr uint8_t CMD_SCAN = 0x20;
    static constexpr uint8_t CMD_FORCE_SCAN = 0x21;
    static constexpr uint8_t CMD_RESET = 0x40;
    static constexpr uint8_t CMD_STOP = 0x25;
    static constexpr uint8_t CMD_GET_DEV_INFO = 0x50;
    static constexpr uint8_t CMD_GET_DEV_HEALTH = 0x52;
    static constexpr uint8_t CMD_SYNC_BYTE = 0xA5;

    static constexpr uint8_t CMDFLAG_HAS_PAYLOAD = 0x80;

    static constexpr uint8_t ANS_SYNC_BYTE1 = 0xA5;
    static constexpr uint8_t ANS_SYNC_BYTE2 = 0x5A;

    static constexpr uint8_t RESP_MEAS_CHECKBIT = (0x1<<0);
    static constexpr uint8_t RESP_MEAS_SYNCBIT = (0x1<<0);
    static constexpr uint8_t RESP_MEAS_QUALITY_SHIFT = 2;
    static constexpr uint8_t RESP_MEAS_ANGLE_SHIFT = 1;

    static constexpr uint8_t ANS_TYPE_MEAS = 0x81;
    static constexpr uint8_t ANS_TYPE_DEV_INFO = 0x4;
    static constexpr uint8_t ANS_TYPE_DEV_HEALTH = 0x6;

    struct device_health_t {
      uint8_t   status;
      uint16_t  error_code;
    } __attribute__((packed));

    struct node_info_t {
      uint8_t    sync_quality; 
      uint16_t   angle_q6_checkbit; 
      uint16_t   distance_q2;
    } __attribute__((packed));
 
    struct device_info_t {
      uint8_t   model;
      uint16_t  firmware_version;
      uint8_t   hardware_version;
      uint8_t   serialnum[16];
    } __attribute__((packed));

    struct ans_header_t {
      uint8_t  syncByte1;
      uint8_t  syncByte2;
      uint8_t  data[4];   // size:30, subType:2
      uint8_t  type;

      uint32_t size() const {
          return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | (((uint32_t)data[2] & 0x3F) << 16);
      }
      uint8_t subType() const {
          return (data[2] >> 6) | (data[3] << 2); // This depends on protocol, usually it's just zero.
      }
    } __attribute__((packed));

    void handle_point(const node_info_t& node, ProtobufHandler* proto_handler);
    int send_command(uint8_t cmd, const void * payload = nullptr, size_t payloadsize = 0);
    int wait_response_header(ans_header_t * header, uint32_t timeout_ms);
    int get_device_info(device_info_t & info, uint32_t timeout_ms);
    int get_health(device_health_t & health, uint32_t timeout_ms);

    static const size_t BATCH_SIZE = constants::LIDAR_BATCH_SIZE;
    ProtobufHandler::LidarPointData points_buffer_[BATCH_SIZE];
    size_t points_count_ = 0;

    uint32_t last_log_ms_ = 0;
    uint32_t total_points_read_ = 0;
    uint32_t total_bytes_read_ = 0;
};
