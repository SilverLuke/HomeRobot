#pragma once

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include "communication/protobuf_handler.h"
#include "constants.h"

class Lidar {
public:
    Lidar(const struct device* uart_dev, const struct pwm_dt_spec* motor_pwm);

    bool init();
    void loop(ProtobufHandler& proto_handler);

    void start();
    void stop();

    void set_motor_speed(float freq_hz);

private:
    const struct device* uart_dev_;
    const struct pwm_dt_spec* motor_pwm_;
    
    enum State {
        IDLE,
        WAITING_HEADER,
        READING_DATA
    } state_;

    uint8_t rx_buffer_[1024];
    size_t rx_idx_;

    void process_byte(uint8_t byte, ProtobufHandler& proto_handler);
    
    // RPLIDAR A1 specific
    struct node_info_t {
      uint8_t    sync_quality; 
      uint16_t   angle_q6_checkbit; 
      uint16_t   distance_q2;
    } __attribute__((packed));

    void handle_point(const node_info_t& node, ProtobufHandler& proto_handler);

    static const size_t BATCH_SIZE = constants::LIDAR_BATCH_SIZE;
    ProtobufHandler::LidarPointData points_buffer_[BATCH_SIZE];
    size_t points_count_ = 0;
};
