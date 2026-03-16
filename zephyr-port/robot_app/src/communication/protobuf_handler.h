#pragma once

#include <zephyr/kernel.h>
#include <messages.pb.h>
#include "net_client.h"

class ProtobufHandler {
public:
    ProtobufHandler(NetClient& client);

    bool send_imu_data(uint32_t millis, float acc_x, float acc_y, float acc_z, 
                       float gyro_x, float gyro_y, float gyro_z);
    
    bool send_battery_status(uint32_t millis, uint32_t percentage, uint32_t voltage_mv, uint32_t raw_value);

    bool send_encoders_data(uint32_t millis, int32_t left, int32_t right);

    bool send_heartbeat(uint32_t millis);

    bool send_lidar_point(uint32_t millis, float distance_mm, float angle_deg, uint32_t quality, bool scan_completed);

    bool receive_and_decode(homerobot_ServerToRobotMessage& message);

    // TODO: Add more telemetry types (Encoders, etc.)

private:
    NetClient& client_;
    uint8_t buffer_[512]; // Buffer for serialized data

    // RX state
    uint8_t rx_buffer_[512];
    size_t rx_idx_;
    uint16_t expected_size_;
    bool reading_header_;

    bool encode_and_send(const homerobot_RobotToServerMessage& message);
};
