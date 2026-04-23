#include "gazebo_bridge.h"
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <string.h>
#include <errno.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)
#include <zephyr/sys/byteorder.h>
#include "../actuator/motor.h"
#endif

LOG_MODULE_REGISTER(gazebo_bridge, LOG_LEVEL_INF);

#if defined(CONFIG_BOARD_NATIVE_SIM)

int GazeboBridge::sock_tx_ = -1;
int GazeboBridge::sock_rx_ = -1;
struct sockaddr_in GazeboBridge::addr_tx_;
struct sockaddr_in GazeboBridge::addr_rx_;
static K_THREAD_STACK_DEFINE(bridge_stack, 8192); // Increased stack for Lidar decoding
struct k_thread GazeboBridge::bridge_thread_data;

int32_t GazeboBridge::ticks_[2] = {0, 0};
float GazeboBridge::accel_[3] = {0, 0, 0};
float GazeboBridge::gyro_[3] = {0, 0, 0};
homerobot_LidarScan GazeboBridge::last_scan_ = homerobot_LidarScan_init_default;
bool GazeboBridge::new_scan_available_ = false;
struct k_mutex GazeboBridge::data_mutex;

static homerobot_MotorMoveCommand current_motor_cmd = homerobot_MotorMoveCommand_init_default;

bool GazeboBridge::init() {
    k_mutex_init(&data_mutex);

    sock_tx_ = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_tx_ < 0) {
        LOG_ERR("Failed to create TX socket: %d", errno);
        return false;
    }

    sock_rx_ = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_rx_ < 0) {
        LOG_ERR("Failed to create RX socket: %d", errno);
        return false;
    }

    memset(&addr_tx_, 0, sizeof(addr_tx_));
    addr_tx_.sin_family = AF_INET;
    addr_tx_.sin_port = htons(6005);
    zsock_inet_pton(AF_INET, "127.0.0.1", &addr_tx_.sin_addr);

    memset(&addr_rx_, 0, sizeof(addr_rx_));
    addr_rx_.sin_family = AF_INET;
    addr_rx_.sin_port = htons(6006);
    addr_rx_.sin_addr.s_addr = INADDR_ANY;

    if (zsock_bind(sock_rx_, (struct sockaddr *)&addr_rx_, sizeof(addr_rx_)) < 0) {
        LOG_ERR("Failed to bind RX socket: %d", errno);
        return false;
    }

    LOG_INF("GazeboBridge initialized: TX port 6005, RX port 6006");
    return true;
}

void GazeboBridge::start() {
    k_thread_create(&bridge_thread_data, bridge_stack,
                    K_THREAD_STACK_SIZEOF(bridge_stack),
                    (k_thread_entry_t)bridge_thread, NULL, NULL, NULL,
                    K_PRIO_COOP(7), 0, K_NO_WAIT);
}

void GazeboBridge::send_motor_cmd(const char* name, int direction, uint8_t power) {
    if (sock_tx_ < 0) return;

    k_mutex_lock(&data_mutex, K_FOREVER);
    
    // Convert direction to angle for Protobuf (1.0 for forward, -1.0 for backward)
    float angle = (direction == (int)Direction::BACKWARD) ? -1.0f : 1.0f;

    if (strcmp(name, "SX") == 0) {
        current_motor_cmd.left_power = power;
        current_motor_cmd.left_angle = angle;
    } else if (strcmp(name, "DX") == 0) {
        current_motor_cmd.right_power = power;
        current_motor_cmd.right_angle = angle;
    }

    uint8_t buffer[128];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    
    if (pb_encode(&stream, homerobot_MotorMoveCommand_fields, &current_motor_cmd)) {
        zsock_sendto(sock_tx_, buffer, stream.bytes_written, 0, (struct sockaddr *)&addr_tx_, sizeof(addr_tx_));
    }
    k_mutex_unlock(&data_mutex);
}

int32_t GazeboBridge::get_virtual_ticks(uint8_t unit_idx) {
    k_mutex_lock(&data_mutex, K_FOREVER);
    int32_t val = ticks_[unit_idx % 2];
    k_mutex_unlock(&data_mutex);
    return val;
}

void GazeboBridge::get_virtual_imu(float* accel, float* gyro) {
    k_mutex_lock(&data_mutex, K_FOREVER);
    memcpy(accel, accel_, sizeof(accel_));
    memcpy(gyro, gyro_, sizeof(gyro_));
    k_mutex_unlock(&data_mutex);
}

bool GazeboBridge::get_virtual_lidar(homerobot_LidarScan* scan) {
    k_mutex_lock(&data_mutex, K_FOREVER);
    if (!new_scan_available_) {
        k_mutex_unlock(&data_mutex);
        return false;
    }
    memcpy(scan, &last_scan_, sizeof(homerobot_LidarScan));
    new_scan_available_ = false;
    k_mutex_unlock(&data_mutex);
    return true;
}

void GazeboBridge::bridge_thread() {
    // Large buffer for Lidar scans (8KB to be safe for 180+ points)
    static uint8_t buffer[8192]; 
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    while (true) {
        int len = zsock_recvfrom(sock_rx_, buffer, sizeof(buffer), 0, (struct sockaddr *)&client_addr, &addr_len);
        if (len > 0) {
            homerobot_Telemetry telemetry = homerobot_Telemetry_init_default;
            pb_istream_t stream = pb_istream_from_buffer(buffer, len);

            if (pb_decode(&stream, homerobot_Telemetry_fields, &telemetry)) {
                k_mutex_lock(&data_mutex, K_FOREVER);
                ticks_[0] = telemetry.encoder_left;
                ticks_[1] = telemetry.encoder_right;
                if (telemetry.has_imu) {
                    accel_[0] = telemetry.imu.acceleration.x;
                    accel_[1] = telemetry.imu.acceleration.y;
                    accel_[2] = telemetry.imu.acceleration.z;
                    gyro_[0] = telemetry.imu.gyroscope.x;
                    gyro_[1] = telemetry.imu.gyroscope.y;
                    gyro_[2] = telemetry.imu.gyroscope.z;
                }
                if (telemetry.has_lidar) {
                    memcpy(&last_scan_, &telemetry.lidar, sizeof(homerobot_LidarScan));
                    new_scan_available_ = true;
                }
                k_mutex_unlock(&data_mutex);
            }
        }
        k_sleep(K_MSEC(10));
    }
}

#endif
