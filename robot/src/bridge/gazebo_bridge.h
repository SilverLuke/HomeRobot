#pragma once

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <messages.pb.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)

class GazeboBridge {
public:
    static bool init();
    static void start();

    // Outbound: Send motor commands to Gazebo
    static void send_motor_cmd(const char* name, int direction, uint8_t power);

    // Inbound: Get latest sensor data from Gazebo
    static int32_t get_virtual_ticks(uint8_t unit_idx);
    static void get_virtual_imu(float* accel, float* gyro);
    static bool get_virtual_lidar(homerobot_LidarScan* scan);

private:
    static void bridge_thread();
    
    static int sock_tx_;
    static int sock_rx_;
    static struct sockaddr_in addr_tx_;
    static struct sockaddr_in addr_rx_;

    static struct k_thread bridge_thread_data;

    static int32_t ticks_[2];
    static float accel_[3];
    static float gyro_[3];
    static homerobot_LidarScan last_scan_;
    static bool new_scan_available_;
    static struct k_mutex data_mutex;
};

#endif
