#pragma once

#include <zephyr/kernel.h>
#include <messages.pb.h>

#if defined(CONFIG_BOARD_NATIVE_SIM)

// Zephyr defines these macros which collide with standard libraries and Protobuf
#undef EMPTY
#undef OK
#undef ERROR

#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>

/**
 * High-fidelity C++ implementation of the LEGO NXT 9842 Motor Model.
 */
class MotorModel {
public:
    MotorModel(double max_torque = 0.167, double no_load_speed = 17.8)
        : max_torque_(max_torque), no_load_speed_(no_load_speed) {}

    void set_power(double power) { requested_power_ = power; }
    void set_velocity(double velocity) { current_velocity_ = velocity; }

    double calculate_torque() const {
        if (std::abs(requested_power_) < 0.01) {
            // Passive Friction Braking (Opposes current motion)
            if (std::abs(current_velocity_) < 0.05) return 0.0;
            return -1.5 * max_torque_ * (current_velocity_ / no_load_speed_);
        }
        // Normal driving torque with Back-EMF
        return max_torque_ * (requested_power_ - (current_velocity_ / no_load_speed_));
    }

private:
    double max_torque_;
    double no_load_speed_;
    double requested_power_ = 0.0;
    double current_velocity_ = 0.0;
};

/**
 * Direct C++ Link to Gazebo Sim.
 */
class GazeboBridge {
public:
    static bool init();
    static void start();

    // Outbound: Direct Torque Application
    static void send_motor_cmd(const char* name, int direction, uint8_t power);

    // Inbound: Low-latency sensor access
    static int32_t get_virtual_ticks(uint8_t unit_idx);
    static void get_virtual_imu(float* accel, float* gyro);
    static bool get_virtual_lidar(homerobot_LidarScan* scan);

private:
    static void bridge_thread();
    static void on_joint_state(const gz::msgs::Model &msg);
    static void on_imu(const gz::msgs::IMU &msg);
    static void on_lidar(const gz::msgs::LaserScan &msg);

    static gz::transport::Node node_;
    static gz::transport::Node::Publisher pub_left_;
    static gz::transport::Node::Publisher pub_right_;

    static MotorModel motor_l_;
    static MotorModel motor_r_;

    static struct k_thread bridge_thread_data;
    static int32_t ticks_[2];
    static float accel_[3];
    static float gyro_[3];
    static homerobot_LidarScan last_scan_;
    static bool new_scan_available_;
    static struct k_mutex data_mutex;
};

#endif
