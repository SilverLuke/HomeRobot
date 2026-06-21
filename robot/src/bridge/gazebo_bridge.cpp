#include "gazebo_bridge.h"
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>
#include "../actuator/motor.h"

#if defined(CONFIG_BOARD_NATIVE_SIM)

LOG_MODULE_REGISTER(gazebo_bridge, LOG_LEVEL_INF);

// Static member definitions
gz::transport::Node GazeboBridge::node_;
gz::transport::Node::Publisher GazeboBridge::pub_left_;
gz::transport::Node::Publisher GazeboBridge::pub_right_;
MotorModel GazeboBridge::motor_l_;
MotorModel GazeboBridge::motor_r_;

int32_t GazeboBridge::ticks_[2] = {0, 0};
float GazeboBridge::accel_[3] = {0, 0, 0};
float GazeboBridge::gyro_[3] = {0, 0, 0};
homerobot_LidarScan GazeboBridge::last_scan_ = homerobot_LidarScan_init_default;
bool GazeboBridge::new_scan_available_ = false;
std::mutex GazeboBridge::data_mutex_;

static K_THREAD_STACK_DEFINE(bridge_stack, 16384);
struct k_thread GazeboBridge::bridge_thread_data;

bool GazeboBridge::init() {

    // Initialize Publishers (Capitalized for Gazebo Sim API)
    pub_left_ = node_.Advertise<gz::msgs::Double>("/model/homerobot/joint/left_wheel_joint/cmd_force");
    pub_right_ = node_.Advertise<gz::msgs::Double>("/model/homerobot/joint/right_wheel_joint/cmd_force");

    // Initialize Subscribers
    if (!node_.Subscribe("/model/homerobot/joint_state", &GazeboBridge::on_joint_state)) {
        LOG_ERR("Failed to subscribe to joint_state");
        return false;
    }
    if (!node_.Subscribe("/model/homerobot/imu", &GazeboBridge::on_imu)) {
        LOG_ERR("Failed to subscribe to imu");
        return false;
    }
    if (!node_.Subscribe("/model/homerobot/lidar", &GazeboBridge::on_lidar)) {
        LOG_ERR("Failed to subscribe to lidar");
        return false;
    }

    LOG_INF("Direct C++ Gazebo Link Initialized (0ms latency mode)");
    return true;
}

void GazeboBridge::start() {
    k_thread_create(&bridge_thread_data, bridge_stack,
                    K_THREAD_STACK_SIZEOF(bridge_stack),
                    (k_thread_entry_t)bridge_thread, NULL, NULL, NULL,
                    K_PRIO_COOP(7), 0, K_NO_WAIT);
}

void GazeboBridge::send_motor_cmd(const char* name, int direction, uint8_t power) {
    double p = (double)power / 255.0;
    if (direction == (int)Direction::BACKWARD) p = -p;
    
    LOG_INF("Bridge Motor Cmd: %s Dir=%d Pwr=%u -> p=%.4f", name, direction, power, p);

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (strcmp(name, "SX") == 0) {
            motor_l_.set_power(p);
        } else if (strcmp(name, "DX") == 0) {
            motor_r_.set_power(p);
        }
    }
}

static uint32_t joint_state_recv_count = 0;
static uint32_t imu_recv_count = 0;
static uint32_t lidar_recv_count = 0;

void GazeboBridge::on_joint_state(const gz::msgs::Model &msg) {
    const double ticks_per_radian = 360.0 / (2.0 * M_PI);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    joint_state_recv_count++;
    for (int i = 0; i < msg.joint_size(); ++i) {
        const auto &joint = msg.joint(i);
        if (joint.name() == "left_wheel_joint") {
            ticks_[0] = (int32_t)(joint.axis1().position() * ticks_per_radian);
            motor_l_.set_velocity(joint.axis1().velocity());
        } else if (joint.name() == "right_wheel_joint") {
            ticks_[1] = (int32_t)(joint.axis1().position() * ticks_per_radian);
            motor_r_.set_velocity(joint.axis1().velocity());
        }
    }
}

void GazeboBridge::on_imu(const gz::msgs::IMU &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    imu_recv_count++;
    accel_[0] = msg.linear_acceleration().x();
    accel_[1] = msg.linear_acceleration().y();
    accel_[2] = msg.linear_acceleration().z();
    gyro_[0] = msg.angular_velocity().x();
    gyro_[1] = msg.angular_velocity().y();
    gyro_[2] = msg.angular_velocity().z();
}

void GazeboBridge::on_lidar(const gz::msgs::LaserScan &msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    lidar_recv_count++;
    
    last_scan_.points_count = 0;
    int count = msg.ranges_size();
    int stride = 1;
    if (count > 180) {
        stride = count / 180;
        count = 180;
    }

    double angle_min = msg.angle_min();
    double angle_step = msg.angle_step();

    for (int i = 0; i < count; ++i) {
        int original_idx = i * stride;
        double dist = msg.ranges(original_idx);
        if (std::isinf(dist)) dist = 0;

        double angle_rad = angle_min + (original_idx * angle_step);
        double angle_deg = angle_rad * (180.0 / M_PI);
        while (angle_deg < 0) angle_deg += 360.0;
        while (angle_deg >= 360.0) angle_deg -= 360.0;
        

        last_scan_.points[i].distance_mm = (float)(dist * 1000.0);
        last_scan_.points[i].angle_deg = (float)angle_deg;
        last_scan_.points[i].quality = 15;
        last_scan_.points[i].scan_completed = (i == count - 1);
    }
    last_scan_.points_count = count;
    new_scan_available_ = true;
}

void GazeboBridge::bridge_thread() {
    LOG_INF("Direct Control Loop started (100Hz)");
    uint32_t loop_count = 0;
    
    while (true) {
        double torque_l, torque_r;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            
            // Calculate physics-accurate torque for both motors
            torque_l = motor_l_.calculate_torque();
            torque_r = motor_r_.calculate_torque();
        }
        
        if (std::abs(torque_l) > 0.001 || std::abs(torque_r) > 0.001) {
            LOG_INF("Torque: L=%.4f R=%.4f", torque_l, torque_r);
        }
        
        // Publish directly to Gazebo (Capitalized for Gazebo Sim API)
        gz::msgs::Double msg_l, msg_r;
        msg_l.set_data(torque_l);
        msg_r.set_data(torque_r);
        
        pub_left_.Publish(msg_l);
        pub_right_.Publish(msg_r);
        
        loop_count++;
        if (loop_count >= 100) {
            LOG_INF("[BRIDGE DEBUG] Messages received - JointState: %u, IMU: %u, Lidar: %u",
                    joint_state_recv_count, imu_recv_count, lidar_recv_count);
            loop_count = 0;
        }
        
        k_sleep(K_MSEC(10)); // 100Hz update rate
        k_yield();
    }
}

int32_t GazeboBridge::get_virtual_ticks(uint8_t unit_idx) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    int32_t val = ticks_[unit_idx % 2];
    return val;
}

void GazeboBridge::get_virtual_imu(float* accel, float* gyro) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    memcpy(accel, accel_, sizeof(accel_));
    memcpy(gyro, gyro_, sizeof(gyro_));
}

bool GazeboBridge::get_virtual_lidar(homerobot_LidarScan* scan) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!new_scan_available_) {
        return false;
    }
    memcpy(scan, &last_scan_, sizeof(homerobot_LidarScan));
    new_scan_available_ = false;
    return true;
}

#endif
