#include "diagnostic.h"
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(Diagnostic, LOG_LEVEL_INF);

Diagnostic::Diagnostic(Motor& left_motor, Motor& right_motor, Imu& imu, 
                       Battery& battery, StatusLed& led)
    : left_motor_(left_motor), right_motor_(right_motor), imu_(imu), 
      battery_(battery), led_(led) {}

void Diagnostic::add_check(homerobot_DiagnosticResult& result, const char* name, bool success, const char* msg) {
    if (result.checks_count < 5) {
        homerobot_DiagnosticCheck& check = result.checks[result.checks_count++];
        strncpy(check.name, name, sizeof(check.name) - 1);
        check.name[sizeof(check.name) - 1] = '\0';
        check.success = success;
        strncpy(check.message, msg, sizeof(check.message) - 1);
        check.message[sizeof(check.message) - 1] = '\0';
    }
}

homerobot_DiagnosticResult Diagnostic::run_rpc() {
    homerobot_DiagnosticResult result = homerobot_DiagnosticResult_init_default;
    result.checks_count = 0;
    
    // Stop motors and lidar before test
    left_motor_.turn_off();
    right_motor_.turn_off();

    bool battery_ok = test_battery(&result);
    bool imu_ok = test_imu(&result);
    bool motors_ok = false;

    if (battery_ok) {
        motors_ok = test_motors_and_encoders(&result);
    } else {
        add_check(result, "Motors", false, "Skipped due to low battery");
    }

    result.all_ok = battery_ok && imu_ok && motors_ok;
    return result;
}

bool Diagnostic::run_all() {
    LOG_INF( "--- STARTING HARDWARE DIAGNOSTICS ---");
    
    // 1. LED Test: Visual confirmation
    led_.set_color(0, 0, 50); // Blue for "Testing"
    k_msleep(500);
    led_.set_color(0, 50, 0); // Green for "Starting"
    k_msleep(500);
    led_.set_color(0, 0, 0);

    bool battery_ok = test_battery();
    bool imu_ok = test_imu();
    
    if (!battery_ok) {
        LOG_ERR( "Critical: Battery test failed!");
        led_.set_color(50, 0, 0); // Solid Red
        return false;
    }

    if (!imu_ok) {
        LOG_WRN( "Warning: IMU test failed!");
    }

    bool motors_ok = test_motors_and_encoders();

    if (battery_ok && imu_ok && motors_ok) {
        LOG_INF( "--- DIAGNOSTICS COMPLETED: ALL OK ---");
        led_.set_color(0, 50, 0); // Success Green
        k_msleep(1000);
        return true;
    } else {
        LOG_WRN( "--- DIAGNOSTICS COMPLETED: ERRORS FOUND ---");
        led_.set_color(50, 25, 0); // Warning Orange
        k_msleep(1000);
        return false;
    }
}

bool Diagnostic::test_battery(homerobot_DiagnosticResult* result) {
    uint32_t mv = battery_.get_voltage_mv();
    uint32_t pct = battery_.get_percentage();
    LOG_INF( "Battery: %u mV (%u%%)", mv, pct);

    char msg[128];
    snprintf(msg, sizeof(msg), "Voltage: %u mV, %u%%", mv, pct);

    if (mv < 10000) { 
        LOG_ERR( "Battery voltage dangerously low or not detected!");
        if (result) add_check(*result, "Battery", false, msg);
        return false;
    }
    if (result) add_check(*result, "Battery", true, msg);
    return true;
}

bool Diagnostic::test_imu(homerobot_DiagnosticResult* result) {
    if (!imu_.update()) {
        LOG_ERR( "IMU: Communication failed!");
        if (result) add_check(*result, "IMU", false, "Communication failed");
        return false;
    }

    float ax, ay, az;
    imu_.get_accel(ax, ay, az);
    LOG_INF( "IMU: Accel X: %.2f, Y: %.2f, Z: %.2f", (double)ax, (double)ay, (double)az);

    char msg[128];
    float total_g = sqrt(ax*ax + ay*ay + az*az);
    snprintf(msg, sizeof(msg), "Total G: %.2f", (double)total_g);

    if (total_g < 8.0f || total_g > 12.0f) {
        LOG_WRN( "IMU: Accelerometer reading is outside normal gravity range (%.2f g)", (double)total_g);
        if (result) add_check(*result, "IMU", false, msg);
        return false;
    }
    
    if (result) add_check(*result, "IMU", true, msg);
    return true;
}

bool Diagnostic::test_motors_and_encoders(homerobot_DiagnosticResult* result) {
    LOG_INF( "Starting Motor/Encoder sequential test...");
    
    bool left_ok = test_single_motor(left_motor_, "Left Motor (SX)", result);
    k_msleep(500); // Brief pause between motor tests
    bool right_ok = test_single_motor(right_motor_, "Right Motor (DX)", result);

    return left_ok && right_ok;
}

bool Diagnostic::test_single_motor(Motor& motor, const char* name, homerobot_DiagnosticResult* result) {
    LOG_INF( "Testing %s...", name);
    
    // Get baseline IMU for vibration detection
    imu_.update();
    float bax, bay, baz;
    imu_.get_accel(bax, bay, baz);
    
    VibrationMetrics vib;
    vib.reset();

    motor.set_position(0);
    int32_t start_pos = motor.get_position();
    
    // Forward test
    motor.set_motor(FORWARD, 100);
    for (int i = 0; i < 5; i++) {
        k_msleep(100);
        sample_vibration(vib, bax, bay, baz);
    }
    
    motor.set_motor(BRAKE, 0);
    k_msleep(200);
    int32_t fwd_pos = motor.get_position();
    int32_t fwd_diff = fwd_pos - start_pos;
    
    // Backward test
    motor.set_motor(BACKWARD, 100);
    for (int i = 0; i < 5; i++) {
        k_msleep(100);
        sample_vibration(vib, bax, bay, baz);
    }

    motor.set_motor(BRAKE, 0);
    k_msleep(200);
    int32_t bwd_pos = motor.get_position();
    int32_t bwd_diff = bwd_pos - fwd_pos;
    
    bool fwd_ok = fwd_diff > 10;
    bool bwd_ok = bwd_diff < -10;
    
    // Health assessment based on vibration
    // Values are heuristic: > 5.0g deviation is very high, > 20 rad/s gyro is very high.
    bool vib_ok = vib.max_acc_vibration < 5.0f && vib.max_gyro_vibration < 15.0f;

    char msg[128];
    snprintf(msg, sizeof(msg), "Fwd:%d Bwd:%d Vib:%.1f/%.1f", 
             fwd_diff, bwd_diff, (double)vib.max_acc_vibration, (double)vib.max_gyro_vibration);

    if (!fwd_ok || !bwd_ok) {
        LOG_ERR( "%s: Failed motion test (%s)", name, msg);
        if (result) add_check(*result, name, false, msg);
        return false;
    }

    if (!vib_ok) {
        LOG_WRN( "%s: High vibration detected! (%s)", name, msg);
        // We still pass the test but report the warning in the message
    }

    if (result) add_check(*result, name, true, msg);
    return true;
}

void Diagnostic::sample_vibration(VibrationMetrics& metrics, float bax, float bay, float baz) {
    if (imu_.update()) {
        float ax, ay, az, gx, gy, gz;
        imu_.get_accel(ax, ay, az);
        imu_.get_gyro(gx, gy, gz);
        
        // Deviation from baseline gravity vector
        float dev_acc = sqrtf(powf(ax - bax, 2) + powf(ay - bay, 2) + powf(az - baz, 2));
        float dev_gyro = sqrtf(gx*gx + gy*gy + gz*gz);
        
        if (dev_acc > metrics.max_acc_vibration) metrics.max_acc_vibration = dev_acc;
        if (dev_gyro > metrics.max_gyro_vibration) metrics.max_gyro_vibration = dev_gyro;

        LOG_DBG( "  [Vib Sample] Acc Dev: %.2f Gyro: %.2f", (double)dev_acc, (double)dev_gyro);
    }
}
