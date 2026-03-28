#pragma once

#include "actuator/motor.h"
#include "actuator/status_led.h"
#include "sensors/imu.h"
#include "sensors/battery.h"
#include "sensors/encoders.h"
#include <messages.pb.h>

class Diagnostic {
public:
    Diagnostic(Motor& left_motor, Motor& right_motor, Imu& imu, 
               Battery& battery, StatusLed& led);

    /**
     * @brief Runs the full suite of hardware diagnostics.
     * @return true if all critical tests pass, false otherwise.
     */
    bool run_all();

    /**
     * @brief Runs the diagnostics and returns the result as a Protobuf struct.
     */
    homerobot_DiagnosticResult run_rpc();

private:
    Motor &left_motor_, &right_motor_;
    Imu &imu_;
    Battery &battery_;
    StatusLed &led_;

    void add_check(homerobot_DiagnosticResult& result, const char* name, bool success, const char* msg);

    bool test_battery(homerobot_DiagnosticResult* result = nullptr);
    bool test_imu(homerobot_DiagnosticResult* result = nullptr);
    bool test_motors_and_encoders(homerobot_DiagnosticResult* result = nullptr);
    
    // Helper for individual motor test
    bool test_single_motor(Motor& motor, const char* name, homerobot_DiagnosticResult* result = nullptr);
    
    struct VibrationMetrics {
        float max_acc_vibration = 0; // Max deviation from baseline
        float max_gyro_vibration = 0;
        void reset() { max_acc_vibration = 0; max_gyro_vibration = 0; }
    };

    void sample_vibration(VibrationMetrics& metrics, float baseline_ax, float baseline_ay, float baseline_az);
};
