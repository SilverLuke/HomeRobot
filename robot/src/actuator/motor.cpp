#include "motor.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <math.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

Motor::Motor(const char* name, 
             const struct pwm_dt_spec* fwd_pwm, 
             const struct pwm_dt_spec* bwd_pwm,
             Encoders* encoder,
             uint8_t lower_limit, 
             uint8_t upper_limit)
    : name_(name), fwd_pwm_(fwd_pwm), bwd_pwm_(bwd_pwm), encoder_(encoder),
      upper_limit_(upper_limit), lower_limit_(lower_limit) {
}

void Motor::init(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    if (!pwm_is_ready_dt(fwd_pwm_) || !pwm_is_ready_dt(bwd_pwm_)) {
        LOG_ERR("PWM device for motor %s not ready", name_);
        return;
    }

    LOG_INF("Motor %s: Initialized (KP: %.2f, KI: %.2f, KD: %.2f)", name_, kp_, ki_, kd_);
    direction_ = BRAKE;
    set_motor(BRAKE, 0);
}

void Motor::set_motor(Direction dir, uint8_t pwm_val) {
    uint32_t pulse = (uint32_t)((float)pwm_val / 255.0f * (float)fwd_pwm_->period);
    
    switch (dir) {
        case FORWARD:
            pwm_set_pulse_dt(fwd_pwm_, pulse);
            pwm_set_pulse_dt(bwd_pwm_, 0);
            break;
        case BACKWARD:
            pwm_set_pulse_dt(fwd_pwm_, 0);
            pwm_set_pulse_dt(bwd_pwm_, pulse);
            break;
        case BRAKE:
            pwm_set_pulse_dt(fwd_pwm_, 0);
            pwm_set_pulse_dt(bwd_pwm_, 0);
            break;
        case FREE:
            pwm_set_pulse_dt(fwd_pwm_, 0);
            pwm_set_pulse_dt(bwd_pwm_, 0);
            break;
    }
    power_ = pwm_val;
}

void Motor::loop() {
    uint64_t current_time = k_uptime_get();
    int32_t current_position = read_encoder();

    uint64_t delta_time = current_time - previous_timestamp_;
    if (delta_time == 0) delta_time = 1; // Prevent division by zero
    previous_timestamp_ = current_time;

    if (direction_ == FREE) {
        return;
    }

    double control_signal = calculate_pid_signal(current_position, delta_time);
    power_ = limit_power(control_signal);

    int32_t position_error = current_position - target_;

    if (abs(position_error) <= POSITION_TOLERANCE) {
        direction_ = BRAKE;
        power_ = 0;
        set_motor(direction_, power_);
        target_is_reached_ = true;
    } else {
        direction_ = (control_signal < 0) ? BACKWARD : FORWARD;
        set_motor(direction_, power_);
        target_is_reached_ = false;
    }
}

double Motor::calculate_pid_signal(int32_t current_position, uint64_t delta_time_ms) {
    double error = (double)target_ - (double)current_position;
    double error_derivative = (error - previous_error_) / (double)delta_time_ms;
    previous_error_ = error;

    eintegral_ += error * ((double)delta_time_ms / 1000.0);

    return (kp_ * error) + (ki_ * eintegral_) + (kd_ * error_derivative);
}

uint8_t Motor::limit_power(double power_signal) const {
    double abs_power = fabs(power_signal);
    if (abs_power > 2) {
        if (abs_power > upper_limit_) return upper_limit_;
        if (abs_power < lower_limit_) return lower_limit_;
        return (uint8_t)abs_power;
    }
    return 0;
}

int32_t Motor::read_encoder() {
    if (encoder_) {
        position_ = encoder_->get_total_ticks();
    }
    return position_;
}

void Motor::turn_on(Direction dir) {
    direction_ = dir;
}

void Motor::turn_off() {
    direction_ = FREE;
    set_motor(FREE, 0);
}

void Motor::set_position(int32_t pos) {
    position_ = pos;
}

int32_t Motor::get_position() {
    return read_encoder();
}

void Motor::set_target(int32_t target) {
    target_ = target;
    eintegral_ = 0;
    previous_error_ = 0;
    previous_timestamp_ = k_uptime_get();
    target_is_reached_ = false;
}

bool Motor::target_reached(bool reset) {
    bool reached = target_is_reached_;
    if (reset) target_is_reached_ = false;
    return reached;
}

void Motor::config_set_limit(uint8_t lower_limit, uint8_t upper_limit) {
    lower_limit_ = lower_limit;
    upper_limit_ = upper_limit;
}

void Motor::config_set_pid(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    previous_error_ = 0;
    eintegral_ = 0;
}
