#include "motor.h"
#include <zephyr/logging/log.h>
#include <math.h>
#include <soc/pcnt_struct.h>

LOG_MODULE_REGISTER(motor, LOG_LEVEL_DBG);

Motor::Motor(const char* name, 
             const struct pwm_dt_spec* fwd_pwm, 
             const struct pwm_dt_spec* bwd_pwm,
             Encoders* encoder,
             uint8_t lower_limit, 
             uint8_t upper_limit)
    : name_(name), 
      fwd_pwm_(fwd_pwm), 
      bwd_pwm_(bwd_pwm), 
      encoder_(encoder),
      upper_limit_(upper_limit), 
      lower_limit_(lower_limit) {}

void Motor::init(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    if (!pwm_is_ready_dt(fwd_pwm_)) {
        LOG_ERR("FWD PWM device %s not ready", fwd_pwm_->dev->name);
    }
    if (!pwm_is_ready_dt(bwd_pwm_)) {
        LOG_ERR("BWD PWM device %s not ready", bwd_pwm_->dev->name);
    }
    if (encoder_ && !encoder_->init()) {
        LOG_ERR("Encoder initialization failed");
    }

    set_motor(BRAKE, 0);
}

void Motor::config_set_pid(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    previous_error_ = 0;
    eintegral_ = 0;
    previous_timestamp_ = k_uptime_get();
}

void Motor::set_motor(Direction dir, uint8_t pwm_val) {
    uint32_t pulse = (uint32_t)((uint64_t)PWM_PERIOD * pwm_val / 255);

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
            pwm_set_pulse_dt(fwd_pwm_, PWM_PERIOD); // Both high for brake
            pwm_set_pulse_dt(bwd_pwm_, PWM_PERIOD);
            break;
        case FREE:
        default:
            pwm_set_pulse_dt(fwd_pwm_, 0);
            pwm_set_pulse_dt(bwd_pwm_, 0);
            break;
    }
    direction_ = dir;
    power_ = pwm_val;
}

void Motor::loop() {
    uint64_t current_time = k_uptime_get();
    int32_t current_position = read_encoder();

    if (previous_timestamp_ == 0) {
        previous_timestamp_ = current_time;
        return;
    }

    uint64_t delta_time_ms = current_time - previous_timestamp_;
    if (delta_time_ms == 0) return;

    previous_timestamp_ = current_time;

    double control_signal = calculate_pid_signal(current_position, delta_time_ms);
    power_ = limit_power(fabs(control_signal));

    int32_t error = target_ - current_position;

    if (direction_ == FREE) {
        return;
    }

    if (abs(error) <= POSITION_TOLERANCE) {
        set_motor(BRAKE, 0);
        target_is_reached_ = true;
    } else {
        Direction dir = (control_signal >= 0) ? FORWARD : BACKWARD;
        set_motor(dir, power_);
        target_is_reached_ = false;
    }
}

double Motor::calculate_pid_signal(int32_t current_position, uint64_t delta_time_ms) {
    double error = (double)target_ - current_position;
    double delta_time_s = (double)delta_time_ms / 1000.0;
    
    double error_derivative = (error - previous_error_) / delta_time_s;
    previous_error_ = error;

    eintegral_ += error * delta_time_s;

    return (kp_ * error) + (ki_ * eintegral_) + (kd_ * error_derivative);
}

uint8_t Motor::limit_power(double power) const {
    if (fabs(power) < 2.0) return 0;
    
    if (power > upper_limit_) return upper_limit_;
    if (power < lower_limit_) return 0; 
    
    return (uint8_t)power;
}

int32_t Motor::read_encoder() {
    if (!encoder_) return position_;

    position_ = encoder_->get_ticks();
    return position_;
}

void Motor::turn_on(Direction dir) {
    direction_ = dir;
}

void Motor::turn_off() {
    set_motor(FREE, 0);
}

void Motor::set_position(int32_t pos) {
    position_ = pos;
    if (pos == 0 && encoder_) {
        encoder_->reset();
    }
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
