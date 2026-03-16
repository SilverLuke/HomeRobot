#pragma once

#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

enum Direction : uint8_t {
  FORWARD = 1,
  BACKWARD = 2,
  BRAKE = 3,
  FREE = 4,
};

class Motor {
 public:
  Motor(const char* name, 
        const struct pwm_dt_spec* fwd_pwm, 
        const struct pwm_dt_spec* bwd_pwm,
        const struct device* encoder_dev,
        uint8_t unit_idx,
        uint8_t lower_limit = 50, 
        uint8_t upper_limit = 255);

  void init(double kp, double ki, double kd);
  void loop();

  void turn_on(Direction dir = FORWARD);
  void turn_off();

  void set_position(int32_t pos);
  int32_t get_position();
  void set_target(int32_t target);
  int32_t get_target() const { return target_; }
  bool target_reached(bool reset = false);

  void config_set_limit(uint8_t lower_limit, uint8_t upper_limit);
  void config_set_pid(double kp, double ki, double kd);
  
  void set_motor(Direction dir, uint8_t pwm_val);

private:
  const char* name_;
  const struct pwm_dt_spec* fwd_pwm_;
  const struct pwm_dt_spec* bwd_pwm_;
  const struct device* encoder_dev_;
  uint8_t unit_idx_;

  uint8_t upper_limit_;
  uint8_t lower_limit_;

  uint64_t previous_timestamp_{0};
  int32_t target_{0};
  int32_t position_{0};

  double previous_error_{0};
  double eintegral_{0};
  bool target_is_reached_{false};

  double kp_{0}, ki_{0}, kd_{0};
  Direction direction_{BRAKE};
  uint8_t power_{0};

  static constexpr uint32_t PWM_PERIOD = 50000; // 20kHz in nanoseconds (1/20000 * 1e9)
  static constexpr int32_t POSITION_TOLERANCE = 2;

  int32_t read_encoder();
  double calculate_pid_signal(int32_t current_position, uint64_t delta_time_ms);
  uint8_t limit_power(double power) const;
};
