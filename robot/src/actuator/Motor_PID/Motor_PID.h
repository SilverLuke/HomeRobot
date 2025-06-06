#pragma once

#include <Arduino.h>
#include <Encoder.h>

enum Direction : uint8_t {
  FORWARD = 1,
  BACKWARD = 2,
  BRAKE = 3,
  FREE = 4,
};

inline const char* DirectionToString(Direction dir) {
  switch (dir) {
    case FORWARD:
      return "FORWARD";
    case BACKWARD:
      return "BACKWARD";
    case BRAKE:
      return "BRAKE";
    case FREE:
      return "FREE";
    default:
      return "UNKNOWN";
  }
}

class Motor {
 public:
  Motor() = default;
  Motor(String name, Encoder* encoder, uint8_t in1, uint8_t in2, uint8_t pwmPin = 0,
        uint8_t lowerLimit = 50, uint8_t upperLimit = 255);
  ~Motor() = default;

  /**
   * @brief place this in the main loop without delays
   */
  void loop();
  /**
   * @brief Initialize some part of the motor
   * @param kp[in] Set the KP
   * @param ki[in] Set the KI
   * @param kd[in] Set the KD
   */
  void init(double kp, double ki, double kd);
  // changes motor_state variable to true
  void turn_on(Direction dir = FORWARD);
  // changes motor_state variable to false
  void turn_off();
  void set_position(int32_t pos);
  int32_t get_position(bool force_update = false);
  void set_target(int32_t target);
  [[nodiscard]] int32_t get_target() const;
  /**
   * @brief check if target position is reached by motor
   * @param reset
   * @return
   */
  bool target_reached(bool reset = false);
  /**
   * @brief
   * @param lower_limit Minimal motor power?
   * @param upper_limit Max motor power?
   */
  void config_set_limit(int lower_limit, int upper_limit);
  void config_set_pid(double kp, double ki, double kd);
  /**
   * @brief
   * @param dir Movement direction
   * @param pwmVal How fast to move
   */
  void set_motor(Direction dir, uint8_t pwmVal) const;

  String print_state() const;

 private:
  static constexpr double MAX_POWER = 255.0;
  static constexpr long POSITION_TOLERANCE = 2;

  uint8_t in2{}, in1{}, pwmpin = 0;
  uint8_t upper_limit = 0, lower_limit = 0;

  unsigned long previus_timestamp{0};
  int32_t target = 0;

  long previous_error = 0;
  double eintegral = 0;
  bool target_is_reached = false;

  double kp = 0.0f, kd = 0.0f, ki = 0.0f;  // your pid variables
  int32_t position = 0;                 // position of rotary encoder \ number of pulses
  Direction direction = BRAKE;  // this variable let the pwm signal go
  Encoder* encoder{};

  // Debug variables
  double ekp = 0, eki = 0, ekd = 0;
  uint8_t power = 0;
  String name;
  unsigned long last_debug_time = 0;

  long read_encoder();
  double calculate_pid_signal(long current_position, unsigned long delta_time);
  uint8_t limit_power(double power) const;

  void set_motor_with_pwm_pin(Direction dir, uint8_t pwmValue) const;
  void set_motor_without_pwm_pin(Direction dir, uint8_t pwmValue) const;
};
