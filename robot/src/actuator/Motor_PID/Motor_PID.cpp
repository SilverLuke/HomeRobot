#include "Motor_PID.h"

#include "definitions.h"

#include <Elog.h>
#include <Encoder.h>
#include <sys/_intsup.h>

Motor::Motor(String name, Encoder* encoder, const uint8_t in1, const uint8_t in2,
             const uint8_t pwmPin, const uint8_t lowerLimit, const uint8_t upperLimit) {
  this->name = name;
  this->encoder = encoder;
  this->in1 = in1;
  this->in2 = in2;
  this->pwmpin = pwmPin;
  this->lower_limit = lowerLimit;
  this->upper_limit = upperLimit;
  this->set_position(0);
}

void Motor::init(const double kp, const double ki, const double kd) {
  this->kp = kp;
  this->kd = kd;
  this->ki = ki;
  if (pwmpin != 0) {
    pinMode(pwmpin, OUTPUT);
  }
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  this->direction = FORWARD;
}

void Motor::config_set_pid(const double kp, const double ki, const double kd) {
  this->kp = kp;
  this->kd = kd;
  this->ki = ki;
  this->previous_error = 0;
  this->eintegral = 0;
  this->previus_timestamp = 0;
}

void Motor::set_motor(const Direction dir, const uint8_t pwmVal) const {

  if (pwmpin != 0) {
    set_motor_with_pwm_pin(dir, pwmVal);
  } else {
    set_motor_without_pwm_pin(dir, pwmVal);
  }
}

void Motor::set_motor_with_pwm_pin(const Direction dir, const uint8_t pwmValue) const {
  analogWrite(pwmpin, pwmValue);

  switch (dir) {
    case FORWARD:
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      break;
    case BACKWARD:
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      break;
    default:
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      break;
  }
}

void Motor::set_motor_without_pwm_pin(const Direction dir,
                                      const uint8_t pwmValue) const {
  switch (dir) {
    case FORWARD:
      analogWrite(in1, pwmValue);
      analogWrite(in2, LOW);
      break;
    case BACKWARD:
      analogWrite(in1, LOW);
      analogWrite(in2, pwmValue);
      break;
    case BRAKE:
      analogWrite(in1, LOW);
      analogWrite(in2, LOW);
      break;
    case FREE:
      analogWrite(in1, LOW);
      analogWrite(in2, LOW);
      break;
    default:
      break;
  }
}

void Motor::loop() {
  const unsigned long current_time = millis();
  const long current_position = this->read_encoder();

  // Calculate time delta in seconds
  const unsigned long delta_time = current_time - previus_timestamp;
  previus_timestamp = current_time;

  // Calculate PID control signal
  const double control_signal =
      calculate_pid_signal(current_position, delta_time);

  // Convert control signal to motor commands
  this->power = limit_power(std::fabs(control_signal));

  // Update motor state based on position error
  const long position_error = current_position - target;

  if (current_time - this->last_debug_time >= 1000) {
    this->last_debug_time = current_time;
    Logger.debug(MOTOR_LOGGER, this->print_state().c_str());
  }

  // If the direction is FREE, do nothing
  if (this->direction == FREE) {
    // TODO refactor this, I don't know if this is needed
    // this->power = 0;
    //this->set_motor(FREE, this->power);
    return;
  }

  if (std::abs(position_error) <= POSITION_TOLERANCE) {
    this->direction = BRAKE;
    this->power = 0;
    this->set_motor(this->direction, this->power);
    this->target_is_reached = true;
  } else {
    this->direction = signbit(control_signal) ? BACKWARD : FORWARD;
    this->set_motor(direction, this->power);
    this->target_is_reached = false;
  }
}

/**
 * @brief
 * @param current_position
 * @param delta_time Delta time in ms
 * @return
 */
double Motor::calculate_pid_signal(const long current_position,
                                   const unsigned long delta_time) {
  const long error = this->target - current_position;
  const double error_derivative =
      static_cast<double>(error - this->previous_error) / delta_time;
  this->previous_error = error;

  this->eintegral = this->eintegral + error * (delta_time / 1000.);

  this->ekp = this->kp * static_cast<double>(error);
  this->eki = this->ki * this->eintegral;
  this->ekd = this->kd * error_derivative;
  return ekp + eki + ekd;
}

/**
 * @brief Limit the PID error to the max and min power to the motor
 * @param power The abs value of the PID error control
 * @return The power to send to the motor in interval [0; [lower_limit,
 * upper_limit]]
 */
uint8_t Motor::limit_power(const double power) const {
  if (fabs(power) > 2) {
    return constrain(power, this->lower_limit, this->upper_limit);
  }
  return 0;
}

long Motor::read_encoder() {
  this->position = this->encoder->read() / 2;
  return this->position;
}

void Motor::turn_on(const Direction dir) {
  this->direction = dir;
}

void Motor::turn_off() {
  this->direction = FREE;
  this->set_motor(FREE, 0);
}

void Motor::set_position(const int32_t pos) {
  this->position = pos;
  this->encoder->write(this->position * 2);
}

int32_t Motor::get_position(const bool force_update) {
  if (force_update)
    this->read_encoder();
  return this->position;
}

void Motor::set_target(const int32_t target) {
  this->target = target;
  // PID
  this->eintegral = 0;
  this->previous_error = 0;
  this->previus_timestamp = millis();

  this->target_is_reached = false;
}

int32_t Motor::get_target() const { return target; }

void Motor::config_set_limit(const int lower_limit, const int upper_limit) {
  this->lower_limit = lower_limit;
  this->upper_limit = upper_limit;
}

bool Motor::target_reached(const bool reset) {
  if (reset) {
    this->target_is_reached = false;
  }
  return this->target_is_reached;
}

String Motor::print_state() const {
  return name +
    " Position: " + String(position) + \
    " Target: " + String(target) + \
    " Reached: " + String(target_is_reached) + \
    " Error: " + String(previous_error) + \
    " Ekp: " + String(ekp) + \
    " Eki: " + String(eki) + \
    " Ekd: " + String(ekd) + \
    " Direction: " + DirectionToString(direction) + \
    " Power: " + String(power);
}
