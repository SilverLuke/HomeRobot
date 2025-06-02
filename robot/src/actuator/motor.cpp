#include "motor.h"

#include "../definitions.h"
#include "../utils/utils.h"

void motor_move(const uint8_t* data) {
  const int8_t dx_power = static_cast<int8_t>(data[0]);

  uint32_t raw_dx_angle;
  memcpy(&raw_dx_angle, &data[1], sizeof(uint32_t));
  float dx_angle = net2hostFloat(raw_dx_angle);

  const int8_t sx_power = static_cast<int8_t>(data[5]);

  uint32_t raw_sx_angle;
  memcpy(&raw_sx_angle, &data[6], sizeof(uint32_t));
  float sx_angle = net2hostFloat(raw_sx_angle);

  if (dx_power == 0) {
    motor_dx->turn_off();
  } else {
    motor_dx->set_target(100.0f * dx_power);
  }

  if (sx_power == 0) {
    motor_sx->turn_off();
  } else {
    motor_sx->set_target(100.0f * sx_power);
  }
  log_d("DX: %d, %f   SX: %d, %f", dx_power, dx_angle, sx_power, sx_angle);
}

uint8_t extract_motor_config(const uint8_t* data, float* kp, float* ki,
                             float* kd, uint8_t* upper_limit, uint8_t offset) {
  memcpy(kp, data, sizeof(float));
  offset += sizeof(float);
  memcpy(ki, data + offset, sizeof(float));
  offset += sizeof(float);
  memcpy(kd, data + offset, sizeof(float));
  offset += sizeof(float);
  memcpy(upper_limit, data + offset, sizeof(uint8_t));
  offset += sizeof(uint8_t);
  return offset;
}

void motor_config(const uint8_t* data) {
  float kp, ki, kd;
  uint8_t upper_limit;
  const uint8_t offset =
      extract_motor_config(data, &kp, &ki, &kd, &upper_limit, 0);
  motor_dx->config_set_pid(kp, ki, kd);
  motor_dx->config_set_limit(50, upper_limit);

  extract_motor_config(data, &kp, &ki, &kd, &upper_limit, offset);
  motor_sx->config_set_pid(kp, ki, kd);
  motor_sx->config_set_limit(50, upper_limit);
}
