#include "motor.h"

#include <Elog.h>

#include "../definitions.h"
#include "../utils/utils.h"

void motor_move(const uint8_t* data) {
  const uint8_t dx_power = data[0];

  // Read 4 bytes for float in network byte order
  uint32_t raw_dx_angle = (static_cast<uint32_t>(data[1]) << 24) |
                          (static_cast<uint32_t>(data[2]) << 16) |
                          (static_cast<uint32_t>(data[3]) << 8) |
                          static_cast<uint32_t>(data[4]);
  const float dx_angle = *reinterpret_cast<float*>(&raw_dx_angle);

  const uint8_t sx_power = data[5];

  // Read next 4 bytes for float in network byte order
  uint32_t raw_sx_angle = (static_cast<uint32_t>(data[6]) << 24) |
                          (static_cast<uint32_t>(data[7]) << 16) |
                          (static_cast<uint32_t>(data[8]) << 8) |
                          static_cast<uint32_t>(data[9]);
  const float sx_angle = *reinterpret_cast<float*>(&raw_sx_angle);

  Logger.debug(PROTO_LOGGER, "Motor DX: %u %f, SX: %u %f", dx_power, dx_angle, sx_power, sx_angle);

  if (dx_power == 0) {
    motor_dx->turn_off();
  } else {
    motor_dx->set_motor(signbit(dx_angle) ? BACKWARD : FORWARD, 255);
  }

  if (sx_power == 0) {
    motor_sx->turn_off();
  } else {
    motor_sx->set_motor(signbit(sx_angle) ? BACKWARD : FORWARD, 255);
  }
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