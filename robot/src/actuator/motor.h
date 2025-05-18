#pragma once
#include <cstdint>

void motor_move(const uint8_t* data);

uint8_t extract_motor_config(
  const uint8_t* data,
  float* kp,
  float* ki,
  float* kd,
  uint8_t* upper_limit,
  uint8_t offset
  );

void motor_config(const uint8_t* data);

