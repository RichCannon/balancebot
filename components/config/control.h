#pragma once

#include "config.h"

// Runtime-tunable control state, shared between the balance loop and the
// optional web UI. Defaults come from Config::Balance.
struct control_state_t {
  bool running = false;
  bool start_pending = false;
  int64_t start_deadline_us = 0;

  bool setpoint_calib_pending = false;
  int64_t setpoint_calib_deadline_us = 0;

  float kp = Config::Balance::KP;
  float ki = Config::Balance::KI;
  float kd = Config::Balance::KD;
  float setpoint_pitch = Config::Balance::SETPOINT_PITCH;
  int min_pwm = Config::Balance::MIN_PWM;
  float max_angle = Config::Balance::MAX_ANGLE;
  float deadband = Config::Balance::DEADBAND;
};

// The single shared instance (defined in control.cpp).
extern control_state_t control;
