#pragma once

#include "config.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdint.h>

// One motor (one half of the TB6612FNG H-bridge).
typedef struct {
  gpio_num_t in1_pin;
  gpio_num_t in2_pin;
  gpio_num_t pwm_pin;
  int8_t offset;              // +1 or -1: flips direction (mounting compensation)
  ledc_channel_t pwm_channel; // LEDC channel assigned to this motor
} motor_t;

// A whole TB6612FNG chip: two motors + a shared standby pin.
typedef struct {
  gpio_num_t standby_pin;
  motor_t motor_a;
  motor_t motor_b;
} motor_driver_t;

// Configure both motors + standby and set up the LEDC PWM timer/channels.
esp_err_t motor_init(motor_driver_t *driver, gpio_num_t a_in1, gpio_num_t a_in2,
                     gpio_num_t a_pwm, gpio_num_t b_in1, gpio_num_t b_in2,
                     gpio_num_t b_pwm, gpio_num_t standby);

// Drive one motor: speed ±Config::Motor::MAX_SPEED (sign = direction, magnitude = duty).
esp_err_t motor_drive(motor_t *motor, int speed);

// Short-brake (fast stop): IN1 = IN2 = HIGH.
void motor_brake(motor_t *motor);

// Coast (free spin): IN1 = IN2 = LOW.
void motor_coast(motor_t *motor);

// Put the whole chip in standby (STBY = LOW): both motors off.
void motor_standby(motor_driver_t *driver);

// Wake the chip from standby (STBY = HIGH).
void motor_wake(motor_driver_t *driver);
