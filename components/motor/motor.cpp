#include "motor.h"

#include "esp_log.h"

static const char *TAG = "MOTOR";

static void motor_gpio_output(gpio_num_t pin) {
  gpio_config_t cfg = {};
  cfg.pin_bit_mask = 1ULL << pin;
  cfg.mode = GPIO_MODE_OUTPUT;
  cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&cfg);
}

static esp_err_t motor_ledc_setup(void) {
  ledc_timer_config_t timer = {};
  timer.speed_mode = Config::Motor::LEDC_MODE;
  timer.duty_resolution = Config::Motor::LEDC_RESOLUTION;
  timer.timer_num = Config::Motor::LEDC_TIMER;
  timer.freq_hz = Config::Motor::LEDC_FREQ_HZ;
  timer.clk_cfg = LEDC_AUTO_CLK;
  return ledc_timer_config(&timer);
}

static esp_err_t motor_ledc_attach(gpio_num_t pwm_pin, ledc_channel_t channel) {
  ledc_channel_config_t ch = {};
  ch.gpio_num = pwm_pin;
  ch.speed_mode = Config::Motor::LEDC_MODE;
  ch.channel = channel;
  ch.timer_sel = Config::Motor::LEDC_TIMER;
  ch.duty = 0;
  ch.hpoint = 0;
  return ledc_channel_config(&ch);
}

static esp_err_t motor_set_pwm(ledc_channel_t channel, uint32_t duty) {
  esp_err_t res = ledc_set_duty(Config::Motor::LEDC_MODE, channel, duty);
  if (res != ESP_OK) {
    return res;
  }
  return ledc_update_duty(Config::Motor::LEDC_MODE, channel);
}

esp_err_t motor_init(motor_driver_t *driver, gpio_num_t a_in1, gpio_num_t a_in2,
                     gpio_num_t a_pwm, gpio_num_t b_in1, gpio_num_t b_in2,
                     gpio_num_t b_pwm, gpio_num_t standby) {
  driver->standby_pin = standby;

  driver->motor_a.in1_pin = a_in1;
  driver->motor_a.in2_pin = a_in2;
  driver->motor_a.pwm_pin = a_pwm;
  driver->motor_a.offset = 1;
  driver->motor_a.pwm_channel = LEDC_CHANNEL_0;

  driver->motor_b.in1_pin = b_in1;
  driver->motor_b.in2_pin = b_in2;
  driver->motor_b.pwm_pin = b_pwm;
  driver->motor_b.offset = -1;
  driver->motor_b.pwm_channel = LEDC_CHANNEL_1;

  motor_gpio_output(a_in1);
  motor_gpio_output(a_in2);
  motor_gpio_output(b_in1);
  motor_gpio_output(b_in2);
  motor_gpio_output(standby);

  esp_err_t res = motor_ledc_setup();
  if (res != ESP_OK) {
    return res;
  }
  res = motor_ledc_attach(a_pwm, driver->motor_a.pwm_channel);
  if (res != ESP_OK) {
    return res;
  }
  res = motor_ledc_attach(b_pwm, driver->motor_b.pwm_channel);
  if (res != ESP_OK) {
    return res;
  }

  gpio_set_level(standby, 1);
  motor_coast(&driver->motor_a);
  motor_coast(&driver->motor_b);

  ESP_LOGI(TAG, "TB6612FNG initialized");

  return ESP_OK;
}

esp_err_t motor_drive(motor_t *motor, int speed) {
  speed *= motor->offset;

  if (speed == 0) {
    motor_coast(motor);
    return ESP_OK;
  }

  if (speed > 0) {
    gpio_set_level(motor->in1_pin, 1);
    gpio_set_level(motor->in2_pin, 0);
  } else {
    gpio_set_level(motor->in1_pin, 0);
    gpio_set_level(motor->in2_pin, 1);
    speed = -speed;
  }

  if (speed > Config::Motor::MAX_SPEED) {
    speed = Config::Motor::MAX_SPEED;
  }

  return motor_set_pwm(motor->pwm_channel, (uint32_t)speed);
}

void motor_brake(motor_t *motor) {
  gpio_set_level(motor->in1_pin, 1);
  gpio_set_level(motor->in2_pin, 1);
  motor_set_pwm(motor->pwm_channel, 0);
}

void motor_coast(motor_t *motor) {
  gpio_set_level(motor->in1_pin, 0);
  gpio_set_level(motor->in2_pin, 0);
  motor_set_pwm(motor->pwm_channel, 0);
}

void motor_standby(motor_driver_t *driver) {
  gpio_set_level(driver->standby_pin, 0);
}

void motor_wake(motor_driver_t *driver) {
  gpio_set_level(driver->standby_pin, 1);
}
