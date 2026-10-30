#include "balance.h"

#include "config.h"
#include "control.h"
#include "motor.h"
#include "mpu.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BALANCE";

static mpu_handle_t mpu;
static motor_driver_t motors;

// Hold the robot at its balance position; averages the filtered angle to find
// the balance setpoint, then logs it.
[[maybe_unused]] static void measure_setpoint(void) {
  const int settle_ms = Config::Timing::MEASURE_SETTLE_MS;
  const int samples = Config::Timing::MEASURE_SAMPLES;
  const TickType_t period = pdMS_TO_TICKS(10);

  mpu9250_raw_data_t raw;
  float pitch_sum = 0.0f;
  float roll_sum = 0.0f;

  ESP_LOGI(TAG, "Place robot in balance position. Measuring in %d s...",
           settle_ms / 1000);
  vTaskDelay(pdMS_TO_TICKS(settle_ms));

  for (int i = 0; i < samples; i++) {
    if (mpu_read_sensor(&mpu, &raw) == ESP_OK) {
      mpu_update(&mpu, &raw);
      pitch_sum += mpu.pitch;
      roll_sum += mpu.roll;
    }
    vTaskDelay(period);
  }

  float pitch_sp = pitch_sum / samples;
  float roll_sp = roll_sum / samples;

  ESP_LOGI(TAG, "============= MEASURED SETPOINT =============");
  ESP_LOGI(TAG, "  PITCH = %.2f", pitch_sp);
  ESP_LOGI(TAG, "  ROLL  = %.2f", roll_sp);
  ESP_LOGI(TAG, "==============================================");
  ESP_LOGI(TAG, "Current setpoint: PITCH=%.2f", control.setpoint_pitch);
}

struct pid_state_t {
  float integral;
};

static float pid_compute(pid_state_t *pid, float error, float rate, float dt) {
  float output =
      control.kp * error + control.ki * pid->integral + control.kd * rate;

  // Anti-windup: freeze the integral when the output is saturated, and clamp
  // it so Ki can't cause overshoot after the motors hit their limit.
  bool saturated =
      output > Config::Motor::MAX_SPEED || output < -Config::Motor::MAX_SPEED;
  if (!saturated) {
    pid->integral += error * dt;
  }
  if (pid->integral > Config::Balance::INTEGRAL_MAX) {
    pid->integral = Config::Balance::INTEGRAL_MAX;
  } else if (pid->integral < -Config::Balance::INTEGRAL_MAX) {
    pid->integral = -Config::Balance::INTEGRAL_MAX;
  }

  return output;
}

static void balance_task(void *) {
  mpu9250_raw_data_t raw;
  pid_state_t pid = {};
  const float dt = 1.0f / Config::Control::FREQ_HZ;
  const TickType_t period = pdMS_TO_TICKS(1000 / Config::Control::FREQ_HZ);
  TickType_t last_wake_time = xTaskGetTickCount();
  int log_counter = 0;

  while (1) {
    if (mpu_read_sensor(&mpu, &raw) == ESP_OK) {
      mpu_update(&mpu, &raw);
    }

    // Setpoint calibration: hold the robot, then apply the measured angle.
    if (control.setpoint_calib_pending) {
      motor_coast(&motors.motor_a);
      motor_coast(&motors.motor_b);
      if (esp_timer_get_time() >= control.setpoint_calib_deadline_us) {
        control.setpoint_pitch = mpu.pitch;
        control.setpoint_calib_pending = false;
        ESP_LOGI(TAG, "Setpoint calibrated: %.2f deg", control.setpoint_pitch);
      }
      vTaskDelayUntil(&last_wake_time, period);
      continue;
    }

    // "Start" is requested with a delay to give time to place the robot.
    if (control.start_pending &&
        esp_timer_get_time() >= control.start_deadline_us) {
      control.running = true;
      control.start_pending = false;
      pid = {};
      ESP_LOGI(TAG, "Balance STARTED");
    }

    if (!control.running) {
      motor_coast(&motors.motor_a);
      motor_coast(&motors.motor_b);
      vTaskDelayUntil(&last_wake_time, period);
      continue;
    }

    float error = mpu.pitch - control.setpoint_pitch;

    // Deadband: within ±deadband of the setpoint, don't correct (avoids
    // hunting/jitter around the balance point).
    if (error > -control.deadband && error < control.deadband) {
      motor_coast(&motors.motor_a);
      motor_coast(&motors.motor_b);
      pid.integral = 0.0f;
      vTaskDelayUntil(&last_wake_time, period);
      continue;
    }

    if (error > control.max_angle || error < -control.max_angle) {
      motor_brake(&motors.motor_a);
      motor_brake(&motors.motor_b);
      pid.integral = 0.0f;

      if (++log_counter >= 10) {
        log_counter = 0;
        ESP_LOGW(TAG, "Fallen: %.2f deg, motors stopped", mpu.pitch);
      }
    } else {
      float output = pid_compute(&pid, error, mpu.pitch_rate, dt);

      if (output > Config::Motor::MAX_SPEED) {
        output = Config::Motor::MAX_SPEED;
      } else if (output < -Config::Motor::MAX_SPEED) {
        output = -Config::Motor::MAX_SPEED;
      }

      int pwm = (int)output;

      if (pwm > 0 && pwm < control.min_pwm) {
        pwm = control.min_pwm;
      } else if (pwm < 0 && pwm > -control.min_pwm) {
        pwm = -control.min_pwm;
      }

      motor_drive(&motors.motor_a, pwm);
      motor_drive(&motors.motor_b, pwm);

      if (++log_counter >= 10) {
        log_counter = 0;
        ESP_LOGI(TAG, "pitch=%.2f err=%.2f pwm=%d", mpu.pitch, error, pwm);
      }
    }

    vTaskDelayUntil(&last_wake_time, period);
  }
}

void balance_init(void) {
  spi_bus_config_t bus_cfg = {};
  bus_cfg.mosi_io_num = Config::Pins::MPU_MOSI;
  bus_cfg.miso_io_num = Config::Pins::MPU_MISO;
  bus_cfg.sclk_io_num = Config::Pins::MPU_CLK;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.quadhd_io_num = -1;
  ESP_ERROR_CHECK(
      spi_bus_initialize(Config::MPU::SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

  mpu9250_config_t cfg = {};
  cfg.gyro_scale = Config::MPU::GYRO_RANGE;
  cfg.accel_scale = Config::MPU::ACCEL_RANGE;
  cfg.dlpf = Config::MPU::DLPF;
  cfg.sample_rate_div = Config::MPU::SAMPLE_RATE_DIV;
  cfg.enable_compass = Config::MPU::ENABLE_COMPASS;

  spi_device_interface_config_t dev_cfg = {};
  dev_cfg.mode = Config::MPU::SPI_MODE;
  dev_cfg.clock_speed_hz = Config::MPU::SPI_CLOCK_HZ;
  dev_cfg.spics_io_num = Config::Pins::MPU_CS;
  dev_cfg.queue_size = Config::MPU::SPI_QUEUE_SIZE;

  ESP_ERROR_CHECK(mpu_init(&mpu, Config::MPU::SPI_HOST, &cfg, &dev_cfg));

  ESP_ERROR_CHECK(motor_init(&motors, Config::Pins::MOTOR_A_IN1,
                             Config::Pins::MOTOR_A_IN2,
                             Config::Pins::MOTOR_A_PWM, Config::Pins::MOTOR_B_IN1,
                             Config::Pins::MOTOR_B_IN2,
                             Config::Pins::MOTOR_B_PWM,
                             Config::Pins::MOTOR_STBY));

  ESP_LOGI(TAG, "Hardware initialized");
}

void balance_start(void) {
  xTaskCreate(balance_task, "balance",
              Config::Tasks::BALANCE_STACK_MULT * configMINIMAL_STACK_SIZE,
              nullptr, Config::Tasks::BALANCE_PRIORITY, nullptr);
}
