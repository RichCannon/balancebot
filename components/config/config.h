#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "env.h"
#include <cstdint>
#include <math.h>

// MPU-9250 configuration enums (raw register values).
typedef enum {
  GYRO_FS_250DPS = 0x00,
  GYRO_FS_500DPS = 0x08,
  GYRO_FS_1000DPS = 0x10,
  GYRO_FS_2000DPS = 0x18
} gyro_scale_t;

typedef enum {
  ACCEL_FS_2G = 0x00,
  ACCEL_FS_4G = 0x08,
  ACCEL_FS_8G = 0x10,
  ACCEL_FS_16G = 0x18
} accel_scale_t;

typedef enum {
  DLPF_BW_250HZ = 0x00,
  DLPF_BW_184HZ = 0x01,
  DLPF_BW_92HZ = 0x02,
  DLPF_BW_41HZ = 0x03,
  DLPF_BW_20HZ = 0x04,
  DLPF_BW_10HZ = 0x05,
  DLPF_BW_5HZ = 0x06
} dlpf_bandwidth_t;

namespace Config {

namespace Pins {
constexpr gpio_num_t MPU_MISO = GPIO_NUM_13;
constexpr gpio_num_t MPU_MOSI = GPIO_NUM_11;
constexpr gpio_num_t MPU_CLK = GPIO_NUM_12;
constexpr gpio_num_t MPU_CS = GPIO_NUM_10;

constexpr gpio_num_t MOTOR_A_PWM = GPIO_NUM_4;
constexpr gpio_num_t MOTOR_A_IN2 = GPIO_NUM_5;
constexpr gpio_num_t MOTOR_A_IN1 = GPIO_NUM_6;
constexpr gpio_num_t MOTOR_STBY = GPIO_NUM_7;
constexpr gpio_num_t MOTOR_B_IN1 = GPIO_NUM_15;
constexpr gpio_num_t MOTOR_B_IN2 = GPIO_NUM_16;
constexpr gpio_num_t MOTOR_B_PWM = GPIO_NUM_17;
} // namespace Pins

namespace MPU {
constexpr spi_host_device_t SPI_HOST = SPI2_HOST;
constexpr uint32_t SPI_CLOCK_HZ = 1000000;
constexpr int SPI_MODE = 0;
constexpr int SPI_QUEUE_SIZE = 1;
constexpr gyro_scale_t GYRO_RANGE = GYRO_FS_1000DPS;
constexpr accel_scale_t ACCEL_RANGE = ACCEL_FS_4G;
constexpr dlpf_bandwidth_t DLPF = DLPF_BW_20HZ;
constexpr uint8_t SAMPLE_RATE_DIV = 4;
constexpr float GYRO_SCALE = 32.8f;    // LSB per deg/s for GYRO_RANGE
constexpr float ACCEL_SCALE = 8192.0f; // LSB per g for ACCEL_RANGE
constexpr float RAD_TO_DEG = 180.0f / M_PI;
constexpr bool ENABLE_COMPASS = false;
} // namespace MPU

namespace Filter {
constexpr float Q_ANGLE = 0.003f;
constexpr float R_ANGLE = 0.7f;
constexpr float ADAPTIVE_R_THRESHOLD_DPS = 100.0f;
constexpr float ADAPTIVE_R_VALUE = 500.0f;
constexpr float GYRO_RATE_ALPHA = 0.25f;
constexpr float COMPL_TAU = 0.5f;
constexpr bool USE_COMPLEMENTARY = true;
} // namespace Filter

namespace Control {
constexpr int FREQ_HZ = 200;
} // namespace Control

namespace Balance {
constexpr float KP = 28.0f;
constexpr float KI = 0.0f;
constexpr float KD = 0.0f;
constexpr float SETPOINT_PITCH = -4.0f;
constexpr int MIN_PWM = 61;
constexpr float MAX_ANGLE = 30.0f;
constexpr float DEADBAND = 0.5f;
constexpr float INTEGRAL_MAX = 100.0f;
} // namespace Balance

namespace Motor {
constexpr int MAX_SPEED = 1023; // 10-bit resolution
constexpr ledc_mode_t LEDC_MODE = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
constexpr uint32_t LEDC_FREQ_HZ = 20000;
constexpr ledc_timer_bit_t LEDC_RESOLUTION = LEDC_TIMER_10_BIT;
} // namespace Motor

namespace Timing {
constexpr int BOOT_DELAY_MS = 5000;
constexpr int START_DELAY_MS = 3000;
constexpr int SETPOINT_CALIB_MS = 10000;
constexpr int MEASURE_SETTLE_MS = 7000;
constexpr int MEASURE_SAMPLES = 300;
} // namespace Timing

namespace Tasks {
constexpr int BALANCE_STACK_MULT = 8;
constexpr int BALANCE_PRIORITY = 5;
} // namespace Tasks

namespace Web {
constexpr bool ENABLED = true;
constexpr const char *WIFI_SSID = ENV::WIFI_SSID;
constexpr const char *WIFI_PASS = ENV::WIFI_PASS;
} // namespace Web

} // namespace Config
