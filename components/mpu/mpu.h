#pragma once

#include "config.h"

#include "driver/spi_master.h"
#include "esp_err.h"
#include <cstdint>

namespace MPU_Config {

namespace Adresses {
constexpr uint8_t HIGH_BYTE_READ = 0x80;
constexpr uint8_t HIGH_BYTE_WRITE = 0x7F;
constexpr uint8_t SMPLRT_DIV = 0x19;
constexpr uint8_t CONFIG = 0x1A;
constexpr uint8_t GYRO_CONFIG = 0x1B;
constexpr uint8_t ACCEL_CONFIG = 0x1C;
constexpr uint8_t ACCEL_CONFIG_2 = 0x1D;
constexpr uint8_t I2C_MST_CTRL = 0x24;
constexpr uint8_t I2C_SLV0_ADDR = 0x25;
constexpr uint8_t I2C_SLV0_REG = 0x26;
constexpr uint8_t I2C_SLV0_CTRL = 0x27;
constexpr uint8_t I2C_SLV0_DO = 0x63;
constexpr uint8_t USER_CTRL = 0x6A;
constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t PWR_MGMT_2 = 0x6C;
constexpr uint8_t WHO_AM_I = 0x75;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr uint8_t EXT_SENS_DATA_00 = 0x49;
namespace AK8963 {
constexpr uint8_t I2C_ADDR = 0x0C;
constexpr uint8_t WIA = 0x00;
constexpr uint8_t HXL = 0x03;
constexpr uint8_t CNTL1 = 0x0A;
constexpr uint8_t CNTL2 = 0x0B;
} // namespace AK8963
} // namespace Adresses

} // namespace MPU_Config

typedef struct {
  int16_t ax, ay, az;
  int16_t temp;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
} mpu9250_raw_data_t;

typedef struct {
  gyro_scale_t gyro_scale;
  accel_scale_t accel_scale;
  dlpf_bandwidth_t dlpf;
  uint8_t sample_rate_div;
  bool enable_compass;
} mpu9250_config_t;

typedef struct {
  spi_host_device_t host;
  spi_device_handle_t spi;
  mpu9250_config_t config;
  spi_device_interface_config_t spi_config;
  float gyro_bias_x, gyro_bias_y, gyro_bias_z;
  float pitch, roll;
  float pitch_rate, roll_rate;
  float pitch_P, roll_P;
  uint64_t last_time_us;
} mpu_handle_t;

esp_err_t mpu_calib(mpu_handle_t *mpu);
uint8_t mpu_read_reg(spi_device_handle_t spi, uint8_t reg);
esp_err_t mpu_write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t data);
esp_err_t mpu_disable_i2c(spi_device_handle_t spi);
uint8_t mpu_who_am_i(spi_device_handle_t spi);
esp_err_t mpu_init(mpu_handle_t *mpu, spi_host_device_t host,
                   const mpu9250_config_t *cfg,
                   const spi_device_interface_config_t *spi_cfg);
esp_err_t mpu9250_configure(mpu_handle_t *mpu);
esp_err_t mpu_read_sensor(mpu_handle_t *mpu, mpu9250_raw_data_t *out);
esp_err_t mpu_spi_add_dev(spi_host_device_t host, spi_device_handle_t *spi,
                          const spi_device_interface_config_t *cfg);
void mpu_update(mpu_handle_t *mpu, const mpu9250_raw_data_t *raw);
