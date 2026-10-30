#include "mpu.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>

static const char *TAG = "MPU9250";

static float calc_pitch(float ax, float ay, float az) {
  return atan2f(-ax, sqrtf(ay * ay + az * az)) *
         Config::MPU::RAD_TO_DEG;
}
static float calc_roll(float ay, float az) {
  return atan2f(ay, az) * Config::MPU::RAD_TO_DEG;
}

static float compl_step(float state, float rate, float meas, float dt) {
  float alpha = Config::Filter::COMPL_TAU / (Config::Filter::COMPL_TAU + dt);
  return alpha * (state + rate * dt) + (1.0f - alpha) * meas;
}

// 1D Kalman filter for one axis: fuses a gyro rate (deg/s) with an
// accelerometer angle (deg).
static float kalman_step(float &state, float &P, float rate, float meas,
                         float dt) {
  state += rate * dt; // predict: integrate gyro
  P += Config::Filter::Q_ANGLE;       // uncertainty grows

  // During fast motion the accelerometer also measures linear/centripetal
  // acceleration, so its angle is unreliable — boost R to trust the gyro.
  float R = (fabsf(rate) > Config::Filter::ADAPTIVE_R_THRESHOLD_DPS)
                ? Config::Filter::ADAPTIVE_R_VALUE
                : Config::Filter::R_ANGLE;

  float K = P / (P + R);       // Kalman gain
  state += K * (meas - state); // update: correct with accel
  P = (1.0f - K) * P;          // uncertainty shrinks

  return state;
}

uint8_t mpu_read_reg(spi_device_handle_t spi, uint8_t reg) {
  uint8_t tx_data[2] = {(uint8_t)(reg | MPU_Config::Adresses::HIGH_BYTE_READ),
                        0x00};
  uint8_t rx_data[2] = {0};

  spi_transaction_t t = {};
  t.length = 16;
  t.tx_buffer = tx_data;
  t.rx_buffer = rx_data;

  spi_device_polling_transmit(spi, &t);

  return rx_data[1];
}
esp_err_t mpu_write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t data) {
  // Clear the R/W bit (MSB) for a write: 0x7F is a mask, so use AND.
  uint8_t tx_data[2] = {(uint8_t)(reg & MPU_Config::Adresses::HIGH_BYTE_WRITE),
                        data};

  spi_transaction_t t = {};
  t.length = 16;
  t.tx_buffer = tx_data;
  t.rx_buffer = NULL;

  return spi_device_polling_transmit(spi, &t);
}

esp_err_t mpu_disable_i2c(spi_device_handle_t spi) {
  esp_err_t res;
  uint8_t current_val = mpu_read_reg(spi, MPU_Config::Adresses::USER_CTRL);
  current_val = current_val | (1 << 4);
  res = mpu_write_reg(spi, MPU_Config::Adresses::USER_CTRL, current_val);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Error while trying to disable I2C");
    return ESP_ERR_INVALID_RESPONSE;
  }
  ESP_LOGI(TAG, "I2C successfuly disabled!");
  return res;
}

uint8_t mpu_who_am_i(spi_device_handle_t spi) {
  uint8_t who_am_i = mpu_read_reg(spi, 0x75);

  ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who_am_i);

  if (who_am_i == 0x71) {
    ESP_LOGI("MPU", "Знайдено MPU-9250 (9-осьовий)");
  } else if (who_am_i == 0x70) {
    ESP_LOGW("MPU", "Знайдено MPU-6500 (6-осьовий)");
  } else {
    ESP_LOGE("MPU", "Невідомий пристрій або помилка підключення!");
  }
  return who_am_i;
}

esp_err_t mpu_spi_add_dev(spi_host_device_t host, spi_device_handle_t *spi,
                          const spi_device_interface_config_t *cfg) {
  esp_err_t res;
  // spi_device_interface_config_t devcfg = {};
  // devcfg.mode = 0;
  // devcfg.clock_speed_hz = 1 * 1000 * 1000;
  // devcfg.spics_io_num = spics_io_num;
  // devcfg.queue_size = 1;

  // res = spi_bus_add_device(SPI2_HOST, cfg, spi);
  res = spi_bus_add_device(host, cfg, spi);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Error during adding device to SPI");
  }

  // uint8_t mpu_id = mpu_who_am_i(spi);

  return ESP_OK;
}

// Запис у внутрішній компас AK8963 через MPU I2C-Master
static esp_err_t ak8963_write_reg(spi_device_handle_t spi, uint8_t reg,
                                  uint8_t data) {
  mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_ADDR,
                MPU_Config::Adresses::AK8963::I2C_ADDR);
  mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_REG, reg);
  mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_DO, data);
  mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_CTRL, 0x81);
  vTaskDelay(pdMS_TO_TICKS(10));
  return ESP_OK;
}

static uint8_t ak8963_read_reg(spi_device_handle_t spi, uint8_t reg) {
  mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_ADDR,
                MPU_Config::Adresses::AK8963::I2C_ADDR |
                    0x80); // Адреса читання (0x8C)
  mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_REG, reg);
  mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_CTRL,
                0x81); // 0x80 (Enable) | 1 байт
  vTaskDelay(pdMS_TO_TICKS(10));
  return mpu_read_reg(
      spi, MPU_Config::Adresses::EXT_SENS_DATA_00); // Дані потрапляють у буфер
                                                    // зовнішніх сенсорів
}

// --- ГОЛОВНА ФУНКЦІЯ КОНФІГУРАЦІЇ ---
esp_err_t mpu9250_configure(mpu_handle_t *mpu) {

  spi_device_handle_t spi = mpu->spi;
  mpu9250_config_t cfg = mpu->config;

  // 1. Скидання пристрою (H_RESET)
  mpu_write_reg(spi, MPU_Config::Adresses::PWR_MGMT_1, 0x80);
  vTaskDelay(pdMS_TO_TICKS(100));

  // 2. Пробудження сенсора та вибір найкращого джерела тактування (Auto-select
  // PLL)
  mpu_write_reg(spi, MPU_Config::Adresses::PWR_MGMT_1, 0x01);
  vTaskDelay(pdMS_TO_TICKS(20));

  // 3. Блокування I2C Slave (0x10) + увімкнення внутрішнього I2C Master (0x20)
  // Разом: USER_CTRL = 0x30
  uint8_t user_ctrl = (1 << 4); // I2C_IF_DIS (забороняє I2C, фіксує SPI)
  if (cfg.enable_compass) {
    user_ctrl |= (1 << 5); // I2C_MST_EN (вмикає I2C-майстер для компаса)
  }
  mpu_write_reg(spi, MPU_Config::Adresses::USER_CTRL, user_ctrl);

  // mpu_disable_i2c(spi);

  // 4. Перевірка наявності MPU-9250 (WHO_AM_I)
  uint8_t mpu_id = mpu_who_am_i(spi);

  if (mpu_id != 0x71 && mpu_id != 0x73) {
    ESP_LOGW(TAG, "Увага: чіп повернув 0x%02X (очікувалось 0x71 для MPU-9250)",
             mpu_id);
  }

  // 5. Встановлення дільника частоти та фільтра (DLPF)
  mpu_write_reg(spi, MPU_Config::Adresses::SMPLRT_DIV, cfg.sample_rate_div);
  mpu_write_reg(spi, MPU_Config::Adresses::CONFIG, (uint8_t)cfg.dlpf);

  // 6. Налаштування діапазонів гіроскопа та акселерометра
  mpu_write_reg(spi, MPU_Config::Adresses::GYRO_CONFIG,
                (uint8_t)cfg.gyro_scale);
  mpu_write_reg(spi, MPU_Config::Adresses::ACCEL_CONFIG,
                (uint8_t)cfg.accel_scale);
  mpu_write_reg(spi, MPU_Config::Adresses::ACCEL_CONFIG_2, (uint8_t)cfg.dlpf);

  ESP_LOGI(TAG, "MPU-9250 успішно сконфігуровано по SPI.");

  // 7. Конфігурація компаса (якщо увімкнено)
  if (cfg.enable_compass) {
    // Тактова частота I2C-Master = 400 кГц (0x0D)
    mpu_write_reg(spi, MPU_Config::Adresses::I2C_MST_CTRL, 0x0D);

    // Програмне скидання AK8963
    ak8963_write_reg(spi, MPU_Config::Adresses::AK8963::CNTL2, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Перевірка ідентифікатора компаса (повинен бути 0x48)
    uint8_t ak_id = ak8963_read_reg(spi, MPU_Config::Adresses::AK8963::WIA);
    ESP_LOGI(TAG, "AK8963 WIA = 0x%02X (очікується 0x48)", ak_id);
    if (ak_id != 0x48) {
      ESP_LOGE(TAG,
               "Компас AK8963 не знайдено або плата містить лише MPU-6500!");
      return ESP_OK;
    }

    // Переведення компаса в режим безперервних вимірювань (16-біт, 100 Гц =
    // 0x16)
    ak8963_write_reg(spi, MPU_Config::Adresses::AK8963::CNTL1, 0x16);

    // Налаштування автоматичного фонового зчитування даних компаса
    // MPU-контролером: Зчитуємо 7 байтів починаючи з HXL (0x03) до ST2 (0x09).
    // Читання регістра ST2 є критично обов'язковим, інакше магнітометр заблокує
    // наступні оновлення!
    mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_ADDR,
                  MPU_Config::Adresses::AK8963::I2C_ADDR | 0x80); // Read режим
    mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_REG,
                  MPU_Config::Adresses::AK8963::HXL);
    mpu_write_reg(spi, MPU_Config::Adresses::I2C_SLV0_CTRL,
                  0x87); // 0x80 (Enable) | 7 байтів
  }

  return ESP_OK;
}

esp_err_t mpu_read_sensor(mpu_handle_t *mpu, mpu9250_raw_data_t *out) {
  spi_device_handle_t spi = mpu->spi;
  // 1 байт адреси + 14 байт (аксель, темп, гіро) + 7 байт (компас HXL..ST2) =
  // 22 байти
  uint8_t tx_buf[15] = {0};
  tx_buf[0] =
      MPU_Config::Adresses::ACCEL_XOUT_H | MPU_Config::Adresses::HIGH_BYTE_READ;
  uint8_t rx_buf[15] = {0};

  spi_transaction_t t = {};
  t.length = sizeof(tx_buf) * 8;
  t.tx_buffer = tx_buf;
  t.rx_buffer = rx_buf;

  esp_err_t res = spi_device_polling_transmit(spi, &t);

  // Парсинг акселерометра (Big Endian)
  out->ax = (int16_t)((rx_buf[1] << 8) | rx_buf[2]);
  out->ay = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
  out->az = (int16_t)((rx_buf[5] << 8) | rx_buf[6]);
  // Температура
  out->temp = (int16_t)((rx_buf[7] << 8) | rx_buf[8]);
  // Гіроскоп (Big Endian)
  out->gx = (int16_t)((rx_buf[9] << 8) | rx_buf[10]);
  out->gy = (int16_t)((rx_buf[11] << 8) | rx_buf[12]);
  out->gz = (int16_t)((rx_buf[13] << 8) | rx_buf[14]);

  // Компас AK8963 (Little Endian!)
  // rx_buf[15..20] — осі, rx_buf[21] — регістр ST2
  // uint8_t st2 = rx_buf[21];
  // if (!(st2 & 0x08)) { // Перевірка відсутності магнітного переповнення
  // (HOFL)
  //   out->mx = (int16_t)((rx_buf[16] << 8) | rx_buf[15]);
  //   out->my = (int16_t)((rx_buf[18] << 8) | rx_buf[17]);
  //   out->mz = (int16_t)((rx_buf[20] << 8) | rx_buf[19]);
  // } else {
  //   out->mx = 0;
  //   out->my = 0;
  //   out->mz = 0;
  // }

  return ESP_OK;
}

esp_err_t mpu_calib(mpu_handle_t *mpu) {

  spi_device_handle_t spi = mpu->spi;

  constexpr int samples = 500;
  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  mpu9250_raw_data_t raw;

  ESP_LOGI(TAG, "Калібрування гіроскопа... Не рухайте модуль!");

  for (int i = 0; i < samples; i++) {
    mpu_read_sensor(mpu, &raw);
    gx_sum += raw.gx;
    gy_sum += raw.gy;
    gz_sum += raw.gz;
    vTaskDelay(pdMS_TO_TICKS(4));
  }

  mpu->gyro_bias_x =
      ((float)gx_sum / samples) / Config::MPU::GYRO_SCALE;
  mpu->gyro_bias_y =
      ((float)gy_sum / samples) / Config::MPU::GYRO_SCALE;
  mpu->gyro_bias_z =
      ((float)gz_sum / samples) / Config::MPU::GYRO_SCALE;

  esp_err_t res = mpu_read_sensor(mpu, &raw);
  if (res != ESP_OK) {

    ESP_LOGI(TAG, "Помилка при читанні даних з датчику");
    return res;
  }
  float ax = (float)raw.ax / Config::MPU::ACCEL_SCALE;
  float ay = (float)raw.ay / Config::MPU::ACCEL_SCALE;
  float az = (float)raw.az / Config::MPU::ACCEL_SCALE;

  mpu->pitch = calc_pitch(ax, ay, az);
  mpu->roll = calc_roll(ay, az);
  mpu->pitch_P = Config::Filter::R_ANGLE;
  mpu->roll_P = Config::Filter::R_ANGLE;

  mpu->last_time_us = esp_timer_get_time();

  ESP_LOGI("IMU",
           "Калібрування завершено! Офсети (°/s): X=%.2f, Y=%.2f, Z=%.2f",
           mpu->gyro_bias_x, mpu->gyro_bias_y, mpu->gyro_bias_z);

  return ESP_OK;
}

void mpu_update(mpu_handle_t *mpu, const mpu9250_raw_data_t *raw) {
  int64_t current_time_us = esp_timer_get_time();
  float dt = (float)(current_time_us - mpu->last_time_us) / 1000000.0f;
  mpu->last_time_us = current_time_us;

  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
  }

  float ax = (float)raw->ax / Config::MPU::ACCEL_SCALE;
  float ay = (float)raw->ay / Config::MPU::ACCEL_SCALE;
  float az = (float)raw->az / Config::MPU::ACCEL_SCALE;

  // Normalize the accelerometer vector to 1 g.
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  if (mag > 0.1f) {
    ax /= mag;
    ay /= mag;
    az /= mag;
  }

  float gx =
      (float)raw->gx / Config::MPU::GYRO_SCALE - mpu->gyro_bias_x;
  float gy =
      (float)raw->gy / Config::MPU::GYRO_SCALE - mpu->gyro_bias_y;

  float accel_pitch = calc_pitch(ax, ay, az);
  float accel_roll = calc_roll(ay, az);

  mpu->pitch_rate += Config::Filter::GYRO_RATE_ALPHA * (gy - mpu->pitch_rate);
  mpu->roll_rate += Config::Filter::GYRO_RATE_ALPHA * (gx - mpu->roll_rate);

  if (Config::Filter::USE_COMPLEMENTARY) {
    mpu->pitch = compl_step(mpu->pitch, gy, accel_pitch, dt);
    mpu->roll = compl_step(mpu->roll, gx, accel_roll, dt);
  } else {
    mpu->pitch = kalman_step(mpu->pitch, mpu->pitch_P, gy, accel_pitch, dt);
    mpu->roll = kalman_step(mpu->roll, mpu->roll_P, gx, accel_roll, dt);
  }
}

esp_err_t mpu_init(mpu_handle_t *mpu, spi_host_device_t host,
                   const mpu9250_config_t *cfg,
                   const spi_device_interface_config_t *spi_cfg) {
  mpu->config = *cfg;
  mpu->spi_config = *spi_cfg;
  mpu->host = host;

  esp_err_t res = mpu_spi_add_dev(host, &mpu->spi, spi_cfg);

  if (res != ESP_OK) {
    return res;
  }

  res = mpu9250_configure(mpu);

  if (res != ESP_OK) {
    return res;
  }
  return mpu_calib(mpu);
}
