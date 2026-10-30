#include "balance.h"
#include "config.h"
#include "web.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

extern "C" void app_main(void) {
  vTaskDelay(pdMS_TO_TICKS(Config::Timing::BOOT_DELAY_MS));

  balance_init();

  if constexpr (Config::Web::ENABLED) {
    if (web_init() != ESP_OK) {
      ESP_LOGW(TAG, "Web control not available (check WiFi credentials)");
    }
  }

  balance_start();
}
