#pragma once

#include "control.h"

#include "esp_err.h"

// Connect to WiFi (station mode) and start the HTTP control server.
esp_err_t web_init(void);
