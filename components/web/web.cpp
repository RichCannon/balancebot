#include "web.h"

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static const char *TAG = "WEB";

static httpd_handle_t server = nullptr;

static const char *HTML_PAGE = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot Control</title>
<style>
body { font-family: system-ui, sans-serif; margin: 1.2em; max-width: 460px; }
button { font-size: 1.1em; padding: 0.5em 1.2em; margin: 0.2em; border: 0;
         border-radius: 6px; cursor: pointer; }
.start { background: #2e7d32; color: #fff; }
.stop  { background: #c62828; color: #fff; }
.apply { background: #1565c0; color: #fff; }
.calib { background: #6a1b9a; color: #fff; }
h1 { font-size: 1.4em; }
label { display: flex; justify-content: space-between; margin: 0.5em 0; }
input { width: 7em; font-size: 1em; }
#statusBox { margin-top: 1em; font-weight: bold; }
#currentBox { margin: 0.5em 0 1em; color: #555; }
</style>
</head>
<body>
<h1>Robot Control</h1>
<button class="start" onclick="start()">Start</button>
<button class="stop" onclick="stop()">Stop</button>
<button class="calib" onclick="calibrate()">Configure setpoint</button>
<div id="statusBox">status: ...</div>

<h2>Current values</h2>
<div id="currentBox">-</div>

<h2>Set new values</h2>
<label>KP <input id="kp" type="number" step="0.5"></label>
<label>KI <input id="ki" type="number" step="0.05"></label>
<label>KD <input id="kd" type="number" step="0.05"></label>
<label>Setpoint (deg) <input id="sp" type="number" step="0.01"></label>
<label>MIN PWM <input id="minpwm" type="number" step="1"></label>
<label>Max angle (deg) <input id="maxangle" type="number" step="1"></label>
<label>Deadband (deg) <input id="db" type="number" step="0.1"></label>
<button class="apply" onclick="apply()">Apply</button>

<script>
function start(){ fetch('/api/start').then(r=>r.text()).then(t=>statusBox.textContent=t); }
function stop(){ fetch('/api/stop').then(r=>r.text()).then(t=>statusBox.textContent=t); }
function calibrate(){ fetch('/api/calibrate_setpoint').then(r=>r.text()).then(t=>statusBox.textContent=t); }
function apply(){
  const q = new URLSearchParams({
    kp: kp.value, ki: ki.value, kd: kd.value,
    sp: sp.value, minpwm: minpwm.value, maxangle: maxangle.value, db: db.value
  });
  fetch('/api/set?'+q).then(r=>r.text()).then(t=>statusBox.textContent=t);
}
async function poll(){
  try {
    const s = await (await fetch('/api/status')).json();
    statusBox.textContent = s.calib_pending ? 'CALIBRATING...'
                          : s.start_pending ? 'STARTING...'
                          : (s.running ? 'RUNNING' : 'STOPPED');
    currentBox.textContent = 'KP: ' + s.kp + '  KI: ' + s.ki + '  KD: ' + s.kd
        + '  SP: ' + s.sp + '  MINPWM: ' + s.minpwm
        + '  MAXANGLE: ' + s.maxangle + '  DB: ' + s.db;
  } catch(e){}
}
poll(); setInterval(poll, 1000);
</script>
</body>
</html>
)rawliteral";

static esp_err_t root_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, HTML_PAGE, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

static esp_err_t start_handler(httpd_req_t *req) {
  control.start_deadline_us =
      esp_timer_get_time() + Config::Timing::START_DELAY_MS * 1000;
  control.start_pending = true;
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "Starting in 3s...");
  return ESP_OK;
}

static esp_err_t stop_handler(httpd_req_t *req) {
  control.running = false;
  control.start_pending = false;
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "Stopped");
  return ESP_OK;
}

static esp_err_t calibrate_setpoint_handler(httpd_req_t *req) {
  control.running = false;
  control.start_pending = false;
  control.setpoint_calib_deadline_us =
      esp_timer_get_time() + Config::Timing::SETPOINT_CALIB_MS * 1000;
  control.setpoint_calib_pending = true;
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "Calibrating setpoint (10s)...");
  return ESP_OK;
}

static esp_err_t set_handler(httpd_req_t *req) {
  char query[256] = {0};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
    char val[32];
    if (httpd_query_key_value(query, "kp", val, sizeof(val)) == ESP_OK)
      control.kp = atof(val);
    if (httpd_query_key_value(query, "ki", val, sizeof(val)) == ESP_OK)
      control.ki = atof(val);
    if (httpd_query_key_value(query, "kd", val, sizeof(val)) == ESP_OK)
      control.kd = atof(val);
    if (httpd_query_key_value(query, "sp", val, sizeof(val)) == ESP_OK)
      control.setpoint_pitch = atof(val);
    if (httpd_query_key_value(query, "minpwm", val, sizeof(val)) == ESP_OK)
      control.min_pwm = atoi(val);
    if (httpd_query_key_value(query, "maxangle", val, sizeof(val)) == ESP_OK)
      control.max_angle = atof(val);
    if (httpd_query_key_value(query, "db", val, sizeof(val)) == ESP_OK)
      control.deadband = atof(val);
  }
  ESP_LOGI(TAG, "PID updated: kp=%.2f ki=%.2f kd=%.2f sp=%.2f", control.kp,
           control.ki, control.kd, control.setpoint_pitch);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_sendstr(req, "Applied");
  return ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req) {
  char json[256];
  snprintf(json, sizeof(json),
           "{\"running\":%s,\"start_pending\":%s,\"calib_pending\":%s,"
           "\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f,\"sp\":%.3f,\"minpwm\":%d,"
           "\"maxangle\":%.1f,\"db\":%.2f}",
           control.running ? "true" : "false",
           control.start_pending ? "true" : "false",
           control.setpoint_calib_pending ? "true" : "false", control.kp,
           control.ki, control.kd, control.setpoint_pitch, control.min_pwm,
           control.max_angle, control.deadband);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_sendstr(req, json);
  return ESP_OK;
}

static void register_uri(const char *uri, esp_err_t (*handler)(httpd_req_t *)) {
  httpd_uri_t u = {
      .uri = uri, .method = HTTP_GET, .handler = handler, .user_ctx = nullptr};
  httpd_register_uri_handler(server, &u);
}

static void start_http_server(void) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  if (httpd_start(&server, &config) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return;
  }
  register_uri("/", root_handler);
  register_uri("/api/start", start_handler);
  register_uri("/api/stop", stop_handler);
  register_uri("/api/calibrate_setpoint", calibrate_setpoint_handler);
  register_uri("/api/set", set_handler);
  register_uri("/api/status", status_handler);
  ESP_LOGI(TAG, "HTTP server started on port 80");
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id,
                               void *data) {
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
    esp_wifi_connect();
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "Connected! Open http://" IPSTR " in your browser",
             IP2STR(&event->ip_info.ip));
  }
}

static void wifi_init_sta(void) {
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                      &wifi_event_handler, nullptr, nullptr);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                      &wifi_event_handler, nullptr, nullptr);

  wifi_config_t wifi_config = {};
  strncpy((char *)wifi_config.sta.ssid, Config::Web::WIFI_SSID,
          sizeof(wifi_config.sta.ssid));
  strncpy((char *)wifi_config.sta.password, Config::Web::WIFI_PASS,
          sizeof(wifi_config.sta.password));

  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
  esp_wifi_start();
}

esp_err_t web_init(void) {
  if (Config::Web::WIFI_SSID[0] == '\0') {
    ESP_LOGE(TAG, "WIFI_SSID is empty — fill it in (config.h)!");
    return ESP_FAIL;
  }

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %s", esp_err_to_name(ret));
    return ESP_FAIL;
  }

  wifi_init_sta();
  start_http_server();
  return ESP_OK;
}
