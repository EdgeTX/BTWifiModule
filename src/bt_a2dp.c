#include "bt_a2dp.h"

#include <string.h>

#include "bt.h"
#include "bt_addr.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define LOG_A2DP "A2DP"

#if defined(CONFIG_BT_CLASSIC_ENABLED) && defined(CONFIG_BT_A2DP_ENABLE)

#include "esp_a2dp_api.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

static int32_t a2dp_data_cb(uint8_t *data, int32_t len)
{
  if (data == NULL || len <= 0) {
    return 0;
  }
  memset(data, 0, (size_t)len);
  return len;
}

static void a2dp_event_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
  if (event == ESP_A2D_CONNECTION_STATE_EVT) {
    ESP_LOGI(LOG_A2DP, "Connection state %d", param->conn_stat.state);
  }
}

bool a2dp_init(const char *device_name)
{
  if (esp_bt_dev_set_device_name(device_name) != ESP_OK ||
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE) != ESP_OK ||
      esp_a2d_register_callback(a2dp_event_cb) != ESP_OK ||
      esp_a2d_source_register_data_cb(a2dp_data_cb) != ESP_OK ||
      esp_a2d_source_init() != ESP_OK) {
    ESP_LOGE(LOG_A2DP, "A2DP source init failed");
    return false;
  }
  return true;
}

void a2dp_deinit(void)
{
  esp_a2d_source_deinit();
}

void a2dp_connect(const char *addr)
{
  bt_addr_t address;
  if (addr == NULL || strlen(addr) != BT_ADDR_STR_LEN - 1) {
    return;
  }
  bt_addr_from_str(address, addr);
  esp_a2d_source_connect(address);
}

#else

bool a2dp_init(const char *device_name)
{
  (void)device_name;
  ESP_LOGE(LOG_A2DP, "A2DP not supported in this build (needs CONFIG_BT_CLASSIC_ENABLED/A2DP)");
  return false;
}
void a2dp_deinit(void) {}
void a2dp_connect(const char *addr) { (void)addr; }

#endif
