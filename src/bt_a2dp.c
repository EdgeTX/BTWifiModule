/* Bluetooth Classic A2DP Source - minimal initialization/shutdown path
 * Streaming is stubbed, silence is sent until audio is fed from the UART.
 */

#include "bt_a2dp.h"

#include <string.h>

#include "bt.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define LOG_A2DP "A2DP"

#if defined(CONFIG_BT_CLASSIC_ENABLED) && defined(CONFIG_BT_A2DP_ENABLE)

#include "esp_a2dp_api.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

static int32_t a2dpDataCb(uint8_t *data, int32_t len)
{
  if (data == NULL || len < 0) return 0;
  memset(data, 0, len);  // Silence
  return len;
}

static void a2dpEventCb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
  if (event == ESP_A2D_CONNECTION_STATE_EVT) {
    ESP_LOGI(LOG_A2DP, "Connection state %d", param->conn_stat.state);
  }
}

bool a2dpInit()
{
  esp_bt_dev_set_device_name(btname);
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
  if (esp_a2d_register_callback(a2dpEventCb) != ESP_OK ||
      esp_a2d_source_register_data_cb(a2dpDataCb) != ESP_OK ||
      esp_a2d_source_init() != ESP_OK) {
    ESP_LOGE(LOG_A2DP, "A2DP source init failed");
    return false;
  }
  return true;
}

void a2dpDeinit()
{
  esp_a2d_source_deinit();
}

void a2dpConnect(const char *addr)
{
  esp_bd_addr_t bda;
  if (strlen(addr) < 12) return;
  strtobtaddr(bda, (char *)addr);
  esp_a2d_source_connect(bda);
}

#else

bool a2dpInit()
{
  ESP_LOGE(LOG_A2DP, "A2DP not supported in this build (needs CONFIG_BT_CLASSIC_ENABLED/A2DP)");
  return false;
}
void a2dpDeinit() {}
void a2dpConnect(const char *addr) {}

#endif
