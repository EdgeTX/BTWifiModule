/* Common Bluetooth Code
 *
 */

#include "bt.h"

#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "sdkconfig.h"


#define LOG_BT "BT"
#define MAX_BTNAME_LEN 50

esp_bd_addr_t localbtaddress;
esp_bd_addr_t rmtbtaddress;

char btname[MAX_BTNAME_LEN] = "Hello";

void strtobtaddr(esp_bd_addr_t dest, char *src)
{
  for (int i = 0; i < 6; i++) {
    char str[3] = "  ";
    memcpy(str, src, 2);
    src += 2;
    dest[i] = strtoul(str, NULL, 16);
  }
}

char *btaddrtostr(char dest[13], esp_bd_addr_t src)
{
  sprintf(dest, "%02X%02X%02X%02X%02X%02X", src[0], src[1], src[2], src[3], src[4], src[5]);
  dest[12] = '\0';
  return dest;
}

bool memreleased = false;

// Classic BT/A2DP needs the controller's Classic memory, which can't be recovered once released
// (until reboot). Only release it in builds that can't use Classic BT anyway.
bool bt_a2dp_supported()
{
#if defined(CONFIG_BT_CLASSIC_ENABLED) && defined(CONFIG_BT_A2DP_ENABLE)
  return true;
#else
  return false;
#endif
}

static void bt_start_stack(esp_bt_mode_t mode)
{
  esp_err_t ret;

  if (!memreleased && !bt_a2dp_supported()) {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    memreleased = true;
  }

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(LOG_BT, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(mode);
  if (ret) {
    ESP_LOGE(LOG_BT, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(LOG_BT, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(LOG_BT, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  if (mode == ESP_BT_MODE_BLE) esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
}

void bt_init()
{
  ESP_LOGI(LOG_BT, "Starting BLE stack");
  bt_start_stack(ESP_BT_MODE_BLE);
}

// Note: BLE and Classic BT are never run together, the stack is restarted when switching.
void bt_classic_init()
{
  ESP_LOGI(LOG_BT, "Starting Classic BT stack");
  bt_start_stack(ESP_BT_MODE_CLASSIC_BT);
}

void bt_disable()
{
  ESP_LOGI(LOG_BT, "Disabling Bluetooth");
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();

  ESP_LOGI(LOG_BT, "Pausing to shutdown");
  vTaskDelay(pdMS_TO_TICKS(BT_PAUSE_BEFORE_RESTART));
}

void btSetName(const char *name)
{
  strncpy(btname, name, sizeof(btname));
  btname[sizeof(btname) - 1] = '\0';
  ESP_LOGI(LOG_BT, "Setting BT Name %s", name);
}