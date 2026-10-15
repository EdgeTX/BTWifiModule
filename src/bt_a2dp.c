#include "bt_a2dp.h"

#include <string.h>

#include "audio_resampler.h"
#include "bt.h"
#include "bt_addr.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"

#define LOG_A2DP "A2DP"

#if defined(CONFIG_BT_CLASSIC_ENABLED) && defined(CONFIG_BT_A2DP_ENABLE)

#include "esp_a2dp_api.h"
#include "esp_gap_bt_api.h"

#define A2DP_PCM_BUFFER_SIZE 8192
#define A2DP_PCM_FEED_CHUNK_SIZE 512
#define A2DP_DISCOVERY_QUEUE_LENGTH 20
#define A2DP_DISCOVERY_MAX_DEVICES 20
#define A2DP_MEDIA_EVENT_QUEUE_LENGTH 8

typedef enum {
  A2DP_MEDIA_EVENT_CONNECTED,
  A2DP_MEDIA_EVENT_DISCONNECTED,
  A2DP_MEDIA_EVENT_CONTROL_ACK,
} a2dp_media_event_type_t;

typedef struct {
  a2dp_media_event_type_t type;
  esp_a2d_media_ctrl_t command;
  esp_a2d_media_ctrl_ack_t status;
} a2dp_media_event_t;

typedef enum {
  A2DP_MEDIA_IDLE,
  A2DP_MEDIA_CHECK_PENDING,
  A2DP_MEDIA_CHECK_WAITING,
  A2DP_MEDIA_START_PENDING,
  A2DP_MEDIA_START_WAITING,
  A2DP_MEDIA_STREAMING,
} a2dp_media_state_t;

#if !defined(A2DP_TEST_TONE)
static StreamBufferHandle_t s_pcm_buffer;
static bool s_pending_input_byte_valid;
static uint8_t s_pending_input_byte;
static bool s_pending_output_byte_valid;
static uint8_t s_pending_output_byte;
#endif
static QueueHandle_t s_discovery_queue;
static QueueHandle_t s_media_event_queue;
static volatile bool s_discovery_active;
static volatile bool s_discovery_complete = true;
static esp_bd_addr_t s_seen_devices[A2DP_DISCOVERY_MAX_DEVICES];
static size_t s_seen_device_count;
static a2dp_media_state_t s_media_state;
static audio_resampler_t s_resampler;

#if defined(A2DP_TEST_TONE)
static const int16_t s_test_tone[] = {
    0,     6393,   12539,  18204,  23170,  27245,  30273,  32137,
    32767, 32137,  30273,  27245,  23170,  18204,  12539,  6393,
    0,     -6393,  -12539, -18204, -23170, -27245, -30273, -32137,
    -32767, -32137, -30273, -27245, -23170, -18204, -12539, -6393,
};
static size_t s_test_tone_index;
#endif

static bool a2dp_read_pcm_sample(void *context, int16_t *sample)
{
  (void)context;
#if defined(A2DP_TEST_TONE)
  *sample = s_test_tone[s_test_tone_index];
  s_test_tone_index = (s_test_tone_index + 1) % (sizeof(s_test_tone) / sizeof(s_test_tone[0]));
  return true;
#else
  if (s_pcm_buffer == NULL) {
    return false;
  }

  uint8_t bytes[2];
  size_t received;
  if (s_pending_output_byte_valid) {
    received = xStreamBufferReceive(s_pcm_buffer, &bytes[1], 1, 0);
    if (received != 1) {
      return false;
    }
    bytes[0] = s_pending_output_byte;
    s_pending_output_byte_valid = false;
  } else {
    received = xStreamBufferReceive(s_pcm_buffer, bytes, sizeof(bytes), 0);
    if (received == 0) {
      return false;
    }
    if (received == 1) {
      s_pending_output_byte = bytes[0];
      s_pending_output_byte_valid = true;
      return false;
    }
  }

  *sample = (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
  return true;
#endif
}

static int32_t a2dp_data_cb(uint8_t *data, int32_t len)
{
  if (data == NULL || len <= 0) {
    return 0;
  }

  size_t byte_count = (size_t)len;
  size_t output_frames = byte_count / (2 * sizeof(int16_t));
  size_t produced = audio_resampler_process(&s_resampler, a2dp_read_pcm_sample, NULL,
                                           (int16_t *)data, output_frames);
  if (produced < output_frames) {
    size_t produced_bytes = produced * 2 * sizeof(int16_t);
    memset(data + produced_bytes, 0, byte_count - produced_bytes);
    audio_resampler_reset(&s_resampler);
  }
  size_t trailing_bytes = byte_count % (2 * sizeof(int16_t));
  if (trailing_bytes > 0) {
    memset(data + byte_count - trailing_bytes, 0, trailing_bytes);
  }
  return len;
}

static void a2dp_event_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
  if (param == NULL) {
    return;
  }

  a2dp_media_event_t media_event = {0};
  if (event == ESP_A2D_CONNECTION_STATE_EVT) {
    ESP_LOGI(LOG_A2DP, "Connection state %d", param->conn_stat.state);
    if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
      media_event.type = A2DP_MEDIA_EVENT_CONNECTED;
    } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
      media_event.type = A2DP_MEDIA_EVENT_DISCONNECTED;
    } else {
      return;
    }
  } else if (event == ESP_A2D_MEDIA_CTRL_ACK_EVT) {
    media_event.type = A2DP_MEDIA_EVENT_CONTROL_ACK;
    media_event.command = param->media_ctrl_stat.cmd;
    media_event.status = param->media_ctrl_stat.status;
  } else if (event == ESP_A2D_AUDIO_STATE_EVT) {
    ESP_LOGI(LOG_A2DP, "Audio state %d", param->audio_stat.state);
    return;
  } else {
    return;
  }

  if (s_media_event_queue != NULL && xQueueSend(s_media_event_queue, &media_event, 0) != pdTRUE) {
    ESP_LOGW(LOG_A2DP, "Dropping A2DP media event %d", media_event.type);
  }
}

static void a2dp_gap_event_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
  if (param == NULL) {
    return;
  }

  if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT) {
    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
      s_seen_device_count = 0;
      s_discovery_active = true;
      s_discovery_complete = false;
    } else {
      s_discovery_active = false;
      s_discovery_complete = true;
    }
  } else if (event == ESP_BT_GAP_DISC_RES_EVT && s_discovery_active &&
             s_discovery_queue != NULL) {
    for (size_t i = 0; i < s_seen_device_count; i++) {
      if (memcmp(s_seen_devices[i], param->disc_res.bda, BT_ADDR_LEN) == 0) {
        return;
      }
    }

    if (s_seen_device_count >= A2DP_DISCOVERY_MAX_DEVICES) {
      return;
    }

    memcpy(s_seen_devices[s_seen_device_count++], param->disc_res.bda, BT_ADDR_LEN);
    xQueueSend(s_discovery_queue, param->disc_res.bda, 0);
  }
}

bool a2dp_init(const char *device_name)
{
  if (s_media_event_queue == NULL) {
    s_media_event_queue = xQueueCreate(A2DP_MEDIA_EVENT_QUEUE_LENGTH, sizeof(a2dp_media_event_t));
    if (s_media_event_queue == NULL) {
      ESP_LOGE(LOG_A2DP, "Failed to allocate media event queue");
      return false;
    }
  }
  xQueueReset(s_media_event_queue);
  s_media_state = A2DP_MEDIA_IDLE;

  if (s_discovery_queue == NULL) {
    s_discovery_queue = xQueueCreate(A2DP_DISCOVERY_QUEUE_LENGTH, sizeof(esp_bd_addr_t));
    if (s_discovery_queue == NULL) {
      ESP_LOGE(LOG_A2DP, "Failed to allocate discovery queue");
      return false;
    }
  }
  s_discovery_active = false;
  s_discovery_complete = true;

#if !defined(A2DP_TEST_TONE)
  if (s_pcm_buffer == NULL) {
    s_pcm_buffer = xStreamBufferCreate(A2DP_PCM_BUFFER_SIZE, 1);
    if (s_pcm_buffer == NULL) {
      ESP_LOGE(LOG_A2DP, "Failed to allocate PCM stream buffer");
      return false;
    }
  }
  xStreamBufferReset(s_pcm_buffer);
  s_pending_input_byte_valid = false;
  s_pending_output_byte_valid = false;
#endif
  audio_resampler_reset(&s_resampler);
#if defined(A2DP_TEST_TONE)
  s_test_tone_index = 0;
#endif

  if (esp_bt_gap_register_callback(a2dp_gap_event_cb) != ESP_OK ||
      esp_bt_gap_set_device_name(device_name) != ESP_OK ||
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE) != ESP_OK ||
      esp_a2d_register_callback(a2dp_event_cb) != ESP_OK ||
      esp_a2d_source_register_data_callback(a2dp_data_cb) != ESP_OK ||
      esp_a2d_source_init() != ESP_OK) {
    ESP_LOGE(LOG_A2DP, "A2DP source init failed");
    return false;
  }
  return true;
}

void a2dp_deinit(void)
{
  esp_a2d_source_deinit();
  s_media_state = A2DP_MEDIA_IDLE;
  if (s_media_event_queue != NULL) {
    xQueueReset(s_media_event_queue);
  }
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

void a2dp_feed_pcm(const uint8_t *data, size_t length)
{
#if !defined(A2DP_TEST_TONE)
  if (data == NULL || s_pcm_buffer == NULL) {
    return;
  }

  uint8_t aligned[A2DP_PCM_FEED_CHUNK_SIZE + 2];
  while (length > 0) {
    size_t chunk_size = length < A2DP_PCM_FEED_CHUNK_SIZE ? length : A2DP_PCM_FEED_CHUNK_SIZE;
    size_t input_index = 0;
    size_t aligned_size = 0;

    if (s_pending_input_byte_valid && chunk_size > 0) {
      aligned[aligned_size++] = s_pending_input_byte;
      aligned[aligned_size++] = data[input_index++];
      s_pending_input_byte_valid = false;
    }

    size_t remaining = chunk_size - input_index;
    size_t complete_bytes = remaining & ~(size_t)1;
    if (complete_bytes > 0) {
      memcpy(aligned + aligned_size, data + input_index, complete_bytes);
      aligned_size += complete_bytes;
      input_index += complete_bytes;
    }

    if (input_index < chunk_size) {
      s_pending_input_byte = data[input_index];
      s_pending_input_byte_valid = true;
    }

    if (aligned_size > 0 && xStreamBufferSpacesAvailable(s_pcm_buffer) >= aligned_size) {
      xStreamBufferSend(s_pcm_buffer, aligned, aligned_size, 0);
    }

    data += chunk_size;
    length -= chunk_size;
  }
#else
  (void)data;
  (void)length;
#endif
}

void a2dp_process(void)
{
  if (s_media_event_queue == NULL) {
    return;
  }

  a2dp_media_event_t event;
  while (xQueueReceive(s_media_event_queue, &event, 0) == pdTRUE) {
    switch (event.type) {
      case A2DP_MEDIA_EVENT_CONNECTED:
        s_media_state = A2DP_MEDIA_CHECK_PENDING;
        break;

      case A2DP_MEDIA_EVENT_DISCONNECTED:
        s_media_state = A2DP_MEDIA_IDLE;
        break;

      case A2DP_MEDIA_EVENT_CONTROL_ACK:
        if (s_media_state == A2DP_MEDIA_CHECK_WAITING &&
            event.command == ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY) {
          if (event.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
            s_media_state = A2DP_MEDIA_START_PENDING;
          } else {
            ESP_LOGW(LOG_A2DP, "Media source not ready, ACK %d", event.status);
            s_media_state = A2DP_MEDIA_IDLE;
          }
        } else if (s_media_state == A2DP_MEDIA_START_WAITING &&
                   event.command == ESP_A2D_MEDIA_CTRL_START) {
          if (event.status == ESP_A2D_MEDIA_CTRL_ACK_SUCCESS) {
            ESP_LOGI(LOG_A2DP, "A2DP media started");
            s_media_state = A2DP_MEDIA_STREAMING;
          } else {
            ESP_LOGE(LOG_A2DP, "A2DP media start failed, ACK %d", event.status);
            s_media_state = A2DP_MEDIA_IDLE;
          }
        }
        break;
    }
  }

  if (s_media_state == A2DP_MEDIA_CHECK_PENDING) {
    esp_err_t err = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_CHECK_SRC_RDY);
    if (err == ESP_OK) {
      s_media_state = A2DP_MEDIA_CHECK_WAITING;
    } else {
      ESP_LOGE(LOG_A2DP, "A2DP source readiness check failed: %s", esp_err_to_name(err));
      s_media_state = A2DP_MEDIA_IDLE;
    }
  } else if (s_media_state == A2DP_MEDIA_START_PENDING) {
    esp_err_t err = esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
    if (err == ESP_OK) {
      s_media_state = A2DP_MEDIA_START_WAITING;
    } else {
      ESP_LOGE(LOG_A2DP, "A2DP media start request failed: %s", esp_err_to_name(err));
      s_media_state = A2DP_MEDIA_IDLE;
    }
  }
}

bool bt_a2dp_start_discovery(void)
{
  if (s_discovery_queue == NULL || s_discovery_active) {
    return false;
  }

  xQueueReset(s_discovery_queue);
  s_seen_device_count = 0;
  s_discovery_complete = false;
  s_discovery_active = true;

  esp_err_t err = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 8, 0);
  if (err != ESP_OK) {
    s_discovery_active = false;
    s_discovery_complete = true;
    ESP_LOGE(LOG_A2DP, "Classic discovery start failed: %s", esp_err_to_name(err));
    return false;
  }

  return true;
}

bool bt_a2dp_get_discovery_result(bt_addr_t address)
{
  return s_discovery_queue != NULL && xQueueReceive(s_discovery_queue, address, 0) == pdTRUE;
}

bool bt_a2dp_discovery_complete(void)
{
  return s_discovery_complete;
}

void bt_a2dp_cancel_discovery(void)
{
  if (s_discovery_active) {
    esp_bt_gap_cancel_discovery();
  }
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
void a2dp_feed_pcm(const uint8_t *data, size_t length)
{
  (void)data;
  (void)length;
}
void a2dp_process(void) {}
bool bt_a2dp_start_discovery(void) { return false; }
bool bt_a2dp_get_discovery_result(bt_addr_t address) { (void)address; return false; }
bool bt_a2dp_discovery_complete(void) { return true; }
void bt_a2dp_cancel_discovery(void) {}

#endif
