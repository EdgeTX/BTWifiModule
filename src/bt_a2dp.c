#include "bt_a2dp.h"

#include <string.h>

#include "audio_resampler.h"
#include "bt.h"
#include "bt_addr.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"

#define LOG_A2DP "A2DP"

#if defined(CONFIG_BT_CLASSIC_ENABLED) && defined(CONFIG_BT_A2DP_ENABLE)

#include "esp_a2dp_api.h"
#include "esp_gap_bt_api.h"

#define A2DP_PCM_BUFFER_SIZE 8192
#define A2DP_PCM_FEED_CHUNK_SIZE 512

#if !defined(A2DP_TEST_TONE)
static StreamBufferHandle_t s_pcm_buffer;
static bool s_pending_input_byte_valid;
static uint8_t s_pending_input_byte;
static bool s_pending_output_byte_valid;
static uint8_t s_pending_output_byte;
#endif
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
  if (event == ESP_A2D_CONNECTION_STATE_EVT) {
    ESP_LOGI(LOG_A2DP, "Connection state %d", param->conn_stat.state);
  }
}

bool a2dp_init(const char *device_name)
{
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

  if (esp_bt_gap_set_device_name(device_name) != ESP_OK ||
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

#endif
