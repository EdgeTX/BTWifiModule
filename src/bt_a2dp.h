#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Initialize the Classic Bluetooth A2DP source with the configured name. */
bool a2dp_init(const char *device_name);

/** Stop the A2DP source. */
void a2dp_deinit(void);

/** Connect to a remote device specified as 12 hexadecimal digits. */
void a2dp_connect(const char *addr);

/**
 * Queue raw signed 16-bit little-endian mono PCM at 16 kHz from the radio UART.
 * The A2DP callback resamples it to ESP-IDF's 44.1 kHz stereo PCM format.
 * Define A2DP_TEST_TONE at build time to substitute a local 500 Hz test signal.
 */
void a2dp_feed_pcm(const uint8_t *data, size_t length);
