#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bt_addr.h"

/** Initialize the Classic Bluetooth A2DP source with the configured name. */
bool a2dp_init(const char *device_name);

/** Stop the A2DP source and wait for profile teardown before controller shutdown. */
void a2dp_deinit(void);

/** Connect to a remote device specified as 12 hexadecimal digits. */
void a2dp_connect(const char *addr);

/**
 * Queue raw signed 16-bit little-endian mono PCM at 16 kHz from the radio UART.
 * The A2DP callback resamples it to ESP-IDF's 44.1 kHz stereo PCM format.
 * Define A2DP_TEST_TONE at build time to substitute a local 500 Hz test signal.
 */
void a2dp_feed_pcm(const uint8_t *data, size_t length);

/** Process A2DP events and start the media stream when the connection is ready. */
void a2dp_process(void);

/** Return whether the A2DP media channel is currently started. */
bool a2dp_media_started(void);

/** Set the AVRCP absolute volume, in the range 0..127. */
bool a2dp_set_volume(uint8_t volume);

/** Send AVRCP play to the connected A2DP sink. */
bool a2dp_play(void);

/** Send AVRCP stop to the connected A2DP sink. */
bool a2dp_stop(void);

/** Read one absolute-volume update received from the A2DP sink without blocking. */
bool a2dp_get_volume_update(uint8_t *volume);

/** Start a Classic Bluetooth inquiry while the A2DP role is active. */
bool bt_a2dp_start_discovery(void);

/** Read one address found by Classic Bluetooth inquiry without blocking. */
bool bt_a2dp_get_discovery_result(bt_addr_t address);

/** Return whether the current or most recent inquiry has completed. */
bool bt_a2dp_discovery_complete(void);

/** Cancel an active Classic Bluetooth inquiry. */
void bt_a2dp_cancel_discovery(void);
