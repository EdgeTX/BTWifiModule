#pragma once

#include <stdbool.h>

/** Initialize the Classic Bluetooth A2DP source with the configured name. */
bool a2dp_init(const char *device_name);

/** Stop the A2DP source. */
void a2dp_deinit(void);

/** Connect to a remote device specified as 12 hexadecimal digits. */
void a2dp_connect(const char *addr);
