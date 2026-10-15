#pragma once

#include <stdbool.h>

// Minimal Bluetooth Classic A2DP source. Audio streaming is stubbed (silence).
// Call after bt_classic_init().
bool a2dpInit();
void a2dpDeinit();
void a2dpConnect(const char *addr);
