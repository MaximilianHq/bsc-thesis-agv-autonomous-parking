#pragma once
#include "types.h"
#include <Arduino.h>

bool _tvl_success(const uint8_t *rx);

bool dwm_ready(Stream &str);
bool dwm_upd_rate_set(Stream &str, uint16_t period_stationary_ms, uint16_t perion_moving_ms);
bool dwm_get_pos(Stream &str, DwmState &s);