#pragma once
#include "types.h"
#include "imu.h"

extern AgvState g_state;
extern AgvState g_state_prev;

float norm_ang(float ang);

// Kalman filter / Luenberger-observer (enkel variant)
void update_agv_state(const DwmState &dwm, const ImuState &imu);