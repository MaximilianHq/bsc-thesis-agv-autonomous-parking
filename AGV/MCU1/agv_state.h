#pragma once
#include "types.h"

float norm_ang(float ang);

// Kalman filter / Luenberger-observer (enkel variant)
void update_agv_state(const DwmState &dwm, const ImuState &imu);