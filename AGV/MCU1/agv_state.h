#pragma once
#include "types.h"
#include <Arduino.h>

float norm_ang(float ang);

// Kalman filter / Luenberger-observer (enkel variant)
void update_agv_state(const dwm_state &dwm, const imu_state &imu);