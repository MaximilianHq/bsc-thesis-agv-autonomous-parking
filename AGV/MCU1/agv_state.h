#pragma once
#include "types.h"
#include <Arduino.h>

float norm_ang(float ang);

// Kalman filter / Luenberger-observer (enkel variant)
void update_agv_state(agv_state &s,
                      float uwb_x, float uwb_y,
                      float wz, float dt);