#pragma once
#include <Arduino.h>

struct agv_state
{
    float x = 0, y = 0, theta = 0;
    float vx = 0, vy = 0; // OBS: tolkat som body-hastigheter (fram/sid) i detta exempel
    uint8_t quality = 0;
    long t_ms = 0;
};

float norm_ang(float ang);

// Kalman filter / Luenberger-observer (enkel variant)
void update_agv_state(agv_state &s,
                      float uwb_x, float uwb_y,
                      float wz, float dt);