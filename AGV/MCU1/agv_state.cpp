#include "agv_state.h"
#include "types.h"
#include <Arduino.h>

float err_co_uwb = 1.0f;
float err_co_imu = 1.0f;

float norm_ang(float ang)
{
    while (ang > PI)
        ang -= 2.0f * PI;
    while (ang < -PI)
        ang += 2.0f * PI;
    return ang;
}

// Kalman filter / Luenberger-observer (enkel variant)
void update_agv_state(agv_state &s,
                      float uwb_x, float uwb_y,
                      float wz, float dt)
{
    agv_state pred = s;

    // predicted state
    // x = \cos θ, y = \sin θ
    // x_{agv} = \cos θ * v_x + \cos(θ-90\deg) * v_y = \cos θ * v_x - \sin θ * v_y
    // y_{agv} = \sin θ * v_y + \sin(θ-90\deg) * v_y = \sin θ * v_x + \cos θ * v_y

    pred.x = s.x + s.vx * cosf(s.theta) - s.vy * sinf(s.theta);
    pred.y = s.y + s.vx * sinf(s.theta) + s.vy * cosf(s.theta);
    pred.theta = norm_ang(s.theta + wz * dt);

    // position error
    float err_x = uwb_x - pred.x;
    float err_y = uwb_y - pred.y;

    // position correction
    agv_state upd = pred;
    upd.x += err_co_uwb * err_x;
    upd.y += err_co_uwb * err_y;

    // rotation error
    // needs reference meashurement, magnetometer?

    s = upd;
}