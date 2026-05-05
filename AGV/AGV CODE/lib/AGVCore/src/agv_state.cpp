#include "agv_state.h"
#include "types.h"
#include <Arduino.h>

AgvState g_state = {};
AgvState g_state_prev = {};

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
void update_agv_state(const DwmState &dwm, const ImuState &imu)
{
    g_state_prev = g_state;

    AgvState pred = g_state;

    // predicted state
    // x = \cos θ, y = \sin θ
    // x_{agv} = \cos θ * v_x + \cos(θ-90\deg) * v_y = \cos θ * v_x - \sin θ * v_y
    // y_{agv} = \sin θ * v_y + \sin(θ-90\deg) * v_y = \sin θ * v_x + \cos θ * v_y

    pred.pos.x = g_state.pos.x + g_state.vx * cosf(g_state.theta) - g_state.vy * sinf(g_state.theta);
    pred.pos.y = g_state.pos.y + g_state.vx * sinf(g_state.theta) + g_state.vy * cosf(g_state.theta);
    pred.theta = norm_ang(g_state.theta + imu.wz * imu.dt);
    
    pred.vx += imu.ax * imu.dt;
    pred.vy += imu.ay * imu.dt;

    // position error
    float err_x = dwm.pos.x - pred.pos.x;
    float err_y = dwm.pos.y - pred.pos.y;

    // position correction
    AgvState upd = pred;
    upd.pos.x += err_co_uwb * err_x;
    upd.pos.y += err_co_uwb * err_y;

    float dx = upd.pos.x - g_state_prev.pos.x;
    float dy = upd.pos.y - g_state_prev.pos.y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist > 0.05f) {
        float theta_meas = atan2f(dy, dx);
        float theta_err = norm_ang(theta_meas - upd.theta);
        upd.theta = norm_ang(upd.theta + err_co_imu * theta_err);
    }

    g_state = upd;

    // --- Rotation correction concept -----------------------------------
    // Goal: prevent long-term drift in theta (IMU yaw integration)
    //
    // Available data:
    //   g_state       -> current estimated AGV state
    //   g_state_prev  -> previous state (before last update)
    //   g_motion  -> commanded motion from higher-level system
    //
    // Idea:
    // 1. When the AGV performs a translational motion (not STOP/ROTATE),
    //    the direction of travel can be estimated from the change in UWB
    //    position.
    //
    // 2. Compute measured travel direction from position change:
    //
    //      dx = g_state.x - g_state_prev.x
    //      dy = g_state.y - g_state_prev.y
    //      theta_meas = atan2(dy, dx)
    //
    // 3. Compare this direction with the predicted heading:
    //
    //      theta_err = norm_ang(theta_meas - g_state.theta)
    //
    // 4. Apply a small correction to reduce IMU drift:
    //
    //      g_state.theta += K_theta * theta_err
    //
    // --------------------------------------------------------------------

}