#pragma once
#include "types.h"
#include <Arduino.h>
#include <math.h>

#define state_dim 6
#define meas_dim 4

class KalmanFilter
{

public:
    float x[state_dim];
    float P[state_dim][state_dim];
    float A[state_dim][state_dim];
    float H[meas_dim][state_dim];
    float Q[state_dim][state_dim];
    float R[meas_dim][meas_dim];
    float K[state_dim][meas_dim];

    KalmanFilter(float dt);

    void update(float z[meas_dim]);

protected:
    void _init_matrices(float dt);
    void _predict();
    void _invert4x4(float A[4][4], float inv[4][4]);
};

// Example use:
// float kalman_position(const ImuState &imu, const DwmState &dwm, int x, int y, float hz = 0.01)
// {
//     KalmanFilter kf(hz);
//     float z[4] = {imu.x, imu.y, dwm.x, dwm.y};
//     kf._predict();
//     kf.update(z);
//     return {kf.x, kf.y};
// }
