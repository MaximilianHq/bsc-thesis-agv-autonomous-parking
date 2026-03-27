#pragma once
#include "types.h"
#include <Arduino.h>
#include <math.h>

#define state_dim 6
#define meas_dim 4

class KalmanFilter {

    public:
        float x[state_dim];
        float P[state_dim][state_dim];
        float A[state_dim][state_dim];
        float H[meas_dim][state_dim];
        float Q[state_dim][state_dim];
        float R[meas_dim][meas_dim];
        float K[state_dim][meas_dim];

        KalmanFilter(float dt);
        void initMatrices(float dt);
        void predict();
        void update(float z[meas_dim]);
        void invert4x4(float A[4][4], float inv[4][4]);
};

float kalman_position(const ImuState &imu, const DwmState &dwm, int x, int y, int hz = 0.01);
