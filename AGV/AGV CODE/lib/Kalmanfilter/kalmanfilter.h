#pragma once
#include "types.h"
#include <Arduino.h>
#include <math.h>

#define state_dim 5
#define meas_dim 2

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

    void update(float z[meas_dim], float accX, float accY, float gyroZ);

protected:
    float _dt;
    void _init_matrices();
    void _predict(float accX, float accY, float gyroZ);
    void _invert2x2(float M[2][2], float inv[2][2]);
};

