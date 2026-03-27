#pragma once
#include "types.h"
#include <Arduino.h>

void initMatrices(float dt);
void predict();
void update(float z[meas_dim]);
void invert4x4(float A[4][4], float inv[4][4]);

float kalman_position(const ImuState &imu, const DwmState &dwm, int x, int y, int hz = 0.01)