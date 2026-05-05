#pragma once

#include <Arduino.h>

// Struktur som matchar din AGV-estimator
struct ImuState {
    float ax;   // acceleration x (m/s^2)
    float ay;   // acceleration y (m/s^2)
    float wz;   // yaw rate (rad/s)
    float dt;   // tidssteg (s)
};

// Initiera IMU
void imu_init();

// Läs IMU-data
ImuState imu_read();



