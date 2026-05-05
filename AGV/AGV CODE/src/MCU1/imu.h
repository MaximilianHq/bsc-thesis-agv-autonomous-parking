#pragma once
#include <types.h>
#include <MPU6050.h>

class IMU
{
public:
    explicit IMU(uint8_t pin_sda, uint8_t pin_scl);

    void setup();
    bool read(ImuState &state);

private:
    MPU6050 _mpu;
    uint8_t _pin_sda, _pin_scl;
    unsigned long last_sample = 0;

    // Skalningskonstanter
    const float _ACC_SCALE = 16384.0f; // LSB/g (±2g default)
    const float _G_TO_MS2 = 9.81f;
    const float _GYRO_SCALE = 131.0f; // LSB/(deg/s)
};
