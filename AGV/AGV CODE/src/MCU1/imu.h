#pragma once
#include <types.h>
#include <MPU9250_asukiaaa.h>

class IMU
{
public:
    explicit IMU(uint8_t pin_sda, uint8_t pin_scl);

    void setup();
    bool read(ImuState &state);
    void calibrate();

private:
    MPU9250_asukiaaa _imu;
    uint8_t _pin_sda, _pin_scl;
    unsigned long last_sample = 0;

    // Skalningskonstanter
    const float _ACC_SCALE = 16384.0f; // LSB/g (±2g default)
    const float _G_TO_MS2 = 9.81f;
    const float _GYRO_SCALE = 65.5f; // LSB/(deg/s) 131.0f

    //offset för kalibrering
    float _offset_ax = 0;
    float _offset_ay = 0;
    float _offset_az = 0;
    float _offset_gz = 0;      
};
