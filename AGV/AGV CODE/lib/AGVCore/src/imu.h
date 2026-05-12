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

    static constexpr float _G_TO_MM_S2 = 9810.0f;
    static constexpr float _ACC_DEADZONE_MM_S2 = 50.0f;
    static constexpr float _GYRO_DEADZONE_RAD_S = 0.01f;
    static constexpr float _MAX_VALID_DT_S = 0.25f;

    // Offsets fran stillastaende kalibrering.
    float _offset_ax = 0;
    float _offset_ay = 0;
    float _offset_az = 0;
    float _offset_gz = 0;
};
