#include "imu.h"
#include <Wire.h>
#include <math.h>

IMU::IMU(uint8_t pin_sda, uint8_t pin_scl)
    : _pin_sda(pin_sda), _pin_scl(pin_scl) {}

void IMU::setup()
{
    Wire.begin(_pin_sda, _pin_scl);

    _mpu.initialize();

    if (!_mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed");
        while (1)
            delay(10);
    }

    // Valfri config (default är oftast ok)
    _mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    _mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

    last_sample = millis();
}

bool IMU::read(ImuState &state)
{
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    _mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    unsigned long now = millis();
    float dt = (now - last_sample) / 1000.0f;
    last_sample = now;

    // === KONVERTERA TILL RIKTIGA ENHETER ===
    float ax = (ax_raw / _ACC_SCALE) * _G_TO_MS2;
    float ay = (ay_raw / _ACC_SCALE) * _G_TO_MS2;
    float wz = (gz_raw / _GYRO_SCALE) * DEG_TO_RAD;

    // === DEADZONE ===
    if (fabs(ax) < 0.05f)
        ax = 0.0f;
    if (fabs(ay) < 0.05f)
        ay = 0.0f;
    if (fabs(wz) < 0.01f)
        wz = 0.0f;

    // === KOORDINATSYSTEM (justera vid behov) ===
    state.ax = ax;
    state.ay = ay;
    state.wz = wz;
    state.dt = dt;

    return true;
}
