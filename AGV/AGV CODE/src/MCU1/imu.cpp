#include "imu.h"

#include <Wire.h>
#include <math.h>
#include <MPU9250_asukiaaa.h>

IMU::IMU(uint8_t pin_sda, uint8_t pin_scl)
    : _pin_sda(pin_sda),
      _pin_scl(pin_scl)
{
}

void IMU::setup()
{
    Wire.begin(_pin_sda, _pin_scl);

    _imu.setWire(&Wire);

    // Starta accelerometer + gyro
    _imu.beginAccel();
    _imu.beginGyro();

    delay(100);

    calibrate();

    last_sample = millis();
}

void IMU::calibrate()
{
    Serial.println("Calibrating IMU... keep still");

    float sx = 0;
    float sy = 0;
    float sz = 0;

    float sgz = 0;

    const int n = 200;

    for (int i = 0; i < n; i++)
    {
        _imu.accelUpdate();
        _imu.gyroUpdate();

        sx += _imu.accelX();
        sy += _imu.accelY();
        sz += _imu.accelZ();

        sgz += _imu.gyroZ();

        delay(5);
    }

    _offset_ax = sx / n;
    _offset_ay = sy / n;

    // Ta bort gravitationen
    _offset_az = (sz / n) - 1.0f;

    _offset_gz = sgz / n;

    Serial.println("Calibration done");
}

bool IMU::read(ImuState &state)
{
    _imu.accelUpdate();
    _imu.gyroUpdate();

    unsigned long now = millis();

    // sekunder istället för millisekunder
    float dt = (now - last_sample) / 1000.0f;

    last_sample = now;

    // === ACCELERATION ===
    // MPU9250_asukiaaa returnerar redan i g
    float ax = (_imu.accelX() - _offset_ax) * 9.81f;
    float ay = (_imu.accelY() - _offset_ay) * 9.81f;

    // === GYRO ===
    // returneras i deg/s
    float wz = (_imu.gyroZ() - _offset_gz) * DEG_TO_RAD;

    // === DEADZONE ===
    if (fabs(ax) < 0.05f)
        ax = 0.0f;

    if (fabs(ay) < 0.05f)
        ay = 0.0f;

    if (fabs(wz) < 0.01f)
        wz = 0.0f;

    // === OUTPUT ===
    state.ax = ax;
    state.ay = ay;
    state.wz = wz;
    state.dt = dt;

    return true;
}
