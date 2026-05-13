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

// kalibrering
void IMU::calibrate()
{
    Serial.println("[IMU] Calibrating... keep still");

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

    // ta bort gravitationen
    _offset_az = (sz / n) - 1.0f;

    _offset_gz = sgz / n;

    Serial.println("[IMU] Calibration Complete");
}

bool IMU::read(ImuState &state)
{
    _imu.accelUpdate();
    _imu.gyroUpdate();

    unsigned long now = millis();

    // Sekunder för att matcha integrationen i positioneringskoden.
    float dt = (now - last_sample) / 1000.0f;

    last_sample = now;

    // MPU9250_asukiaaa returnerar acceleration i g, konvertera till mm/s^2.
    float ax = (_imu.accelX() - _offset_ax) * _G_TO_MM_S2;
    float ay = (_imu.accelY() - _offset_ay) * _G_TO_MM_S2;

    // Biblioteket returnerar gyro i deg/s, konvertera till rad/s.
    float wz = (_imu.gyroZ() - _offset_gz) * DEG_TO_RAD;

    // Efter langa pauser, t.ex. startup eller blockerande delay, vill vi inte
    // integrera hela luckan som om den vore riktig rorelse.
    if (dt > _MAX_VALID_DT_S)
    {
        dt = 0.0f;
        ax = 0.0f;
        ay = 0.0f;
        wz = 0.0f;
    }

    // Ignorera supersma signaler som sannolikt bara ar brus.
    if (fabs(ax) < _ACC_DEADZONE_MM_S2)
        ax = 0.0f;

    if (fabs(ay) < _ACC_DEADZONE_MM_S2)
        ay = 0.0f;

    if (fabs(wz) < _GYRO_DEADZONE_RAD_S)
        wz = 0.0f;

    // output
    state.ax = ax;
    state.ay = ay;
    state.wz = wz;
    state.dt = dt;

    return true;
}
