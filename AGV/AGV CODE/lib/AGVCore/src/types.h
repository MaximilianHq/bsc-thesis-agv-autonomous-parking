#pragma once
#include <Arduino.h>

struct Position
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

struct DwmState
{
    Position pos;
    uint8_t q = 0;
};

struct ImuState {
    float ax;   // acceleration x (m/s^2)
    float ay;   // acceleration y (m/s^2)
    float wz;   // yaw rate (rad/s)
    float dt;   // tidssteg (s)
};

struct AgvState
{
    Position pos;
    float vx = 0, vy = 0; // OBS: tolkat som body-hastigheter (fram/sid) i detta exempel
    float theta = 0;
    long t_ms = 0;
};

struct AgvMotion
{
    uint8_t cmd = 0x00;   // movement
    uint8_t velocity = 0; // speed
};

struct Debug
{
    const bool IAction = true;
    const bool dwm = true;
    const bool imu = true;
    const bool comm = true;
    const bool mcu1 = true;
    const bool mcu2 = true;
    const bool sonar = true;
    const bool driver = true;
};

extern Debug g_debug;
