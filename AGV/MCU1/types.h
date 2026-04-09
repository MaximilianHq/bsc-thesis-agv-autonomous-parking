#pragma once
#include "status_led.h"
#include "comm.h"
#include "static_vector.h"
#include <Arduino.h>

struct Position
{
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
};

struct DwmState
{
    Position pos;
    uint8_t q = 0;
};

struct ImuState
{
    int wz = 0; // tmp
    int dt = 0; // tmp
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
    uint8_t cmd = 0x00; // movement
    Position pos;
};

struct Debug
{
    const bool dwm = true;
    const bool imu = true;
    const bool comm = true;
    const bool uta = true;
    const bool mcu1 = true;
    const bool mcu2 = true;
    const bool sonar = true;
    const bool driver = true;
};

extern Debug g_debug;
extern AgvState g_state;
extern AgvState g_state_prev;
extern StaticVector<AgvMotion> g_motion;

struct AgvStatus
{
    StatusLED::State sys, cmd;
};
extern AgvStatus g_led_status;
