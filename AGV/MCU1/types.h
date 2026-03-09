#pragma once
#include <Arduino.h>

struct Packet
{
    char type;             // type
    size_t data_len = 0;   // data_length
    uint8_t data[9] = {0}; // data
    uint8_t crc = 0;       // checksum
    bool approved = false; // data integrity
};

struct dwm_state
{
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    uint8_t q = 0;
};

struct imu_state
{
    int wz = 0; // tmp
    int dt = 0; // tmp
};

struct agv_state
{
    float x = 0, y = 0, theta = 0;
    float vx = 0, vy = 0; // OBS: tolkat som body-hastigheter (fram/sid) i detta exempel
    long t_ms = 0;
};

struct agv_motion
{
    uint8_t cmd = 0x00; // movement
    long dur = 0;       // movement duration
    int32_t pred_x = 0;
    int32_t prex_y = 0
};

extern agv_state g_state;
extern agv_state g_state_prev;
extern agv_motion g_motion;
