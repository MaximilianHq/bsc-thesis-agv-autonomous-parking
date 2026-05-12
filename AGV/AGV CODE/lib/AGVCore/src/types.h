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

struct ImuState
{
    float ax; // acceleration x (mm/s^2)
    float ay; // acceleration y (mm/s^2)
    float wz; // yaw rate (rad/s)
    float dt; // tidssteg (s)
};

struct AgvState
{
    Position pos;
    float vx = 0, vy = 0; // REP 103 body-hastigheter: x framat, y vanster (mm/s)
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
    const bool sysctrl = true;
    const bool dwm = false;
    const bool imu = true;
    const bool comm = true;
    const bool mcu1 = true;
    const bool mcu2 = true;
    const bool sonar = true;
    const bool crane = true;
    const bool driver = true;
};

extern Debug g_debug;

static void pds(uint8_t *arr, int base = HEX, size_t len = 0, String msg = "")
{
    Serial.print(msg);
    Serial.print(len);
    Serial.print("/");
    Serial.print(sizeof(len));
    Serial.print(": ");

    for (size_t i = 0; i < (len != 0) ? len : sizeof(arr); i++)
    {
        if (arr[i] < 0x10)
            Serial.print("0");
        Serial.print(arr[i], base);
        Serial.print(" ");
    }
}
