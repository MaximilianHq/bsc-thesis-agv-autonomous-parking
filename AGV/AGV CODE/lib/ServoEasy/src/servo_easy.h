#pragma once
#include <Arduino.h>
#include <ServoEasing.h>

class ServoEasy : public ServoEasing
{
public:
    ServoEasy(float offset = 0.0f, float angle_min = 0.0f, float angle_max = 180.0f, bool inverted = false);
    void setOffset(float offset);

    // degrees/s (default)
    void setEaseTo(float angle, float speed);
    void startEaseTo(float angle, float speed);
    void setEaseTo(float angle, uint32_t duration);
    void startEaseTo(float angle, uint32_t duration);

    float unprocessAngle(float angle);

    float get_min_angle() const;
    float get_max_angle() const;

    static float rad2deg(float val);
    static float deg2rad(float val);

private:
    float _processAngle(float angle);
    float _angle_offset;
    float _angle_min;
    float _angle_max;
    bool _inverted;
};