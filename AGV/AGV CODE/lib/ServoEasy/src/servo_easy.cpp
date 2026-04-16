#include <Arduino.h>
#include <types.h>
#include "servo_easy.h"

ServoEasy::ServoEasy(float offset, float angle_min, float angle_max, bool inverted)
    : _angle_offset(offset), _angle_min(angle_min), _angle_max(angle_max), _inverted(inverted) {}

void ServoEasy::setOffset(float offset) { _angle_offset = offset; }

// degrees/s (default)
void ServoEasy::setEaseTo(float angle, float speed)
{
    this->ServoEasing::setEaseTo(_processAngle(angle), speed);
}
void ServoEasy::startEaseTo(float angle, float speed)
{
    this->ServoEasing::startEaseTo(_processAngle(angle), speed);
}
// duration ms
void ServoEasy::setEaseTo(float angle, uint32_t duration)
{
    this->ServoEasing::setEaseToD(_processAngle(angle), duration);
}
void ServoEasy::startEaseTo(float angle, uint32_t duration)
{
    this->ServoEasing::startEaseToD(_processAngle(angle), duration);
}

float ServoEasy::unprocessAngle(float angle)
{
    if (_inverted)
        angle = _angle_min + _angle_max - angle;
    angle -= _angle_offset;
    return angle;
}

float ServoEasy::get_min_angle() const { return _angle_min; }
float ServoEasy::get_max_angle() const { return _angle_max; }

float ServoEasy::rad2deg(float val) { return val * 180.0f / PI; }
float ServoEasy::deg2rad(float val) { return val * PI / 180.0f; }

float ServoEasy::_processAngle(float angle)
{
    angle += _angle_offset;
    angle = constrain(angle, _angle_min, _angle_max);
    if (_inverted)
        angle = _angle_min + _angle_max - angle;
    return angle;
}
