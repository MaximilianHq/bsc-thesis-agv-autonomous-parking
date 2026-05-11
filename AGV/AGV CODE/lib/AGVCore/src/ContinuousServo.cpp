#include <Arduino.h>
#include <types.h>
#include "ContinuousServo.h"

ContinuousServo::ContinuousServo(int servoPin, int pwmChannel, int stopUs = 1500, int minUs = 1000, int maxUs = 2000)
    : _pin(servoPin),
      _channel(pwmChannel),
      _stopPulse(stopUs),
      _minPulse(minUs),
      _maxPulse(maxUs)
{
}

void ContinuousServo::setup()
{
    pinMode(_pin, OUTPUT);
    ledcSetup(_channel, _frequency, _resolutionBits);
    ledcAttachPin(_pin, _channel);
    stop();
}

void ContinuousServo::update()
{
    if (_timed_move_active && (millis() - _move_start_time >= _move_duration))
    {
        stop();
        _timed_move_active = false;
    }
}

void ContinuousServo::hissa_upp()
{
    forward(100);
    _move_start_time = millis();
    _move_duration = 1000; // Move forward for 1 second
    _timed_move_active = true;
}

void ContinuousServo::hissa_ner()
{
    backward(100);
    _move_start_time = millis();
    _move_duration = 1000; // Move backward for 1 second
    _timed_move_active = true;
}

void ContinuousServo::setSpeed(int speed)
{
    speed = constrain(speed, -100, 100);
    int pulse;

    if (speed == 0)
    {
        pulse = _stopPulse;
    }
    else if (speed > 0)
    {
        pulse = map(speed, 0, 100, _stopPulse, _maxPulse);
    }
    else
    {
        pulse = map(speed, -100, 0, _minPulse, _stopPulse);
    }

    writeMicroseconds(pulse);
}

void ContinuousServo::stop()
{
    writeMicroseconds(_stopPulse);
}

void ContinuousServo::forward(int speed)
{
    setSpeed(abs(speed));
}

void ContinuousServo::backward(int speed)
{
    setSpeed(-abs(speed));
}

void ContinuousServo::writeMicroseconds(int pulseUs)
{
    pulseUs = constrain(pulseUs, _minPulse, _maxPulse);

    uint32_t maxDuty = (1UL << _resolutionBits) - 1;
    uint32_t duty = (uint32_t)pulseUs * maxDuty / _periodUs;

    ledcWrite(_channel, duty);
}