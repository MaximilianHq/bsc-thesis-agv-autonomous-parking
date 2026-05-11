#pragma once
#include <Arduino.h>
#include <types.h>

class ContinuousServo
{
public:
    ContinuousServo(int servoPin, int pwmChannel, int stopUs = 1500, int minUs = 1000, int maxUs = 2000);
    void setup();
    void update();

    void hissa_upp();

    void setSpeed(int speed);
    void stop();
    void forward(int speed = 100);
    void backward(int speed = 100);

private:
    int _pin;
    int _channel;

    int _stopPulse;
    int _minPulse;
    int _maxPulse;

    unsigned long _move_duration = 0;
    unsigned long _move_start_time = 0;
    bool _timed_move_active = false;

    static const int _frequency = 50;
    static const int _resolutionBits = 16;
    static const int _periodUs = 1000000 / _frequency;

    void writeMicroseconds(int pulseUs);
};