#pragma once
#include <Arduino.h>
#include <types.h>

class ServoContinious
{
public:
    ServoContinious(int servoPin, int pwmChannel, int stopUs = 1500,
                    int minUs = 1000, int maxUs = 2000, bool invert = false);

    void setup();
    void update();

    void drive_forward(int8_t speed = 100, unsigned long duration = 0);
    void drive_backward(int8_t speed = 100, unsigned long duration = 0);
    void drive_manual(int8_t speed = 100);
    void stop();

private:
    int _pin_servo, _chnl;
    int _stop_pulse, _min_pulse, _max_pulse;
    bool _invert;

    unsigned long _move_duration = 0;
    unsigned long _move_start_time = 0;
    bool _timed_move_active = false;

    static const int _frequency = 50;
    static const int _resolutionBits = 16;
    static const int _periodUs = 1000000 / _frequency;

    void writeMicroseconds(int pulseUs);
};