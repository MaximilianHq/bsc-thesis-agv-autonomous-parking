#pragma once
#include <Arduino.h>
#include <types.h>

class ServoContinious
{
public:
    ServoContinious(int pin_servo, int pwm_chnl, bool invert = false);

    void setup();
    void update();

    void drive_forward(int8_t speed = 100, unsigned long duration = 0);
    void drive_backward(int8_t speed = 100, unsigned long duration = 0);
    void drive_manual(int8_t speed = 100);
    void stop();

private:
    int _pin_servo, _chnl;
    bool _invert;

    int _stop_us = 1500;
    int _min_us = 1000;
    int _max_us = 2000;

    unsigned long _move_duration = 0;
    unsigned long _move_start_time = 0;
    bool _timed_move_active = false;

    static const int _frequency = 50;
    static const int _resolution_bits = 16;
    static const int _period_us = 1000000 / _frequency;

    void write_microseconds(int pulse_us);
};