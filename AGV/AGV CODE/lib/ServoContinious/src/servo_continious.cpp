#include <Arduino.h>
#include <types.h>
#include "servo_continious.h"

ServoContinious::ServoContinious(int pin_servo, int pwm_chnl, int stop_us, int min_pulse_us, int max_pulse_us, bool invert)
    : _pin_servo(pin_servo),
      _chnl(pwm_chnl),
      _stop_pulse(stop_us),
      _min_pulse(min_pulse_us),
      _max_pulse(max_pulse_us),
      _invert(invert)
{
}

void ServoContinious::setup()
{
    pinMode(_pin_servo, OUTPUT);
    ledcSetup(_chnl, _frequency, _resolutionBits);
    ledcAttachPin(_pin_servo, _chnl);
    stop();
}

void ServoContinious::update()
{
    if (_timed_move_active && (millis() - _move_start_time >= _move_duration))
    {
        stop();
        _timed_move_active = false;
    }
}

void ServoContinious::drive_forward(int8_t speed, unsigned long duration)
{
    drive_manual(abs(speed));
    if (!duration)
        return;

    _move_duration = duration;
    _timed_move_active = true;
    _move_start_time = millis();
}

void ServoContinious::drive_backward(int8_t speed, unsigned long duration)
{
    drive_manual(-abs(speed));
    if (!duration)
        return;

    _move_duration = duration;
    _timed_move_active = true;
    _move_start_time = millis();
}

void ServoContinious::drive_manual(int8_t speed)
{
    speed = constrain(speed, -100, 100);
    int pulse;

    if (speed == 0)
        pulse = _stop_pulse;
    else if (speed > 0)
        pulse = map(speed, 0, 100, _stop_pulse, _max_pulse);
    else
        pulse = map(speed, -100, 0, _min_pulse, _stop_pulse);

    writeMicroseconds(pulse);
}

void ServoContinious::stop() { writeMicroseconds(_stop_pulse); }

void ServoContinious::writeMicroseconds(int pulse_us)
{
    pulse_us = constrain(pulse_us, _min_pulse, _max_pulse);

    uint32_t maxDuty = (1UL << _resolutionBits) - 1;
    uint32_t duty = (uint32_t)pulse_us * maxDuty / _periodUs;

    ledcWrite(_chnl, duty);
}