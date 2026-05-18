#include <Arduino.h>
#include <types.h>
#include "servo_continious.h"

ServoContinious::ServoContinious(int pin_servo, int pwm_chnl, bool invert)
    : _pin_servo(pin_servo),
      _chnl(pwm_chnl),
      _invert(invert)
{
}

void ServoContinious::setup()
{

    if (g_debug.crane)
        Serial.println("[Servo] Running setup...");

    pinMode(_pin_servo, OUTPUT);
    ledcSetup(_chnl, _frequency, _resolution_bits);
    ledcAttachPin(_pin_servo, _chnl);
    stop();

    if (g_debug.crane)
        Serial.println("[Servo] Setup finished");
}

void ServoContinious::update()
{
    if (_timed_move_active && (millis() - _move_start_time >= _move_duration))
        stop();
}

void ServoContinious::drive_forward(int8_t speed, unsigned long duration)
{
    drive_manual(abs(speed));
    if (!duration)
        duration = 4294967295UL;

    _move_duration = duration;
    _timed_move_active = true;
    _move_start_time = millis();
}

void ServoContinious::drive_backward(int8_t speed, unsigned long duration)
{
    drive_manual(-abs(speed));
    if (!duration)
        duration = 4294967295UL;

    _move_duration = duration;
    _timed_move_active = true;
    _move_start_time = millis();
}

void ServoContinious::drive_manual(int8_t speed)
{
    speed = constrain(speed, -100, 100);
    if (_invert)
        speed = -speed;
    int pulse;

    if (speed == 0)
        pulse = _stop_us;
    else if (speed > 0)
        pulse = map(speed, 0, 100, _stop_us, _max_us);
    else
        pulse = map(speed, -100, 0, _min_us, _stop_us);

    write_microseconds(pulse);
}

void ServoContinious::stop()
{
    write_microseconds(_stop_us);
    _timed_move_active = false;
}

void ServoContinious::write_microseconds(int pulse_us)
{
    pulse_us = constrain(pulse_us, _min_us, _max_us);

    uint32_t maxDuty = (1UL << _resolution_bits) - 1;
    uint32_t duty = (uint32_t)pulse_us * maxDuty / _period_us;

    ledcWrite(_chnl, duty);
}
