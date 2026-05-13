#pragma once
#include <Arduino.h>
#include <types.h>
#include <servo_continious.h>

class SysCtrl;

class Lift
{
public:
    Lift(int pin_servo, int pin_begin_stop, int pin_endstop,
         int pwm_chnl, SysCtrl &sysctrl, bool invert)
        : _servo(pin_servo, pwm_chnl, invert),
          _pin_begin_stop(pin_begin_stop),
          _pin_endstop(pin_endstop),
          _sysctrl(sysctrl) {}

    void setup()
    {
        _servo.setup();
        pinMode(_pin_begin_stop, INPUT_PULLUP);
        pinMode(_pin_endstop, INPUT_PULLUP);
    }

    void update()
    {
        _servo.update();

        if (_is_moving)
            if (digitalRead(_pin_endstop) == LOW || digitalRead(_pin_begin_stop) == LOW)
            {
                _is_moving = false;
                _servo.stop();
            }
    }

    void lift(int8_t speed = 100)
    {
        _is_moving = true;
        _servo.drive_forward(speed);
    }

    void lower(int8_t speed = 100)
    {
        _is_moving = true;
        _servo.drive_backward(speed);
    }

private:
    SysCtrl &_sysctrl;
    ServoContinious _servo;
    int _pin_endstop, _pin_begin_stop;
    bool _is_moving = false;
};