#pragma once
#include <Arduino.h>
#include "system_actions.h"
#include <types.h>
#include <servo_continious.h>

class SysCtrl;

class Lift
{
public:
    Lift(int pin_servo, int pin_begin_stop, int pin_endstop,
         int pwm_chnl, bool invert)
        : _servo(pin_servo, pwm_chnl, invert),
          _pin_begin_stop(pin_begin_stop),
          _pin_endstop(pin_endstop) {}

    void setup()
    {
        _servo.setup();
        pinMode(_pin_begin_stop, INPUT_PULLUP);
        pinMode(_pin_endstop, INPUT_PULLUP);
    }

    void attach_sysctrl(SysCtrl &sysctrl)
    {
        _sysctrl = &sysctrl;
    }

    void update()
    {
        _servo.update();

        if (_is_moving)
        {
            if (digitalRead(_pin_endstop) == LOW || digitalRead(_pin_begin_stop) == LOW)
            {
                _is_moving = false;
                _servo.stop();

                if (_sysctrl != nullptr)
                {
                    _sysctrl->on_lift_done();
                }
            }
        }
    }

    void lift(int8_t speed = 100, unsigned long duration = 0)
    {
        _is_moving = true;
        _servo.drive_forward(speed, duration);
    }

    void lower(int8_t speed = 100, unsigned long duration = 0)
    {
        _is_moving = true;
        _servo.drive_backward(speed, duration);
    }

private:
    SysCtrl *_sysctrl = nullptr;
    ServoContinious _servo;
    int _pin_endstop, _pin_begin_stop;
    bool _is_moving = false;
};