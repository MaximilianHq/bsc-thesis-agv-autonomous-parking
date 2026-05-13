#pragma once
#include <Arduino.h>
#include <types.h>
#include <servo_continious.h>

class lift : public ServoContinious
{
public:
    lift(int pin_servo, int pin_endstop, int pwm_chnl, bool invert)
        : ServoContinious(pin_servo, pwm_chnl, invert),
          _pin_endstop(pin_endstop) {}

    void setup()
    {
        ServoContinious::setup();
    }

    void update()
    {
        ServoContinious::update();

        if (_is_homing)
            if (digitalRead(_pin_endstop) == LOW)
            {
                _is_homed = true;
                _is_homing = false;
                stop();
            }
    }

    void home()
    {
        _is_homing = true;
        _is_homed = false;
        drive_backward(100);
    }

private:
    int _pin_endstop;
    bool _is_homing = false, _is_homed = false;
};