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

    void home()
    {
        if (g_debug.crane)
            Serial.println("[CRANE] Homing...");
        while (digitalRead(_pin_begin_stop) == LOW)
            lower(1);
        if (g_debug.crane)
            Serial.println("[CRANE] Hominging complete");
    }

    void setup()
    {
        _servo.setup();
        pinMode(_pin_begin_stop, INPUT);
        pinMode(_pin_endstop, INPUT);
        home();

        if (g_debug.crane)
            Serial.println("[CRANE] Setup complete");
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
            if (digitalRead(_pin_endstop) == HIGH || digitalRead(_pin_begin_stop) == HIGH)
            {
                _is_moving = false;
                _servo.stop();

                if (g_debug.crane && digitalRead(_pin_endstop) == HIGH)
                    Serial.println("[CRANE] Endstop hit, stopping");

                if (g_debug.crane && digitalRead(_pin_begin_stop) == HIGH)
                    Serial.println("[CRANE] Contact with Car, stopping");

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
        _servo.drive_backward(speed);
    }

private:
    SysCtrl *_sysctrl = nullptr;
    ServoContinious _servo;
    int _pin_endstop, _pin_begin_stop;
    bool _is_moving = false;
};