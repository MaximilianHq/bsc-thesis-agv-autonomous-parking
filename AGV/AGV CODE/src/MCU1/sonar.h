#pragma once
#include <Arduino.h>
#include <types.h>
#include <servo_easy.h>

class SysCtrl;

class Sonar
{
public:
    struct SonarConfig
    {
        int pin_servo, pin_trig, pin_echo;
        int sonar_range, sonar_speed, sonar_angle;
        float servo_offset;
        float min_servo_ang = 0;
        float max_servo_ang = 180;
        bool servo_inverted = false;
    };

    Sonar(const SonarConfig &cfg, SysCtrl &actions);

    bool setup();
    bool update();
    Position get_obstacle_position();
    void reset();

private:
    SysCtrl &_actions;
    // Servo
    ServoEasy _servo;

    // Ultrasonic
    float _duration, _distance = 0;

    // Sonar
    int _pin_servo, _pin_trig, _pin_echo;
    bool _dir = true; // true = right, false = left
    bool _scan = true;
    int _sonar_range, _sonar_angle;
    float _sonar_speed;
};
