#pragma once
#include "types.h"
#include "servo_easy.h"
#include <Arduino.h>

class Sonar
{
public:
    Sonar(int pin_servo, int pin_trig, int pin_echo, float sonar_range,
          float sonar_speed, float sonar_angle, float servo_offset,
          float min_servo_ang = 0, float max_servo_ang = 180, bool servo_inverted = false);

    bool setup();
    bool update();
    Position get_obstacle_position();
    void reset();

private:
    // Servo
    ServoEasy _servo;

    // Ultrasonic
    float _duration, _distance = 0;

    // Sonar
    int _pin_servo, _pin_trig, _pin_echo;
    bool _dir = true; // true = right, false = left
    bool _scan = true;
    float _sonar_range, _sonar_speed, _sonar_angle;
};
