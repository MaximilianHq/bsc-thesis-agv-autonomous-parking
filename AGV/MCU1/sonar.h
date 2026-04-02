#pragma once
#include "types.h"
#include "servo_easy.h"
#include <Arduino.h>

constexpr float SERVO_SPEED = 20;

class Sonar
{
public:
    Sonar(int servo_pin, int trig_pin, int echo_pin, float sonar_range, float servo_offset, float min_ang = 90, float max_ang = 90, bool servo_inverted = false);

    bool sonar_init();
    bool update();
    Position get_obstacle_position();
    void reset();

private:
    // Servo
    ServoEasy _servo;

    // Ultrasonic
    float _duration,
        _distance = 0;
    int _servo_pin, _trig_pin, _echo_pin;

    // Sonar
    char _sonar_dir = 'R';
    bool _sonar_scan = true;
    float _sonar_range;
};
