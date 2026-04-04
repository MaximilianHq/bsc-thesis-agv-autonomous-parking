#include "sonar.h"
#include "types.h"
#include "servo_easy.h"
#include <Arduino.h>

Sonar::Sonar(int pin_servo, int pin_trig, int pin_echo, float sonar_range, float servo_offset, float min_servo_ang, float max_servo_ang, bool servo_inverted)
    : _servo(servo_offset, min_servo_ang, max_servo_ang, servo_inverted), _pin_servo(pin_servo), _pin_trig(pin_trig), _pin_echo(pin_echo), _range(sonar_range) {}

bool Sonar::setup()
{
    if (g_debug.sonar)
        Serial.println("[SONAR] Running setup...");

    // Servos
    _servo.attach(_pin_servo, 0);
    enableServoEasingInterrupt();
    _servo.startEaseTo(0, (float)SERVO_SPEED);

    // Ultrasonic
    pinMode(_pin_trig, OUTPUT);
    pinMode(_pin_echo, INPUT);

    if (g_debug.sonar)
        Serial.println("[SONAR] Setup finished");
    return true;
}

bool Sonar::update()
{
    if (!_scan)
        return false;

    digitalWrite(_pin_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(_pin_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(_pin_trig, LOW);

    _duration = pulseIn(_pin_echo, HIGH);
    _distance = (_duration * .0343) / 2;

    if (_distance <= _range)
    {
        stopAllServos();
        _scan = false;
        if (g_debug.sonar)
            Serial.println("[SONAR] Obstacle detected");
        return true;
    }

    if (_servo.isMoving())
        return false;

    if (_dir)
        _dir = false;
    else if (!_dir)
        _dir = true;

    _servo.startEaseTo((_dir == true) ? _servo.get_min_angle() : _servo.get_max_angle(), (float)SERVO_SPEED);
    return false;
}

Position Sonar::get_obstacle_position()
{
    float ang = ServoEasy::deg2rad(_servo.getCurrentAngle());
    Position p;
    p.x = _distance * sin(ang);
    p.y = _distance * cos(ang);

    if (g_debug.sonar)
        Serial.println("[SONAR] Obstacle at x: " + String(p.x) + " , y: " + String(p.y));
    return p;
}

void Sonar::reset()
{
    _scan = true;
}
