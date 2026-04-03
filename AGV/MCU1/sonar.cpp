#include "sonar.h"
#include "types.h"
#include "servo_easy.h"
#include <Arduino.h>

Sonar::Sonar(int servo_pin, int trig_pin, int echo_pin, float sonar_range, float servo_offset, float min_ang, float max_ang, bool servo_inverted)
    : _servo(servo_offset, min_ang, max_ang, servo_inverted), _servo_pin(servo_pin), _trig_pin(trig_pin), _echo_pin(echo_pin), _sonar_range(sonar_range) {}

bool Sonar::sonar_init()
{
    if (g_debug.sonar)
        Serial.println("[SONAR] Running setup...");

    // Servos
    _servo.attach(_servo_pin, 0);
    enableServoEasingInterrupt();
    _servo.startEaseTo(0, (float)SERVO_SPEED);

    // Ultrasonic
    pinMode(_trig_pin, OUTPUT);
    pinMode(_echo_pin, INPUT);

    if (g_debug.sonar)
        Serial.println("[SONAR] Setup finished");
    return true;
}

bool Sonar::update()
{
    if (!_sonar_scan)
        return false;

    digitalWrite(_trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig_pin, LOW);

    _duration = pulseIn(_echo_pin, HIGH);
    _distance = (_duration * .0343) / 2;

    if (_distance <= _sonar_range)
    {
        stopAllServos();
        _sonar_scan = false;
        if (g_debug.sonar)
            Serial.println("[SONAR] Obstacle detected");
        return true;
    }

    if (_servo.isMoving())
        return false;

    if (_sonar_dir == 'R')
        _sonar_dir = 'L';
    else if (_sonar_dir == 'L')
        _sonar_dir = 'R';

    _servo.startEaseTo((_sonar_dir == 'R') ? _servo.get_min_angle() : _servo.get_max_angle(), (float)SERVO_SPEED);
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
    _sonar_scan = true;
}
