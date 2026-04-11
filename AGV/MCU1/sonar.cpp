#include "sonar.h"
#include "types.h"
#include "servo_easy.h"
#include <Arduino.h>

Sonar::Sonar(const SonarConfig &cfg, IActions &actions)
    : _pin_servo(cfg.pin_servo),
      _pin_trig(cfg.pin_trig),
      _pin_echo(cfg.pin_echo),
      _sonar_range(cfg.sonar_range),
      _sonar_speed(cfg.sonar_speed),
      _sonar_angle(cfg.sonar_angle),
      _servo_offset(cfg.servo_offset),
      _min_servo_ang(cfg.min_servo_ang),
      _max_servo_ang(cfg.max_servo_ang),
      _servo_inverted(cfg.servo_inverted),
      _actions(actions) {}

bool Sonar::setup()
{
    if (g_debug.sonar)
        Serial.println("[SONAR] Running setup...");

    // Servos
    _servo.attach(_pin_servo, 0);
    enableServoEasingInterrupt();
    _servo.startEaseTo(0, _sonar_speed);

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

    uint32_t timeout = static_cast<uint32_t>((2.0 * 1.3 * _sonar_range) / 0.343);
    _duration = pulseIn(_pin_echo, HIGH, timeout);
    if (_duration == 0)
        return false;
    _distance = _duration * .343 / 2; // mm

    if (_distance <= _sonar_range)
    {
        stopAllServos();
        _scan = false; // toggle for stop behaviour
        _actions.on_obstacle_detected(get_obstacle_position());
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

    _servo.startEaseTo((_dir == true) ? _sonar_angle : -_sonar_angle, _sonar_speed);
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
