#include <Arduino.h>
#include <types.h>
#include "motor_driver.h"

MotorChannel::MotorChannel(MotorChannelConfig &cfg)
    : _pin_dir(cfg.pin_dir), _pin_pwm(cfg.pin_pwm),
      _chnl(cfg.channel), _invert_dir(cfg.invert_dir) {}

void MotorChannel::execute_move(bool dir, uint8_t percent)
{
    digitalWrite(_pin_dir, (dir ^ _invert_dir) ? LOW : HIGH);
    ledcWrite(_chnl, percentage_to_bits(percent));
}

void MotorChannel::stop() { ledcWrite(_chnl, 0); }

uint32_t MotorChannel::percentage_to_bits(uint8_t percent)
{
    if (percent > 100)
        percent = 100;

    uint32_t max_duty = (1UL << PWM_RES) - 1;

    return (percent * max_duty + 50) / 100;
}

void MotorChannel::_setup()
{
    pinMode(_pin_dir, OUTPUT);
    ledcSetup(_chnl, PWM_FREQ, PWM_RES);
    ledcAttachPin(_pin_pwm, _chnl);
    stop();
}

MotorDriver::MotorDriver(MotorDriverConfig &cfg)
    : c1(cfg.c1), c2(cfg.c2), c3(cfg.c3), c4(cfg.c4),
      _pin_drv(cfg.pin_drv), _pin_en(cfg.pin_en), _pin_err(cfg.pin_err) {}

void MotorDriver::setup()
{
    if (g_debug.driver)
        Serial.println("[DRIVER] Running setup...");

    pinMode(_pin_drv, OUTPUT);
    pinMode(_pin_en, OUTPUT);
    pinMode(_pin_err, INPUT);

    outputs_disable();
    drivers_disable();

    c1._setup();
    c2._setup();
    c3._setup();
    c4._setup();

    if (g_debug.driver)
        Serial.println("[DRIVER] Setup finished");
}

void MotorDriver::update()
{
    if (g_debug.driver && has_error())
        Serial.println("[DRIVER] Error detected");

    if (_timed_move_active)
    {
        if (millis() - _move_start_time >= _move_duration)
        {
            channels_stop_all();
            _timed_move_active = false;
        }
    }
}

void MotorDriver::drivers_enable() { digitalWrite(_pin_en, HIGH); }
void MotorDriver::drivers_disable() { digitalWrite(_pin_en, LOW); }
void MotorDriver::drivers_restart()
{
    if (g_debug.driver)
        Serial.println("[DRIVER] Restarting drivers...");
    drivers_disable();
    delayMicroseconds(10);
    drivers_enable();
}

void MotorDriver::outputs_enable() { digitalWrite(_pin_drv, LOW); }
void MotorDriver::outputs_disable() { digitalWrite(_pin_drv, HIGH); }
void MotorDriver::outputs_restart()
{
    if (g_debug.driver)
        Serial.println("[DRIVER] Restarting outputs...");
    outputs_disable();
    delayMicroseconds(10);
    outputs_enable();
}

void MotorDriver::channels_stop_all()
{
    if (g_debug.driver)
        Serial.println("[DRIVER] Stopping all channels...");
    c1.stop();
    c2.stop();
    c3.stop();
    c4.stop();
}

bool MotorDriver::has_error()
{
    return (digitalRead(_pin_err) && // IF SPAM CHANGE TO HIGH
            digitalRead(_pin_en) &&
            digitalRead(_pin_drv))
               ? true
               : false;
}

void MotorDriver::temperature_error_reset_all()
{
    if (g_debug.driver)
        Serial.println("[DRIVER] Attempting temperature error reset...");

    // #1
    drivers_disable();
    outputs_disable();
    delayMicroseconds(10);
    // #2
    drivers_enable();

    uint8_t i = 0;
    uint8_t lim = 5;
    do
    {
        // #3
        delay(2);
        // #4
        outputs_enable();
        // #5
        delay(2);
        // #6
        outputs_disable();
        // #7
        delayMicroseconds(100);
    } while (!digitalRead(_pin_err) && i++ < lim);

    if (g_debug.driver && i >= lim)
        Serial.println("[DRIVER] Temerature error not fixable");
    else if (g_debug.driver)
        Serial.println("[DRIVER] Temerature error reset");
}

void MotorDriver::move(uint8_t cmd, uint8_t spd_percent, unsigned long duration_ms)
{
    if (duration_ms > 0)
    {
        channels_stop_all();
        _move_start_time = millis();
        _move_duration = duration_ms;
        if (g_debug.driver && _timed_move_active)
            Serial.println("[DRIVER] Exicuting new movement...");
        _timed_move_active = true;
    }
    else
    {
        _timed_move_active = false;
    }

    // CALIBRATION VARIABLES
    uint8_t spd_diff = spd_percent / 2; // For curved trajectories, slows down one of the sides
    double comp_move_right = 0;
    double comp_move_left = 0;

    switch (cmd)
    {
    case 0x00:
        // Forward
        c1.execute_move(true, spd_percent);
        c2.execute_move(true, spd_percent);
        c3.execute_move(true, spd_percent);
        c4.execute_move(true, spd_percent);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Forward");
        break;

    case 0x01:
        // Backward
        c1.execute_move(false, spd_percent);
        c2.execute_move(false, spd_percent);
        c3.execute_move(false, spd_percent);
        c4.execute_move(false, spd_percent);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Backward");
        break;

    case 0x02:
        // Right
        c1.execute_move(true, spd_percent);
        c2.execute_move(false, spd_percent);
        c3.execute_move(false, spd_percent);
        c4.execute_move(true, spd_percent);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Right");
        break;
    case 0x03:
        // Left
        c1.execute_move(false, spd_percent);
        c2.execute_move(true, spd_percent);
        c3.execute_move(true, spd_percent);
        c4.execute_move(false, spd_percent);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Left");
        break;
    case 0x04:
        // Forward-Right
        c1.execute_move(true, spd_percent);
        c2.stop();
        c3.stop();
        c4.execute_move(true, spd_percent);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Forward-Right");
        break;
    case 0x05:
        // Forward-Left
        c1.stop();
        c2.execute_move(true, spd_percent);
        c3.execute_move(true, spd_percent);
        c4.stop();

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Forward-Left");
        break;
    case 0x06:
        // Backward-Right
        c1.stop();
        c2.execute_move(false, spd_percent);
        c3.execute_move(false, spd_percent);
        c4.stop();

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Backward-Right");
        break;

    case 0x07:
        // Backward-Left
        c1.execute_move(false, spd_percent);
        c2.stop();
        c3.stop();
        c4.execute_move(false, spd_percent);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Backward-Left");
        break;

    case 0x08:
        // Turning-Right
        c1.execute_move(true, spd_percent);
        c2.execute_move(false, spd_percent);
        c3.execute_move(true, spd_percent);
        c4.execute_move(false, spd_percent);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Turning-Right");
        break;

    case 0x09:
        // Turning-Left
        c1.execute_move(false, spd_percent);
        c2.execute_move(true, spd_percent);
        c3.execute_move(false, spd_percent);
        c4.execute_move(true, spd_percent);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Turning-Left");
        break;

    case 0x0A:
        // Curved-Trajectory-Right
        c1.execute_move(true, spd_percent);
        c2.execute_move(true, spd_percent - spd_diff);
        c3.execute_move(true, spd_percent);
        c4.execute_move(true, spd_percent - spd_diff);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Curved-Trajectory-Right");
        break;

    case 0x0B:
        // Curved-Trajectory-Left
        c1.execute_move(true, spd_percent - spd_diff);
        c2.execute_move(true, spd_percent);
        c3.execute_move(true, spd_percent - spd_diff);
        c4.execute_move(true, spd_percent);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Curved-Trajectory-Left");
        break;

    case 0x0C:
    { // <-- DO NOT REMOVE
        // Lateral-Arc-Right
        MotorDriver::SpeedPair lateral_arc = ratio_speed_pair(spd_percent, 0.75f);
        c1.execute_move(true, lateral_arc.fast);
        c2.execute_move(false, lateral_arc.fast);
        c3.execute_move(false, lateral_arc.slow);
        c4.execute_move(true, lateral_arc.slow);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Lateral-Arc-Right");
    } // <-- DO NOT REMOVE
    break;

    case 0x0D:
    { // <-- DO NOT REMOVE
        // Lateral-Arc-Left
        MotorDriver::SpeedPair lateral_arc = ratio_speed_pair(spd_percent, 0.75f);
        c1.execute_move(false, lateral_arc.fast);
        c2.execute_move(true, lateral_arc.fast);
        c3.execute_move(true, lateral_arc.slow);
        c4.execute_move(false, lateral_arc.slow);

        if (g_debug.driver)
            Serial.println("[DRIVER] Exicuted new movement: Lateral-Arc-Left");
    } // <-- DO NOT REMOVE
    break;

    case 0xF0: // Back parkeringsmanöver nedre rutorna

        // Lateral-Arc-Left
        c1.execute_move(false, 25);
        c2.execute_move(false, 25);
        c3.execute_move(true, 50);
        c4.execute_move(true, 50);

        delay(3000);

        // Backward
        c1.execute_move(false, 50);
        c2.execute_move(false, 50);
        c3.execute_move(false, 50);
        c4.execute_move(false, 50);

        delay(3000);

        channels_stop_all();
        break;

    case 0xF1: // Back parkeringsmanöver övre rutorna

        // Lateral-Arc-Right
        c1.execute_move(true, 50);
        c2.execute_move(true, 50);
        c3.execute_move(false, 25);
        c4.execute_move(false, 25);

        delay(3000);

        // Backward
        c1.execute_move(false, 50);
        c2.execute_move(false, 50);
        c3.execute_move(false, 50);
        c4.execute_move(false, 50);

        delay(3000);

        // Lateral-Arc-Left
        c1.execute_move(false, 25);
        c2.execute_move(false, 25);
        c3.execute_move(true, 50);
        c4.execute_move(true, 50);

        delay(3000);

        channels_stop_all();
        break;
    default:
        if (g_debug.driver)
            Serial.println("[DRIVER] Unknown movement command");
        break;
    }
}

MotorDriver::SpeedPair MotorDriver::ratio_speed_pair(uint8_t target_percent, float slow_ratio)
{
    if (slow_ratio < 0.0f)
        slow_ratio = 0.0f;
    else if (slow_ratio > 1.0f)
        slow_ratio = 1.0f;

    // Keep the requested speed as the average of the fast/slow pair when possible.
    float fast = (2.0f * target_percent) / (1.0f + slow_ratio);
    if (fast > 100.0f)
        fast = 100.0f;

    float slow = fast * slow_ratio;

    return {static_cast<uint8_t>(fast + 0.5f),
            static_cast<uint8_t>(slow + 0.5f)};
}
