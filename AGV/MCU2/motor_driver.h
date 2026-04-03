#pragma once
#include "types.h"

// CAUTION DO NOT CHANGE
#define PWM_FREQ 20000
#define PWM_RES 10

struct MotorChannelConfig
{
    int pin_dir;
    int pin_pwm;
};

struct MotorDriverConfig
{
    MotorChannelConfig c1;
    MotorChannelConfig c2;
    MotorChannelConfig c3;
    MotorChannelConfig c4;
    int pin_drv;
    int pin_en;
    int pin_err;
};

class MotorChannel
{
public:
    MotorChannel(MotorChannelConfig &cfg);

    void execute_move(bool dir, uint8_t percent);
    void stop();

    static uint32_t percentage_to_bits(uint8_t percent);

private:
    void _setup();
    int _pin_dir;
    int _pin_pwm;

    friend class MotorDriver;
};

class MotorDriver
{
public:
    MotorDriver(MotorDriverConfig &cfg);

    MotorChannel c1;
    MotorChannel c2;
    MotorChannel c3;
    MotorChannel c4;

    void setup();
    void update();

    // ENABLE PIN
    void drivers_enable();
    void drivers_disable();
    void drivers_restart();

    // DRIVE PIN
    void outputs_enable();
    void outputs_disable();
    void outputs_restart();

    // CHANNELS
    void channels_stop_all();

    void temperature_error_reset_all();

    void move(uint8_t cmd, uint8_t spd_percent = 50, unsigned long duration_ms = 0);

private:
    int _pin_drv;
    int _pin_en;
    int _pin_err;

    unsigned long _move_start_time;
    unsigned long _move_duration;
    bool _timed_move_active = false;
};