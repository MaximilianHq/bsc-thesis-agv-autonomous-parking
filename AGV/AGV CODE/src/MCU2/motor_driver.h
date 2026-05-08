#pragma once
#include <types.h>
#include <park.h>

// CAUTION DO NOT CHANGE
#define PWM_FREQ 1000
#define PWM_RES 10

class MotorChannel
{
public:
    struct MotorChannelConfig
    {
        int pin_dir, pin_pwm, channel;
        bool invert_dir;
    };

    MotorChannel(MotorChannelConfig &cfg);

    void execute_move(bool dir, uint8_t percent);
    void stop();

    static uint32_t percentage_to_bits(uint8_t percent);

private:
    void _setup();
    int _pin_dir, _pin_pwm, _chnl;
    bool _invert_dir;

    friend class MotorDriver;
};

class MotorDriver
{
public:
    struct MotorDriverConfig
    {
        MotorChannel::MotorChannelConfig c1, c2, c3, c4;
        int pin_drv, pin_en, pin_err;
    };

    struct SpeedPair
    {
        uint8_t fast, slow;
    };

    MotorDriver(MotorDriverConfig &cfg);

    MotorChannel c1, c2, c3, c4;

    void setup();
    void update();

    void drivers_enable();  // ENABLE PIN ON
    void drivers_disable(); // ENABLE PIN OFF
    void drivers_restart(); // ENABLE PIN RESTART

    void outputs_enable();  // DRIVE PIN ON
    void outputs_disable(); // DRIVE PIN OFF
    void outputs_restart(); // DRIVE PIN RESTART

    // CHANNELS
    void channels_stop_all();

    // ERRORS
    bool has_error();
    void temperature_error_reset_all();

    void move(uint8_t cmd, uint8_t spd_percent = 50, unsigned long duration_ms = 0);
    static SpeedPair ratio_speed_pair(uint8_t target_percent, float slow_ratio);

private:
    int _pin_drv, _pin_en, _pin_err;
    unsigned long _move_start_time, _move_duration;
    bool _timed_move_active = false;
};