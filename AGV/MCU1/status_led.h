#pragma once
#include "sreg_handler.h"
#include <Arduino.h>

class StatusLED
{
public:
    enum State
    {
        // --- SYSTEM STATUS ---
        STATUS_BOOT,          // AGV booting
        STATUS_READY,         // AGV is powered on and idle
        STATUS_BLE_SEARCHING, // Searching for Bluetooth device
        STATUS_BLE_CONNECTED, // Bluetooth connected
        STATUS_ERROR,         // Error state

        // --- OPERATION / COMMAND STATES ---
        STATUS_CMD_RECEIVING, // Receiving or updating command
        STATUS_CMD_STOPPING,  // Stopping current action
        STATUS_OBSTACLE,      // Obstacle detected
        STATUS_PARKING,       // Performing parking maneuver
        STATUS_RETURNING      // Returning to home/base
    };

    struct Color
    {
        uint8_t r, g, b;
    };

    struct LEDConfig
    {
        Color color;
        bool blinking;
        unsigned long interval;
    };

    // Om boolean_return = false:
    //   pin_r/pin_g/pin_b = vanliga PWM/GPIO-pins
    //
    // Om boolean_return = true:
    //   pin_r/pin_g/pin_b = ID i det kopplade shiftregistret
    //   Ex: LED1 = 0,1,2  LED2 = 3,4,5

    StatusLED(SRegHandler sreg, int pin_r, int pin_g, int pin_b,
              bool boolean_return = true,
              State state = STATUS_BOOT);

    void setup();
    void update(StatusLED::State state);
    void update();
    void set_status(State state);

private:
    int _pin_r, _pin_g, _pin_b;

    State _state = STATUS_READY;
    bool _blinking = false;
    bool _led_on = true;

    bool _led_boolean_return;
    SRegHandler _sreg;

    unsigned long _last_toggle = 0;
    unsigned long _interval = 500;

    void _set_color(uint8_t r, uint8_t g, uint8_t b);
    void _set_color(Color c);
    void _write_sreg_color(Color c);
    void _blinking_routine();

    static const LEDConfig _led_states[];
};