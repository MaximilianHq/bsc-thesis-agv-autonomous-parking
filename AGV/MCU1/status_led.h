#pragma once
#include "sreg_handler.h"
#include <Arduino.h>

struct Color
{
    uint8_t r, g, b;
};

struct LEDState
{
    Color color;
    bool blinking;
    unsigned long interval;
};

class StatusLED
{
public:
    enum LED
    {
        // --- SYSTEM STATUS ---
        STATUS_BOOT,          // AGV booting
        STATUS_READY,         // AGV is powered on and idle
        STATUS_BLE_SEARCHING, // Searching for Bluetooth device
        STATUS_BLE_CONNECTED, // Bluetooth connected
        STATUS_ERROR,         // Error state

        // --- OPERATION / COMMAND STATES ---
        STATUS_CMD_RECEIVING, // Receiving or updating command
        STATUS_CMD_EXECUTING, // Executing current command
        STATUS_CMD_STOPPING,  // Stopping current action
        STATUS_OBSTACLE,      // Obstacle detected
        STATUS_PARKING,       // Performing parking maneuver
        STATUS_RETURNING      // Returning to home/base
    };

    StatusLED(SRegHandler sreg, int pin_r, int pin_g, int pin_b, LED status = STATUS_READY, bool boolean_return = true);

    void setup();
    void update();

    void set_status(LED status);

private:
    int _pin_r, _pin_g, _pin_b;

    LED _status = STATUS_READY;
    bool _blinking = false;
    bool _led_on = true;

    // Implementation specific
    bool _led_boolean_return;
    SRegHandler _sreg;

    unsigned long _last_toggle;
    unsigned long _interval;

    void _set_color(uint8_t r, uint8_t g, uint8_t b);
    void _set_color(Color c);
    void _this_function_sucks(Color c); // Implementation specific
    void _blinking_routine();

    static const LEDState _led_states[];
};