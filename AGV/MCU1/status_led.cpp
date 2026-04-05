#include "led_easy.h"
#include <Arduino.h>

const LEDState StatusLED::_led_states[] = {
    {{0, 255, 0}, false, 500}, // STATUS_READY
    {{0, 0, 255}, true, 500},  // STATUS_BLE_SEARCHING
    {{0, 0, 255}, false, 500}, // STATUS_BLE_CONNECTED
    {{255, 0, 0}, false, 500}, // STATUS_ERROR

    {{0, 255, 0}, false, 500},  // STATUS_CMD_RECEIVING
    {{0, 255, 0}, true, 500},   // STATUS_CMD_EXECUTING
    {{255, 0, 0}, false, 500},  // STATUS_CMD_STOPPING
    {{255, 0, 0}, true, 500},   // STATUS_OBSTACLE
    {{0, 255, 255}, true, 500}, // STATUS_PARKING
    {{255, 255, 0}, true, 500}  // STATUS_RETURNING
};

StatusLED::StatusLED(int pin_r, int pin_g, int pin_b, LED status)
    : _pin_r(pin_r), _pin_g(pin_g), _pin_b(pin_b), _status(status) {}

void StatusLED::setup()
{
    pinMode(_pin_r, OUTPUT);
    pinMode(_pin_g, OUTPUT);
    pinMode(_pin_b, OUTPUT);
    set_status(_status);
}

void StatusLED::update()
{
    _blinking_routine();
}

void StatusLED::_blinking_routine()
{
    unsigned long now = millis();

    if (now - _last_toggle >= _interval)
    {
        _last_toggle = now;
        _led_on = !_led_on;

        if (_led_on)
            _set_color(_led_states[_status].color);
        else
            _set_color(0, 0, 0);
    }
}

void StatusLED::set_status(LED status)
{
    _status = status;

    const LEDState &state = _led_states[status];

    _blinking = state.blinking;
    _interval = state.interval;
    _last_toggle = millis();
    _led_on = true;

    _set_color(state.color);
}

void StatusLED::_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    ledcWrite(_pin_r, r);
    ledcWrite(_pin_g, g);
    ledcWrite(_pin_b, b);
}

void StatusLED::_set_color(Color c) { _set_color(c.r, c.g, c.b); }
