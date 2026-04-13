#include <Arduino.h>
#include "status_led.h"
#include "sreg_handler.h"

const StatusLED::LEDConfig StatusLED::_led_states[] = {
    {{255, 0, 0}, false, 500},   // STATUS_BOOT
    {{0, 255, 0}, false, 500},   // STATUS_READY
    {{0, 0, 255}, true, 500},    // STATUS_BLE_SEARCHING
    {{0, 0, 255}, false, 500},   // STATUS_BLE_CONNECTED
    {{0, 255, 255}, false, 500}, // STATUS_CMD_RECEIVING
    {{255, 0, 0}, false, 500},   // STATUS_ERROR

    {{255, 0, 0}, false, 500},  // STATUS_CMD_STOPPING
    {{255, 0, 0}, true, 500},   // STATUS_OBSTACLE
    {{0, 255, 255}, true, 500}, // STATUS_PARKING
    {{255, 255, 0}, true, 500}  // STATUS_RETURNING
};

StatusLED::StatusLED(SRegHandler &sreg, int pin_r, int pin_g, int pin_b,
                     bool boolean_return, State state)
    : _sreg(sreg),
      _pin_r(pin_r),
      _pin_g(pin_g),
      _pin_b(pin_b),
      _state(state),
      _led_boolean_return(boolean_return)
{
}

void StatusLED::setup()
{
    if (!_led_boolean_return)
    {
        // Vanligt GPIO/PWM-läge
        pinMode(_pin_r, OUTPUT);
        pinMode(_pin_g, OUTPUT);
        pinMode(_pin_b, OUTPUT);
    }

    set_status(_state);
}

void StatusLED::update(StatusLED::State state)
{
    _state = state;
    update();
}

void StatusLED::update()
{
    if (_blinking)
        _blinking_routine();
}

void StatusLED::_blinking_routine()
{
    unsigned long now = millis();

    if (now - _last_toggle >= _interval)
    {
        const LEDConfig &s = _led_states[_state];
        _last_toggle = now;
        _led_on = !_led_on;

        if (_led_on)
        {
            if (_led_boolean_return)
                _write_sreg_color(s.color);
            else
                _set_color(s.color);
        }
        else
        {
            if (_led_boolean_return)
                _write_sreg_color({0, 0, 0});
            else
                _set_color(0, 0, 0);
        }
    }
}

void StatusLED::set_status(State state)
{
    _state = state;

    const LEDConfig &s = _led_states[_state];
    _blinking = s.blinking;
    _interval = s.interval;
    _last_toggle = millis();
    _led_on = true;

    if (_led_boolean_return)
        _write_sreg_color(s.color);
    else
        _set_color(s.color);
}

void StatusLED::_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    ledcWrite(_pin_r, r);
    ledcWrite(_pin_g, g);
    ledcWrite(_pin_b, b);
}

void StatusLED::_set_color(Color c) { _set_color(c.r, c.g, c.b); }

void StatusLED::_write_sreg_color(Color c)
{
    // Här är _pin_r/_pin_g/_pin_b ID i shiftregistret, inte GPIO-pins
    _sreg.set_pin(static_cast<uint8_t>(_pin_r), c.r > 0, false);
    _sreg.set_pin(static_cast<uint8_t>(_pin_g), c.g > 0, false);
    _sreg.set_pin(static_cast<uint8_t>(_pin_b), c.b > 0, false);
    _sreg.apply();
}