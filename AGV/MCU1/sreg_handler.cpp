#include "sreg_handler.h"
#include <Arduino.h>
// THIS CODE IS AI-GENERATED 2026-04-05

void SRegHandler::_write_register()
{
    digitalWrite(_pin_latch, LOW);
    shiftOut(_pin_data, _pin_clk, LSBFIRST, _reg_state);
    digitalWrite(_pin_latch, HIGH);
    digitalWrite(_pin_latch, LOW);
}

SRegHandler::SRegHandler(int pin_data, int pin_clk, int pin_latch)
    : _pin_data(pin_data),
      _pin_clk(pin_clk),
      _pin_latch(pin_latch),
      _reg_state(0)
{
}

void SRegHandler::setup()
{
    pinMode(_pin_data, OUTPUT);
    pinMode(_pin_clk, OUTPUT);
    pinMode(_pin_latch, OUTPUT);

    digitalWrite(_pin_data, LOW);
    digitalWrite(_pin_clk, LOW);
    digitalWrite(_pin_latch, LOW);

    _reg_state = 0x00;
    _write_register();
}

void SRegHandler::apply() { _write_register(); }

void SRegHandler::set_pin(pin_sreg pin, bool state, bool write_now) { set_pin(static_cast<uint8_t>(pin), state, write_now); }

void SRegHandler::set_pin(uint8_t pin, bool state, bool write_now)
{
    if (pin > 7)
        return;

    if (state)
        _reg_state |= (1 << pin);
    else
        _reg_state &= ~(1 << pin);

    if (write_now)
        _write_register();
}

void SRegHandler::toggle_pin(pin_sreg pin, bool write_now) { toggle_pin(static_cast<uint8_t>(pin), write_now); }

void SRegHandler::toggle_pin(uint8_t pin, bool write_now)
{
    if (pin > 7)
        return;

    _reg_state ^= (1 << pin);

    if (write_now)
        _write_register();
}

void SRegHandler::set_all(uint8_t value, bool write_now)
{
    _reg_state = value;

    if (write_now)
        _write_register();
}

void SRegHandler::clear_all(bool write_now)
{
    _reg_state = 0x00;

    if (write_now)
        _write_register();
}

void SRegHandler::set_all_high(bool write_now)
{
    _reg_state = 0xFF;

    if (write_now)
        _write_register();
}

uint8_t SRegHandler::get_state() const { return _reg_state; }

bool SRegHandler::get_pin(pin_sreg pin) const { return get_pin(static_cast<uint8_t>(pin)); }

bool SRegHandler::get_pin(uint8_t pin) const
{
    if (pin > 7)
        return false;

    return ((_reg_state >> pin) & 0x01) != 0;
}