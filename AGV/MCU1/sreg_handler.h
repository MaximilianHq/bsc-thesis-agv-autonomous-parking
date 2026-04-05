#pragma once
#include <Arduino.h>
// THIS CODE IS AI-GENERATED 2026-04-05

enum sreg_pin
{
    QA = 0,
    QB,
    QC,
    QD,
    QE,
    QF,
    QG,
    QH
};

class SRegHandler
{

public:
    SRegHandler(int pin_data, int pin_clk, int pin_latch);

    void setup();
    void apply();

    void set_pin(sreg_pin pin, bool state, bool write_now = true);
    void set_pin(uint8_t pin, bool state, bool write_now = true);

    void toggle_pin(sreg_pin pin, bool write_now = true);
    void toggle_pin(uint8_t pin, bool write_now = true);

    void set_all(uint8_t value, bool write_now = true);
    void clear_all(bool write_now = true);
    void set_all_high(bool write_now = true);

    uint8_t get_state() const;
    bool get_pin(sreg_pin pin) const;
    bool get_pin(uint8_t pin) const;

private:
    int _pin_data, _pin_clk, _pin_latch;
    uint8_t _reg_state = 0;

    void _write_register();
};