#pragma once
#include <Arduino.h>
#include <types.h>

class SysCtrl;

class DWM
{
public:
    DWM(Stream &str);

    void setup(uint16_t id, uint16_t p_stat, uint16_t p_mov);

    bool dwm_cfg_get(uint16_t &cfg_node);
    bool dwm_status_get();
    bool dwm_get_pos(DwmState &s);

private:
    struct DwmStatus
    {
        bool loc_ready = false;
        bool uwbmac_joined = false;
    };

    Stream &_str;

    void _clear_rx();
    bool _read_exact(uint8_t *rx, size_t expected_len, const char *cmd_name);

    static bool _tvl_success(const uint8_t *rx);
};