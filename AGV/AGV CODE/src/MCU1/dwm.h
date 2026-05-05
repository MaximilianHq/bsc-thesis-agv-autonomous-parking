#pragma once

#include <Arduino.h>
#include <types.h>

class DWM
{
public:
    DWM(Stream &str, unsigned long d_rate = 100);

    bool dwm_cfg_get(uint16_t &cfg_node);
    bool read(DwmState &out);

private:
    enum ParserState
    {
        WAIT_TYPE,
        READ_LEN,
        READ_BODY
    };

    Stream &_str;
    ParserState _state = WAIT_TYPE;

    uint8_t _pkt[128] = {0};
    size_t _read_index = 0;
    size_t _expected_len = 0;

    unsigned long _last_req = 0;
    unsigned long _d_rate = 0;
    unsigned long _request_start = 0;

    bool _waiting_for_response = false;
    bool _response_has_status = false;
    bool _response_has_position = false;

    static constexpr unsigned long RESPONSE_TIMEOUT_MS = 100;

    void _request_pos();
    void _clear_rx();
    void _reset_parser();
    void _reset_response();
    bool _read_exact(uint8_t *rx, size_t expected_len, const char *cmd_name);

    static bool _tvl_success(const uint8_t *rx);
};
