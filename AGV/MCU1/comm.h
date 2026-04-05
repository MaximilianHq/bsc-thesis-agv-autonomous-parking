#pragma once
#include "types.h"
#include <Arduino.h>

uint8_t csum(const Packet &pkt);

class Comm
{
public:
    Comm(Stream &s, const String &debug_id);

    void read(Packet &out);
    bool write(const Packet &pkt);

private:
    Stream &_str;

    enum ParserState
    {
        WAIT_START,
        READ_TYPE,
        READ_BODY
    };

    ParserState _state = WAIT_START;
    Packet _pkt;
    size_t i = 0;

    const String _dbg_name;
};