#pragma once
#include "types.h"
#include <Arduino.h>

uint8_t csum(const Packet &pkt);

class Comm
{
public:
    Comm(Stream &s, const String &dbgn);

    void read(Packet &out);
    bool write(const Packet &pkt);

private:
    Stream &str;

    enum ParserState
    {
        WAIT_START,
        READ_TYPE,
        READ_BODY
    };

    ParserState state = WAIT_START;
    Packet pkt;
    size_t i = 0;

    const String debug_name;
};