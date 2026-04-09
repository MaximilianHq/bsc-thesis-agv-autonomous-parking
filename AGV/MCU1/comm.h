#pragma once
#include "types.h"
#include <Arduino.h>

class Comm
{
public:
    struct Packet
    {
        uint8_t type;          // type
        uint8_t seq = 0;       // sequence
        uint8_t data[9] = {0}; // data
        size_t data_len = 0;   // data_length
        uint8_t crc = 0;       // checksum
        bool approved = false; // data integrity
    };

    Comm(Stream &s, const String &debug_id);

    bool read(Packet &out);
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

class PacketHandler
{
public:
    uint8_t csum(const Comm::Packet &pkt);

    explicit PacketHandler(Comm &comm) : _comm(comm) {}
    virtual ~PacketHandler() = default;

    virtual void handle(const Comm::Packet &pkt) = 0;

protected:
    void respond(const Comm::Packet &pkt);

    Comm &_comm;
};

class BTPacketHandler : public PacketHandler
{
public:
    explicit BTPacketHandler(Comm &comm);

    void handle(const Comm::Packet &pkt) override;
};

class MCUPacketHandler : public PacketHandler
{
public:
    explicit MCUPacketHandler(Comm &comm);

    void handle(const Comm::Packet &pkt) override;
};
