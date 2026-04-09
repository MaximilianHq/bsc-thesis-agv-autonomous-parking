#pragma once
#include "types.h"
#include "static_vector.h"
#include <Arduino.h>

class Comm
{
public:
    struct Packet
    {
        char type;             // type
        uint8_t seq = 0;       // sequence
        char data[9] = {0};    // data
        int data_len = 0;      // data_length
        uint8_t crc = 0;       // checksum
        bool approved = false; // data integrity
    };

    static uint8_t csum(const Comm::Packet &pkt);

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
    explicit PacketHandler(Comm &comm);
    virtual ~PacketHandler() = default;

    virtual void handle(const Comm::Packet &pkt);

protected:
    virtual void handle_approved(const Comm::Packet &pkt) = 0;
    static Comm::Packet make_ack(const Comm::Packet &pkt);

    void respond(const Comm::Packet &pkt);

    Comm &_comm;

    StaticVector<Comm::Packet> pkt_buffer_sent(10);
};

class BTPacketHandler : public PacketHandler
{
public:
    using PacketHandler::PacketHandler;
    // explicit BTPacketHandler(Comm &comm);

protected:
    void handle_approved(const Comm::Packet &pkt) override;
};

class MCUPacketHandler : public PacketHandler
{
public:
    using PacketHandler::PacketHandler;
    // explicit MCUPacketHandler(Comm &comm);

protected:
    void handle_approved(const Comm::Packet &pkt) override;
};
