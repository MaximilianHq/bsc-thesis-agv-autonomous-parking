#pragma once
#include <Arduino.h>
#include <types.h>
#include <static_vector.h>

class IActions;

class Comm
{
public:
    struct Packet
    {
        char type;             // type
        uint16_t seq = 0;      // sequence
        uint8_t data[9] = {0}; // data
        uint8_t data_len = 0;  // data_length
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
    size_t _read_index = 0;

    const String _dbg_name;
};

class PacketHandler
{
public:
    explicit PacketHandler(Comm &comm, IActions &actions);
    virtual ~PacketHandler() = default;

    uint16_t get_sequence() const;
    virtual void handle(const Comm::Packet &pkt);

protected:
    virtual void handle_approved(const Comm::Packet &pkt) = 0;
    static Comm::Packet make_ack(const Comm::Packet &pkt);

    void respond(const Comm::Packet &pkt);

    Comm &_comm;
    uint16_t _seq; // TODO MAKE SHURE INCREMENTED PROPERLY
    IActions &_actions;

    StaticVector<Comm::Packet, 10> _pkt_buffer_sent;
    StaticVector<Comm::Packet, 10> _pkt_buffer_recieved;
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
