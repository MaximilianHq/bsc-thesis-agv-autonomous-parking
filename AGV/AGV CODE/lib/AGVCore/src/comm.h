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
        uint8_t seq = 0;       // sequence
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

class ProtocolHandler
{
public:
    ProtocolHandler(Comm &comm, IActions &actions);

    uint8_t get_sequence() const;
    void itterate_sequence();
    void add_buffer_sent(const Comm::Packet &pkt);
    void add_buffer_rcvd(const Comm::Packet &pkt);
    void handle(const Comm::Packet &pkt);

private:
    static Comm::Packet _make_ack(const Comm::Packet &pkt);

    Comm &_comm;
    uint8_t _seq;
    IActions &_actions;

    StaticVector<Comm::Packet, 10> _pkt_buffer_sent;
    StaticVector<Comm::Packet, 10> _pkt_buffer_rcvd;
};